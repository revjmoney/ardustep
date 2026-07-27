# ardustep — a USB Arduino step generator for LinuxCNC (for science)

A deliberately-experimental rig that makes a **$14 ATmega328 Arduino (Uno/Nano)**
act as a 3-axis step generator for **LinuxCNC over plain USB serial**.

> **Read this first — the honest part.** LinuxCNC normally refuses USB step
> generators for a real reason: its motion controller wants to exchange a
> setpoint with the hardware every servo period (1 ms), *deterministically*, and
> USB is host-scheduled and jittery. Real boards (Mesa, Remora, the PicoBOB DLX)
> use a deterministic **wired Ethernet/SPI** link instead. This project ignores
> that on purpose, to see how far a thin-buffer USB design gets. It is a
> learning/measurement rig, **not** a controller for production cutting. The
> "correct" cheap version of this idea is a Raspberry Pi Pico + W5500 running
> Remora over Ethernet.

## How it dodges (some of) the USB problem

The Arduino runs its **own step-pulse timing on-chip** (a Timer1 DDS at 30 kHz),
so the USB link only has to deliver *position setpoints*, not microsecond-accurate
step pulses. LinuxCNC streams an absolute target (in steps) once per cycle; the
firmware sweeps each axis from its current position to that target over one host
interval (first-order interpolation, "thin buffer"). The board echoes its actual
step count back every cycle so LinuxCNC closes the position loop and enforces
following error. USB jitter shows up as velocity ripple and the occasional
following-error trip — which is exactly what this rig lets you *measure*.

```
LinuxCNC motion (1 kHz servo thread)
   │  joint.N.motor-pos-cmd / motor-pos-fb
   ▼
ardustep.py        userspace, NON-realtime HAL component (pyserial)
   │  ==== protocol.h / protocol.py : the frozen wire contract ====
   ▼  USB CDC serial @ 1 Mbaud
ATmega328 firmware  Timer1 DDS step generator
   ▼  STEP/DIR
external stepper drivers
```

## Repository layout

| Path | What it is |
|------|------------|
| `firmware/ardustep/protocol.h` | **Frozen contract**: packet structs + CRC-8 |
| `firmware/ardustep/config.h` | Pin map, ISR rate, baud, watchdog |
| `firmware/ardustep/stepgen.{h,cpp}` | Timer1 DDS step generator + ISR |
| `firmware/ardustep/ardustep.ino` | Main loop: parse → interpolate → reply → watchdog |
| `linuxcnc/protocol.py` | Python mirror of `protocol.h` (must match byte-for-byte) |
| `linuxcnc/ardustep.py` | Userspace HAL component |
| `linuxcnc/ardustep.hal` | HAL wiring to `motion` |
| `linuxcnc/ardustep.ini` | 3-axis machine config (loose FERROR) |
| `firmware/ardustep/platformio.ini` | PlatformIO build (nano / nano_new / uno) |
| `tools/bench_test.py` | Standalone firmware tester / jitter meter (no LinuxCNC) |
| `tools/plot_bench.py` | Turn a bench CSV into tracking / latency / jitter graphs |

## Wiring (Arduino Uno/Nano)

| Signal | Pin | Port bit |
|--------|-----|----------|
| X STEP | D2 | PD2 |
| Y STEP | D3 | PD3 |
| Z STEP | D4 | PD4 |
| X DIR  | D5 | PD5 |
| Y DIR  | D6 | PD6 |
| Z DIR  | D7 | PD7 |
| X/Y/Z limit | A0/A1/A2 | PC0/1/2 (active-low, internal pull-up) |
| Driver ENABLE | D8 | PB0 |
| Spindle on/off | D9 | PB1 |

All STEP and DIR lines share PORTD so every axis steps on the same instruction.
D0/D1 are left for USB serial. Polarity inversions live in `config.h`
(`DIR_INVERT_MASK`, `STEP_ACTIVE_HIGH`, `ENABLE_ACTIVE_HIGH`).

## Build & flash the firmware

PlatformIO (recommended):

```sh
cd firmware/ardustep
pio run                    # build default env (nano, old bootloader)
pio run -e nano -t upload  # build + flash; -e nano_new or -e uno as needed
```

Arduino IDE: open `firmware/ardustep/ardustep.ino` (the `.h`/`.cpp` files are
picked up automatically), select your board/port, upload.

Or arduino-cli:

```sh
arduino-cli compile --fqbn arduino:avr:nano firmware/ardustep
arduino-cli upload  --fqbn arduino:avr:nano -p /dev/ttyUSB0 firmware/ardustep
```

## Bench-test the firmware first (no LinuxCNC)

```sh
pip install pyserial
python3 tools/bench_test.py --device /dev/ttyUSB0 --mode sine \
    --axis 0 --amp 10 --freq 0.5 --spu 200 --period 1 --duration 20
```

Put a logic analyzer on D2/D5 and confirm: step frequency tracks the commanded
velocity, DIR flips at the turnaround, `fb(steps)` chases `cmd(steps)`, and the
summary shows a low miss rate and sane latency. Yank the USB mid-run — motion
should stop and the board should report `fault` (watchdog).

### Graph the run (the actual "science")

Capture a run to CSV, then plot it:

```sh
pip install matplotlib
python3 tools/bench_test.py --device /dev/ttyUSB0 --mode sine --axis 0 \
    --amp 10 --freq 0.5 --spu 200 --period 1 --duration 30 --csv run.csv
python3 tools/plot_bench.py run.csv            # writes run.png
```

`plot_bench.py` produces a 2x2 figure — position tracking (cmd vs actual),
following error, round-trip latency over time (with dropped replies marked and
the host period line), and a latency histogram with p50/p95/p99/max markers —
and prints miss rate + latency percentiles. This is how you quantify, in numbers
and pictures, exactly how much the USB link jitters and whether the thin buffer
is keeping up.

## Run it under LinuxCNC

1. Copy `linuxcnc/ardustep.py`, `linuxcnc/protocol.py`, `ardustep.hal` and
   `ardustep.ini` into a LinuxCNC config directory
   (e.g. `~/linuxcnc/configs/ardustep/`).
2. Edit the `--device` and `--spu` values in `ardustep.hal` for your machine.
3. `pip install pyserial` for the Python that LinuxCNC uses.
4. Launch: `linuxcnc ~/linuxcnc/configs/ardustep/ardustep.ini`.
5. Bring-up order: keep motors disconnected first, enable the machine, jog X/Y/Z
   in the Axis GUI, and confirm the step pins pulse (LED/analyzer) and `pos-fb`
   tracks `pos-cmd`. Then connect current-limited drivers with no load.

## The frozen contract

`protocol.h` and `protocol.py` define identical little-endian packed packets:

* **PC → MCU (18 B):** magic, seq, flags, `pos_cmd[3]` (i32 steps), spindle, CRC-8
* **MCU → PC (19 B):** magic, seq, status, limits, `pos_fb[3]` (i32 steps),
  underruns, CRC-8

Change one side → change the other in the same commit and bump `PROTO_VERSION`.
Because the firmware only cares about this contract, the Python driver can later
be swapped for a C `halcompile` component that `#include`s `protocol.h` directly,
with no firmware changes.

## Known limitations (by design)

* The host-side driver is **userspace and non-realtime** — jitter is expected,
  FERROR is loose, not for production.
* USB stalls → buffer underruns → velocity ripple; the thin buffer makes them
  visible (that's the experiment).
* ATmega328 ceiling is roughly **10–20 kHz/axis**; 3 axes for now.
* Want it to *actually* work well and cheaply? Pico + W5500 + Remora over
  Ethernet. This repo exists to understand *why* that's the answer.
