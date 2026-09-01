# Ardustep v2 — a USB Arduino step generator for LinuxCNC (for science)

> **UNTESTED PROTOTYPE.** Ardustep v2 has passed host-side tests and AVR build
> checks, but it has not been validated on physical hardware. Do not connect it
> to a spindle or a machine capable of causing damage. Perform the isolated
> logic-analyzer bench procedure below first.

Ardustep deliberately asks a questionable engineering question: how far can a
$14 ATmega328P Uno/Nano get as a three-axis LinuxCNC STEP/DIR generator over
plain USB serial? It is a learning and measurement rig, not a production motion
controller or safety system. USB and the Python userspace component are
nondeterministic; Mesa, Remora, and wired real-time transports exist for good
reasons. V2 keeps the thin-buffer USB experiment so its limitations can be
measured honestly.

## Architecture

```text
LinuxCNC motion (1 kHz servo thread)
        |
        v
linuxcnc/ardustep.py (userspace HAL + pyserial)
        |
        v
USB serial at 1 Mbaud, absolute positions every 1.000 ms
        |
        v
firmware/ardustep/ardustep.ino
        |
        v
Timer1 DDS at nominal 30 kHz -> STEP / DIR / ENABLE
```

The AVR interpolates from its actual step count toward each absolute target and
returns actual counts and status. The buffer is intentionally only one command
interval deep: USB jitter remains visible as velocity ripple, following error,
timeouts, and saturation.

## V2 timing and safety contract

- The command interval is fixed at **1.000 ms**. Firmware, HAL component, bench
  tool, and startup INFO data must agree; `--period` values other than `1.0` are
  rejected.
- Timer1 runs at nominal **30 kHz** (one tick is approximately 33.3 us).
- STEP is high for one tick, followed by at least one complete low tick. The
  conservative maximum is therefore **15,000 steps/s per axis**. Commands above
  15 steps per 1 ms interval are saturated and increment the underrun counter.
- A direction reversal discards fractional phase, waits for the previous STEP
  hold interval, changes DIR in the ISR, then observes at least one setup tick
  before a new STEP. Ordinary same-direction retargets preserve DDS phase.
- ESTOP and watchdog events stop pulses immediately, turn physical driver ENABLE
  and spindle outputs off, and latch fault. Ordinary enabled motion never clears
  the latch. Clear it only with a disabled `FLAG_CLEAR_FAULT` transaction; the
  LinuxCNC component performs this deliberate sequence at startup and exposes
  `ardustep.reset-fault` for recovery afterward. Its optional active-high
  `ardustep.estop` HAL input transmits the latched ESTOP command.
- `ST_ENABLED` reports the actual physical ENABLE output. ESTOP plus ENABLE can
  never report enabled.
- A **50 ms** valid-command watchdog stops and latches the board if the host or
  USB link disappears.

This is not a certified ESTOP or safety controller. Hardware enable chains and
proper machine safety remain external requirements.

## Protocol v2

`firmware/ardustep/protocol.h` and `linuxcnc/protocol.py` are byte-for-byte
mirrors. Motion packets remain compact:

- PC to MCU command: 18 bytes (`0xA5`, sequence, flags, three signed positions,
  spindle value, CRC-8).
- MCU to PC feedback: 19 bytes (`0x5A`, echoed sequence, status, limits, three
  signed positions, cumulative saturation count, CRC-8).

Before motion, the host sends a four-byte HELLO and requires a 17-byte INFO
reply. INFO reports protocol/firmware versions, axes, ISR rate, command period,
safe step-rate ceiling, and watchdog interval. A mismatch refuses to arm.

For each command, the host reads until the matching modulo-256 sequence arrives
or the deadline expires. Stale and unexpected future replies are discarded and
counted. Bad-CRC packets never satisfy a transaction. Latency means command send
to its matching feedback—not merely the first bytes received.

`underruns`/`ST_UNDERRUN` mean the requested one-interval move exceeded the
15-step safe pulse capacity and was rate-saturated. The count is cumulative and
does not claim that USB itself dropped a packet.

## Repository layout

| Path | Purpose |
|---|---|
| `firmware/ardustep/protocol.h` | AVR wire contract |
| `firmware/ardustep/config.h` | Pin map and timing constants |
| `firmware/ardustep/stepgen.{h,cpp}` | Atomic DDS and pulse/DIR state machine |
| `firmware/ardustep/ardustep.ino` | Framing, handshake, fault state, feedback |
| `linuxcnc/protocol.py` | Python protocol mirror and tested framers |
| `linuxcnc/ardustep.py` | Non-realtime LinuxCNC HAL component |
| `tools/bench_test.py` | Standalone matching-transaction/jitter bench tool |
| `tools/plot_bench.py` | CSV tracking and latency plots |
| `tests/` | Hardware-free protocol and state-model tests |

## Wiring (Uno/Nano)

| Signal | Pin |
|---|---|
| X/Y/Z STEP | D2 / D3 / D4 |
| X/Y/Z DIR | D5 / D6 / D7 |
| X/Y/Z limit | A0 / A1 / A2, active-low with pull-ups |
| Driver ENABLE | D8 |
| Spindle on/off | D9 |

Polarity settings are in `firmware/ardustep/config.h`. D0/D1 remain dedicated
to serial.

## Test and build

No hardware or LinuxCNC is needed for the Python suite:

```sh
python -m unittest discover -s tests -v
python -m py_compile linuxcnc/protocol.py linuxcnc/ardustep.py \
  tools/bench_test.py tools/plot_bench.py
```

Build all supported AVR targets:

```sh
cd firmware/ardustep
pio run -e nano -e nano_new -e uno
```

Build success is not hardware validation.

## First-hardware bench procedure

1. Power the Arduino by USB only; disconnect drivers, motors, spindle, and load.
2. Flash the correct Nano/Uno PlatformIO environment.
3. Run the bench tool and verify that HELLO/INFO plus its no-motion ESTOP latch,
   enable-rejection, and disabled-clear diagnostics succeed:

   ```sh
   python tools/bench_test.py --device /dev/ttyUSB0 --mode sine \
     --axis 0 --amp 10 --freq 0.5 --spu 200 --period 1 --duration 20 \
     --csv run.csv
   ```

4. Probe STEP and DIR with a scope or logic analyzer. Start with one axis and a
   very low rate; confirm that slow steps are not starved by repeated commands.
5. Reverse direction and confirm DIR hold/setup before the next STEP.
6. Sweep up to 15 ksteps/s. Confirm discrete pulses with roughly one tick high
   and at least one complete tick low—never a merged or stuck-high waveform.
7. Disconnect USB and confirm the watchdog removes ENABLE and latches fault.
8. Send ESTOP with ENABLE set; confirm no STEP, spindle, or ENABLE output.
9. Send ordinary enabled commands and confirm the fault remains latched. Then
   perform disabled clear-fault and confirm the next enable is accepted.
10. Inject delayed, out-of-order, corrupt, and junk-framed replies in a mock or
    serial proxy; verify the reported stale/CRC/resync/timeout counters.
11. Only after those checks, connect one current-limited driver and unloaded
    motor at safe voltage/current. Integrate LinuxCNC last.

The bench summary reports TX, matched RX, timeouts, stale/future replies, CRC and
resync events, p50/p95/p99/max matching latency, command interval/jitter, maximum
following error, and saturation. Its CSV retains the corresponding per-cycle
raw fields. `tools/plot_bench.py run.csv` creates the tracking/latency plot.

## LinuxCNC bring-up

Copy the four files from `linuxcnc/` into a LinuxCNC configuration directory,
adjust device and steps-per-unit in `ardustep.hal`, and start LinuxCNC only after
the isolated bench succeeds. The included INI uses a 1 ms servo period and loose
following-error limits because this link is non-realtime.

After a runtime fault, disable the machine, pulse the reset pin, release it, and
then re-enable:

```sh
halcmd setp ardustep.reset-fault true
halcmd setp ardustep.reset-fault false
```

Do not tighten following-error settings or claim performance until real captures
show what this particular USB host, adapter, and workload actually do.
