#!/usr/bin/env python3
"""
bench_test.py -- drive the ardustep firmware WITHOUT LinuxCNC.

Use this for firmware bring-up: it streams position setpoints over USB serial at
a fixed cadence and reports what the board echoes back, so you can:
  * watch STEP/DIR on a logic analyzer and confirm frequency + direction,
  * confirm the feedback step count tracks what you commanded,
  * measure round-trip latency and jitter,
  * watch the underrun counter climb when you push past the step ceiling,
  * exercise the watchdog (Ctrl-C: motion stops, board faults).

Examples:
  # Sine on X (axis 0), +/-10 units at 0.5 Hz, 1 ms cadence, 200 steps/unit:
  python3 bench_test.py --device /dev/ttyUSB0 --mode sine --axis 0 \
      --amp 10 --freq 0.5 --spu 200 --period 1 --duration 20

  # Trapezoidal there-and-back ramp on Z (axis 2):
  python3 bench_test.py --device /dev/ttyUSB0 --mode ramp --axis 2 --amp 25
"""
import argparse
import csv
import math
import os
import sys
import time

import serial

# protocol.py lives in ../linuxcnc relative to this file.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "linuxcnc"))
import protocol as P  # noqa: E402


def parse_args(argv):
    ap = argparse.ArgumentParser(description="ardustep firmware bench tester")
    ap.add_argument("--device", required=True)
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--period", type=float, default=1.0, help="cadence in ms")
    ap.add_argument("--spu", type=float, default=200.0, help="steps per unit")
    ap.add_argument("--axis", type=int, default=0, choices=range(P.NUM_AXES))
    ap.add_argument("--mode", choices=["sine", "ramp", "hold"], default="sine")
    ap.add_argument("--amp", type=float, default=10.0, help="amplitude, units")
    ap.add_argument("--freq", type=float, default=0.5, help="sine Hz")
    ap.add_argument("--duration", type=float, default=15.0, help="seconds")
    ap.add_argument("--csv", default=None,
                    help="write a per-cycle log to this CSV (feed it to plot_bench.py)")
    return ap.parse_args(argv)


def target_units(mode, t, amp, freq, duration):
    if mode == "hold":
        return amp
    if mode == "sine":
        return amp * math.sin(2.0 * math.pi * freq * t)
    # ramp: 0 -> +amp -> -amp -> 0 over the duration (triangle)
    phase = (t / duration) % 1.0
    if phase < 0.25:
        return amp * (phase / 0.25)
    if phase < 0.75:
        return amp * (1.0 - (phase - 0.25) / 0.25)
    return amp * (-1.0 + (phase - 0.75) / 0.25)


def main(argv):
    args = parse_args(argv)
    period_s = args.period / 1000.0

    ser = serial.Serial(args.device, args.baud, timeout=period_s)
    time.sleep(2.0)            # bootloader settle
    ser.reset_input_buffer()

    buf = bytearray()

    def read_feedback(deadline):
        while time.monotonic() < deadline:
            buf.extend(ser.read(max(1, ser.in_waiting)))
            while True:
                i = buf.find(bytes([P.FB_MAGIC]))
                if i < 0:
                    buf.clear()
                    break
                if i > 0:
                    del buf[:i]
                if len(buf) < P.FB_PACKET_LEN:
                    break
                fb = P.unpack_feedback(bytes(buf[:P.FB_PACKET_LEN]))
                if fb is not None:
                    del buf[:P.FB_PACKET_LEN]
                    return fb
                del buf[:1]
        return None

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["cycle", "t_s", "cmd_steps", "fb_steps",
                             "err_steps", "underruns", "latency_us", "link"])

    seq = 0
    cycles = replies = 0
    lat_sum = lat_max = 0.0
    t0 = time.monotonic()
    next_tick = t0
    last_print = t0

    print("cycle  cmd(steps)  fb(steps)  err  underruns  lat_us  link")
    try:
        while True:
            now = time.monotonic()
            t = now - t0
            if t >= args.duration:
                break

            u = target_units(args.mode, t, args.amp, args.freq, args.duration)
            steps = int(round(u * args.spu))
            pos = [0, 0, 0]
            pos[args.axis] = steps

            seq = (seq + 1) & 0xFF
            sent_at = time.monotonic()
            ser.write(P.pack_command(seq, P.FLAG_ENABLE, pos, 0))
            cycles += 1

            fb = read_feedback(sent_at + 2.0 * period_s)
            lat = fbsteps = err = underruns = None
            if fb is not None:
                replies += 1
                lat = (time.monotonic() - sent_at) * 1e6
                lat_sum += lat
                lat_max = max(lat_max, lat)
                fbsteps = fb["pos_fb"][args.axis]
                err = steps - fbsteps
                underruns = fb["underruns"]
                if now - last_print >= 0.25:
                    print("%5d  %9d  %9d  %4d  %8d  %6.0f  %s" % (
                        cycles, steps, fbsteps, err, underruns, lat,
                        "fault" if fb["status"] & P.ST_FAULT else "ok"))
                    last_print = now

            if csv_writer is not None:
                csv_writer.writerow([
                    cycles, "%.6f" % t, steps,
                    "" if fbsteps is None else fbsteps,
                    "" if err is None else err,
                    "" if underruns is None else underruns,
                    "" if lat is None else "%.1f" % lat,
                    1 if fb is not None else 0,
                ])

            next_tick += period_s
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ser.write(P.pack_command((seq + 1) & 0xFF, 0, [0, 0, 0], 0))
        ser.flush()
        ser.close()
        if csv_file is not None:
            csv_file.close()
            print("wrote %s" % args.csv)

    miss = cycles - replies
    print("\n--- summary ---")
    print("cycles sent     : %d" % cycles)
    print("replies received: %d  (%.1f%% missed)" %
          (replies, 100.0 * miss / cycles if cycles else 0.0))
    if replies:
        print("latency  mean   : %.0f us" % (lat_sum / replies))
        print("latency  max    : %.0f us" % lat_max)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
