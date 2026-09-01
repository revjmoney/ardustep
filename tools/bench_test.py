#!/usr/bin/env python3
"""Hardware bench/jitter tool for Ardustep v2 (no LinuxCNC required)."""
import argparse
import csv
import math
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "linuxcnc"))
import protocol as P  # noqa: E402


def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    position = (len(values) - 1) * q / 100.0
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    fraction = position - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def read_next(ser, framer, deadline):
    while time.monotonic() < deadline:
        packet = framer.next()
        if packet is not None:
            return packet
        framer.feed(ser.read(max(1, ser.in_waiting)))
    return framer.next()


def read_matching(ser, framer, matcher, expected, deadline):
    while time.monotonic() < deadline:
        packet = framer.next()
        while packet is not None:
            matched = matcher.consider(packet, expected)
            if matched is not None:
                return matched
            packet = framer.next()
        framer.feed(ser.read(max(1, ser.in_waiting)))
    return None


def startup(ser):
    info_framer = P.PacketFramer(P.INFO_MAGIC, P.INFO_PACKET_LEN, P.unpack_info)
    hello_seq = 0x42
    ser.write(P.pack_hello(hello_seq))
    info = read_next(ser, info_framer, time.monotonic() + 0.25)
    if info is None or info["seq"] != hello_seq:
        raise RuntimeError("no matching INFO reply")
    problem = P.validate_info(info)
    if problem:
        raise RuntimeError(problem)
    return info


def parse_args(argv):
    parser = argparse.ArgumentParser(description="ardustep v2 firmware bench tester")
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--period", type=float, default=1.0,
                        help="fixed v2 cadence in ms (must be 1.0)")
    parser.add_argument("--spu", type=float, default=200.0)
    parser.add_argument("--axis", type=int, default=0, choices=range(P.NUM_AXES))
    parser.add_argument("--mode", choices=["sine", "ramp", "hold"], default="sine")
    parser.add_argument("--amp", type=float, default=10.0)
    parser.add_argument("--freq", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--csv", default=None)
    return parser.parse_args(argv)


def target_units(mode, elapsed, amplitude, frequency, duration):
    if mode == "hold":
        return amplitude
    if mode == "sine":
        return amplitude * math.sin(2.0 * math.pi * frequency * elapsed)
    phase = (elapsed / duration) % 1.0
    if phase < 0.25:
        return amplitude * (phase / 0.25)
    if phase < 0.75:
        return amplitude * (1.0 - (phase - 0.25) / 0.25)
    return amplitude * (-1.0 + (phase - 0.75) / 0.25)


def main(argv):
    args = parse_args(argv)
    if abs(args.period * 1000.0 - P.COMMAND_INTERVAL_US) > 1e-9:
        sys.stderr.write("v2 requires --period %.3f ms\n" %
                         (P.COMMAND_INTERVAL_US / 1000.0))
        return 2
    if args.spu <= 0:
        sys.stderr.write("--spu must be positive\n")
        return 2
    period_s = P.COMMAND_INTERVAL_US / 1000000.0

    ser = serial.Serial(args.device, args.baud, timeout=period_s)
    time.sleep(2.0)
    ser.reset_input_buffer()
    try:
        info = startup(ser)
    except RuntimeError as exc:
        ser.close()
        sys.stderr.write("startup failed: %s\n" % exc)
        return 1
    print("firmware %d.%d protocol %d; max %d step/s; watchdog %d ms" %
          (info["fw_version"][0], info["fw_version"][1], info["proto_version"],
           info["max_step_rate"], info["watchdog_ms"]))

    framer = P.PacketFramer(P.FB_MAGIC, P.FB_PACKET_LEN, P.unpack_feedback)
    matcher = P.SequenceMatcher()
    def transact(sequence, flags):
        ser.write(P.pack_command(sequence, flags, [0, 0, 0], 0))
        return read_matching(ser, framer, matcher, sequence,
                             time.monotonic() + 2.0 * period_s)

    # Prove the latch before generating any pulses: ESTOP wins over ENABLE,
    # ordinary enable cannot recover, and only disabled CLEAR_FAULT releases it.
    seq = 1
    estopped = transact(seq, P.FLAG_ESTOP | P.FLAG_ENABLE)
    seq = (seq + 1) & 0xFF
    still_faulted = transact(seq, P.FLAG_ENABLE)
    seq = (seq + 1) & 0xFF
    cleared = transact(seq, P.FLAG_CLEAR_FAULT)
    if (estopped is None or not estopped["status"] & P.ST_FAULT or
            estopped["status"] & P.ST_ENABLED or still_faulted is None or
            not still_faulted["status"] & P.ST_FAULT or
            still_faulted["status"] & P.ST_ENABLED or cleared is None or
            cleared["status"] & (P.ST_FAULT | P.ST_ENABLED)):
        ser.close()
        sys.stderr.write("ESTOP/fault-latch startup diagnostic failed\n")
        return 1
    print("ESTOP latch/enable rejection/disabled clear: PASS")

    csv_file = open(args.csv, "w", newline="") if args.csv else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow([
            "cycle", "seq", "t_s", "tx_interval_us", "cmd_steps", "fb_steps",
            "err_steps", "underruns", "latency_us", "link", "stale_total",
            "future_total", "crc_total", "resync_total", "status"])

    cycles = timeouts = 0
    latencies = []
    intervals = []
    max_error = 0
    max_underruns = 0
    start = next_tick = time.monotonic()
    previous_tx = None
    last_print = start
    print("cycle  cmd(steps)  fb(steps)  err  underruns  lat_us  link")
    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= args.duration:
                break
            units = target_units(args.mode, elapsed, args.amp, args.freq,
                                 args.duration)
            steps = int(round(units * args.spu))
            positions = [0, 0, 0]
            positions[args.axis] = steps
            seq = (seq + 1) & 0xFF
            sent_at = time.monotonic()
            interval_us = None if previous_tx is None else (sent_at - previous_tx) * 1e6
            previous_tx = sent_at
            if interval_us is not None:
                intervals.append(interval_us)
            ser.write(P.pack_command(seq, P.FLAG_ENABLE, positions, 0))
            cycles += 1

            feedback = read_matching(
                ser, framer, matcher, seq, sent_at + 2.0 * period_s)
            latency = fb_steps = error = underruns = status = None
            if feedback is None:
                timeouts += 1
            else:
                latency = (time.monotonic() - sent_at) * 1e6
                latencies.append(latency)
                fb_steps = feedback["pos_fb"][args.axis]
                error = steps - fb_steps
                max_error = max(max_error, abs(error))
                underruns = feedback["underruns"]
                max_underruns = max(max_underruns, underruns)
                status = feedback["status"]
                if now - last_print >= 0.25:
                    print("%5d  %9d  %9d  %4d  %8d  %6.0f  %s" % (
                        cycles, steps, fb_steps, error, underruns, latency,
                        "fault" if status & P.ST_FAULT else "ok"))
                    last_print = now

            if writer:
                writer.writerow([
                    cycles, seq, "%.6f" % elapsed,
                    "" if interval_us is None else "%.1f" % interval_us,
                    steps, "" if fb_steps is None else fb_steps,
                    "" if error is None else error,
                    "" if underruns is None else underruns,
                    "" if latency is None else "%.1f" % latency,
                    int(feedback is not None), matcher.stale, matcher.future,
                    framer.crc_failures, framer.resync_events,
                    "" if status is None else status])

            next_tick += period_s
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        try:
            ser.write(P.pack_command((seq + 1) & 0xFF, 0, [0, 0, 0], 0))
            ser.flush()
        finally:
            ser.close()
            if csv_file:
                csv_file.close()

    print("\n--- summary (matching transactions only) ---")
    print("TX count            : %d" % cycles)
    print("matched RX count    : %d" % len(latencies))
    print("timeouts/missed     : %d" % timeouts)
    print("stale replies       : %d" % matcher.stale)
    print("unexpected future   : %d" % matcher.future)
    print("CRC failures        : %d" % framer.crc_failures)
    print("framing/resync      : %d" % framer.resync_events)
    print("max following error : %d steps" % max_error)
    print("underrun/saturation : %d" % max_underruns)
    if latencies:
        print("latency us p50/p95/p99/max: %.0f / %.0f / %.0f / %.0f" % (
            percentile(latencies, 50), percentile(latencies, 95),
            percentile(latencies, 99), max(latencies)))
    if intervals:
        jitter = [value - P.COMMAND_INTERVAL_US for value in intervals]
        print("TX interval us p50/p95/p99/max: %.0f / %.0f / %.0f / %.0f" % (
            percentile(intervals, 50), percentile(intervals, 95),
            percentile(intervals, 99), max(intervals)))
        print("max absolute TX jitter: %.0f us" % max(abs(value) for value in jitter))
    if args.csv:
        print("wrote %s" % args.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
