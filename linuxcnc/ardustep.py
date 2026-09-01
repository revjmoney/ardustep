#!/usr/bin/env python3
"""Non-realtime LinuxCNC HAL component for the experimental Ardustep v2."""
import argparse
import signal
import sys
import time

import serial
import hal
import protocol as P


def log(message):
    sys.stderr.write("ardustep: %s\n" % message)
    sys.stderr.flush()


def read_next(ser, framer, deadline):
    while time.monotonic() < deadline:
        packet = framer.next()
        if packet is not None:
            return packet
        framer.feed(ser.read(max(1, ser.in_waiting)))
    return framer.next()


def handshake(ser, timeout_s=0.25):
    seq = 0x42
    framer = P.PacketFramer(P.INFO_MAGIC, P.INFO_PACKET_LEN, P.unpack_info)
    ser.write(P.pack_hello(seq))
    info = read_next(ser, framer, time.monotonic() + timeout_s)
    if info is None:
        raise RuntimeError("no valid INFO response from firmware")
    if info["seq"] != seq:
        raise RuntimeError("INFO sequence mismatch: got %d expected %d" %
                           (info["seq"], seq))
    problem = P.validate_info(info)
    if problem:
        raise RuntimeError(problem)
    return info


def read_matching(ser, framer, matcher, expected, deadline):
    while time.monotonic() < deadline:
        packet = framer.next()
        while packet is not None:
            matched = matcher.consider(packet, expected)
            if matched is not None:
                return matched
            packet = framer.next()
        framer.feed(ser.read(max(1, ser.in_waiting)))
    packet = framer.next()
    while packet is not None:
        matched = matcher.consider(packet, expected)
        if matched is not None:
            return matched
        packet = framer.next()
    return None


def parse_args(argv):
    parser = argparse.ArgumentParser(description="USB Arduino step-gen HAL component")
    parser.add_argument("--name", default="ardustep")
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--period", type=float, default=1.0,
                        help="fixed v2 cadence in ms (must be 1.0)")
    parser.add_argument("--spu", default="200,200,200")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    if abs(args.period * 1000.0 - P.COMMAND_INTERVAL_US) > 1e-9:
        log("v2 requires --period %.3f ms" % (P.COMMAND_INTERVAL_US / 1000.0))
        return 2
    spu = [float(value) for value in args.spu.split(",")]
    if len(spu) != P.NUM_AXES or any(value <= 0 for value in spu):
        log("--spu needs %d positive values" % P.NUM_AXES)
        return 2
    period_s = P.COMMAND_INTERVAL_US / 1000000.0

    try:
        ser = serial.Serial(args.device, args.baud, timeout=period_s)
        time.sleep(2.0)
        ser.reset_input_buffer()
        info = handshake(ser)
    except (serial.SerialException, RuntimeError) as exc:
        log("startup failed: %s" % exc)
        try:
            ser.close()
        except Exception:
            pass
        return 1
    log("firmware %d.%d, protocol %d, %d Hz ISR, %d step/s ceiling" %
        (info["fw_version"][0], info["fw_version"][1], info["proto_version"],
         info["isr_hz"], info["max_step_rate"]))

    framer = P.PacketFramer(P.FB_MAGIC, P.FB_PACKET_LEN, P.unpack_feedback)
    matcher = P.SequenceMatcher()
    seq = 1
    ser.write(P.pack_command(seq, P.FLAG_CLEAR_FAULT, [0, 0, 0], 0))
    cleared = read_matching(ser, framer, matcher, seq,
                            time.monotonic() + 2.0 * period_s)
    if cleared is None or cleared["status"] & (P.ST_FAULT | P.ST_ENABLED):
        log("startup fault-clear handshake failed")
        ser.close()
        return 1

    component = hal.component(args.name)
    for joint in range(P.NUM_AXES):
        component.newpin("joint.%d.pos-cmd" % joint, hal.HAL_FLOAT, hal.HAL_IN)
        component.newpin("joint.%d.pos-fb" % joint, hal.HAL_FLOAT, hal.HAL_OUT)
        component.newpin("limit.%d" % joint, hal.HAL_BIT, hal.HAL_OUT)
    for name in ("enable", "estop", "spindle-on", "reset-fault"):
        component.newpin(name, hal.HAL_BIT, hal.HAL_IN)
    component.newpin("spindle-speed", hal.HAL_FLOAT, hal.HAL_IN)
    for name in ("fault", "running", "connected"):
        component.newpin(name, hal.HAL_BIT, hal.HAL_OUT)
    for name in ("underruns", "stale-replies", "unexpected-replies",
                 "crc-failures", "resync-events", "timeouts", "matched-replies"):
        component.newpin(name, hal.HAL_S32, hal.HAL_OUT)
    component.ready()

    timeouts = 0
    was_connected = None
    running = {"go": True}

    def stop(_signal, _frame):
        running["go"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    next_tick = time.monotonic()
    try:
        while running["go"]:
            flags = 0
            if component["enable"]:
                flags |= P.FLAG_ENABLE
            if component["estop"]:
                flags |= P.FLAG_ESTOP
            if component["spindle-on"]:
                flags |= P.FLAG_SPINDLE
            if component["reset-fault"] and not component["enable"]:
                flags |= P.FLAG_CLEAR_FAULT
            positions = [int(round(component["joint.%d.pos-cmd" % j] * spu[j]))
                         for j in range(P.NUM_AXES)]
            speed = max(0.0, min(1.0, component["spindle-speed"]))
            seq = (seq + 1) & 0xFF
            ser.write(P.pack_command(seq, flags, positions, int(speed * 0xFFFF)))

            feedback = read_matching(
                ser, framer, matcher, seq, time.monotonic() + 2.0 * period_s)
            connected = feedback is not None
            component["connected"] = connected
            if feedback is not None:
                for joint in range(P.NUM_AXES):
                    component["joint.%d.pos-fb" % joint] = \
                        feedback["pos_fb"][joint] / spu[joint]
                    component["limit.%d" % joint] = bool(
                        feedback["limits"] & (1 << joint))
                component["fault"] = bool(feedback["status"] & P.ST_FAULT)
                component["running"] = bool(feedback["status"] & P.ST_RUNNING)
                component["underruns"] = feedback["underruns"]
            else:
                timeouts += 1
                component["fault"] = True

            component["stale-replies"] = matcher.stale
            component["unexpected-replies"] = matcher.future
            component["crc-failures"] = framer.crc_failures
            component["resync-events"] = framer.resync_events
            component["timeouts"] = timeouts
            component["matched-replies"] = matcher.matched
            if connected != was_connected:
                log("link up" if connected else "link DOWN (matching reply timeout)")
                was_connected = connected

            next_tick += period_s
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(P.pack_command((seq + 1) & 0xFF, 0, [0, 0, 0], 0))
            ser.flush()
        except Exception:
            pass
        ser.close()
        log("exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
