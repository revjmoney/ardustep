#!/usr/bin/env python3
"""
ardustep.py -- userspace LinuxCNC HAL component for the USB Arduino step gen.

This is a NON-REALTIME, free-running userspace component. It is deliberately not
added to a HAL real-time thread (it can't be -- it does blocking USB I/O). It
paces its own loop at --period and exchanges one packet per cycle with the
firmware. That is the whole "for science" caveat: timing is best-effort, so the
.ini must use a loose FERROR.

Why a single self-paced loop instead of the worker-thread split sketched in the
plan: this component is the only thing touching these HAL pins, so there is no
other HAL work to keep alive during a serial stall. A bounded read timeout caps
each cycle, and pacing off a monotonic clock keeps cadence. A background thread
would only add GIL/locking surface for no benefit here.

HAL pins (prefixed with the component name, default "ardustep"):
    joint.N.pos-cmd   float in    commanded position, machine units (N=0..2)
    joint.N.pos-fb    float out   actual position echoed by the MCU
    enable            bit   in    master enable (wire to motion.motion-enabled)
    spindle-on        bit   in    spindle on/off
    spindle-speed     float in    0..1 spindle duty (maps to u16)
    fault             bit   out    MCU watchdog/estop latched
    running           bit   out    at least one axis moving
    connected         bit   out    valid feedback seen this cycle
    limit.N           bit   out    limit switch N pressed
    underruns         s32   out    cumulative MCU underrun events
"""
import argparse
import signal
import sys
import time

import serial  # pyserial

import hal

import protocol as P


def log(msg):
    sys.stderr.write("ardustep: %s\n" % msg)
    sys.stderr.flush()


class FeedbackFramer:
    """Accumulates serial bytes and extracts CRC-valid FeedbackPackets."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk):
        if chunk:
            self._buf.extend(chunk)

    def next(self):
        """Return the next valid feedback dict, or None if not yet available."""
        while True:
            i = self._buf.find(bytes([P.FB_MAGIC]))
            if i < 0:
                self._buf.clear()
                return None
            if i > 0:
                del self._buf[:i]
            if len(self._buf) < P.FB_PACKET_LEN:
                return None
            frame = bytes(self._buf[:P.FB_PACKET_LEN])
            fb = P.unpack_feedback(frame)
            if fb is not None:
                del self._buf[:P.FB_PACKET_LEN]
                return fb
            # false magic / bad CRC -> drop one byte and resync
            del self._buf[:1]


def parse_args(argv):
    ap = argparse.ArgumentParser(description="USB Arduino step-gen HAL component")
    ap.add_argument("--name", default="ardustep", help="HAL component name")
    ap.add_argument("--device", required=True, help="serial device, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--period", type=float, default=1.0,
                    help="loop period in milliseconds (host cycle)")
    ap.add_argument("--spu", default="200,200,200",
                    help="steps per machine unit, comma list per joint")
    return ap.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    spu = [float(x) for x in args.spu.split(",")]
    if len(spu) != P.NUM_AXES:
        log("--spu needs %d values, got %d" % (P.NUM_AXES, len(spu)))
        return 2
    period_s = args.period / 1000.0

    try:
        ser = serial.Serial(args.device, args.baud, timeout=period_s)
    except serial.SerialException as e:
        log("cannot open %s: %s" % (args.device, e))
        return 1
    # Give the board its post-reset bootloader moment, then flush.
    time.sleep(2.0)
    ser.reset_input_buffer()

    c = hal.component(args.name)
    for j in range(P.NUM_AXES):
        c.newpin("joint.%d.pos-cmd" % j, hal.HAL_FLOAT, hal.HAL_IN)
        c.newpin("joint.%d.pos-fb" % j, hal.HAL_FLOAT, hal.HAL_OUT)
        c.newpin("limit.%d" % j, hal.HAL_BIT, hal.HAL_OUT)
    c.newpin("enable", hal.HAL_BIT, hal.HAL_IN)
    c.newpin("spindle-on", hal.HAL_BIT, hal.HAL_IN)
    c.newpin("spindle-speed", hal.HAL_FLOAT, hal.HAL_IN)
    c.newpin("fault", hal.HAL_BIT, hal.HAL_OUT)
    c.newpin("running", hal.HAL_BIT, hal.HAL_OUT)
    c.newpin("connected", hal.HAL_BIT, hal.HAL_OUT)
    c.newpin("underruns", hal.HAL_S32, hal.HAL_OUT)
    c.ready()

    framer = FeedbackFramer()
    seq = 0
    was_connected = None

    running = {"go": True}

    def stop(_sig, _frame):
        running["go"] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    next_tick = time.monotonic()
    try:
        while running["go"]:
            # ---- build & send command from current HAL pin state ----------
            flags = 0
            if c["enable"]:
                flags |= P.FLAG_ENABLE
            if c["spindle-on"]:
                flags |= P.FLAG_SPINDLE
            pos_cmd = [int(round(c["joint.%d.pos-cmd" % j] * spu[j]))
                       for j in range(P.NUM_AXES)]
            spd = max(0.0, min(1.0, c["spindle-speed"]))
            seq = (seq + 1) & 0xFF
            ser.write(P.pack_command(seq, flags, pos_cmd, int(spd * 0xFFFF)))

            # ---- wait for this cycle's feedback (bounded) -----------------
            deadline = time.monotonic() + 2.0 * period_s
            fb = None
            while time.monotonic() < deadline:
                framer.feed(ser.read(max(1, ser.in_waiting)))
                fb = framer.next()
                if fb is not None:
                    break

            connected = fb is not None
            c["connected"] = connected
            if connected:
                for j in range(P.NUM_AXES):
                    c["joint.%d.pos-fb" % j] = fb["pos_fb"][j] / spu[j]
                c["fault"] = bool(fb["status"] & P.ST_FAULT)
                c["running"] = bool(fb["status"] & P.ST_RUNNING)
                c["underruns"] = fb["underruns"]
                c["limit.0"] = bool(fb["limits"] & P.LIM_X)
                c["limit.1"] = bool(fb["limits"] & P.LIM_Y)
                c["limit.2"] = bool(fb["limits"] & P.LIM_Z)
            else:
                # No reply: hold last feedback but flag fault so motion stops.
                c["fault"] = True

            if connected != was_connected:
                log("link up" if connected else "link DOWN (no feedback)")
                was_connected = connected

            # ---- pace the loop --------------------------------------------
            next_tick += period_s
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()  # we fell behind; resync cadence
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: tell the board to disable, then release.
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
