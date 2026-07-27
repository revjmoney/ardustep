#!/usr/bin/env python3
"""
plot_bench.py -- turn a bench_test.py CSV log into the "science" graphs.

This is where the experiment pays off: it visualizes how badly (or not) the
non-realtime USB link jitters, and how well the firmware's on-chip step gen
tracks the commanded position despite that jitter.

Usage:
    # 1) capture a run
    python3 bench_test.py --device /dev/ttyUSB0 --mode sine --axis 0 \
        --amp 10 --freq 0.5 --spu 200 --period 1 --duration 30 --csv run.csv
    # 2) plot it
    python3 plot_bench.py run.csv                 # writes run.png, also shows it
    python3 plot_bench.py run.csv -o out.png --no-show

Produces a 2x2 figure:
    [0,0] commanded vs actual position over time (tracking)
    [0,1] following error (cmd - fb) over time
    [1,0] round-trip latency over time, with the host period marked
    [1,1] latency histogram with mean / p95 / p99 / max markers

and prints a numeric summary (miss rate, latency percentiles, worst error).
"""
import argparse
import csv
import sys

try:
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError:
    sys.stderr.write("matplotlib not installed: pip install matplotlib\n")
    sys.exit(1)


def percentile(sorted_vals, q):
    """Linear-interpolated percentile of an already-sorted list (q in 0..100)."""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = (q / 100.0) * (len(sorted_vals) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = pos - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def load(path):
    rows = {"t": [], "cmd": [], "fb": [], "err": [], "underruns": [],
            "lat": [], "link": []}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows["t"].append(float(r["t_s"]))
            rows["cmd"].append(float(r["cmd_steps"]))
            rows["link"].append(int(r["link"]))
            rows["fb"].append(float(r["fb_steps"]) if r["fb_steps"] != "" else None)
            rows["err"].append(float(r["err_steps"]) if r["err_steps"] != "" else None)
            rows["underruns"].append(
                int(r["underruns"]) if r["underruns"] != "" else None)
            rows["lat"].append(
                float(r["latency_us"]) if r["latency_us"] != "" else None)
    return rows


def main(argv):
    ap = argparse.ArgumentParser(description="plot a bench_test.py CSV log")
    ap.add_argument("csv", help="CSV produced by bench_test.py --csv")
    ap.add_argument("-o", "--out", default=None, help="output PNG (default: <csv>.png)")
    ap.add_argument("--period", type=float, default=1.0,
                    help="host period in ms to mark on the latency plot")
    ap.add_argument("--no-show", action="store_true", help="save only, don't display")
    args = ap.parse_args(argv)

    d = load(args.csv)
    n = len(d["t"])
    if n == 0:
        sys.stderr.write("empty CSV\n")
        return 1

    # --- numeric summary -------------------------------------------------------
    lat = sorted(v for v in d["lat"] if v is not None)
    replies = len(lat)
    miss = n - replies
    errs = [abs(v) for v in d["err"] if v is not None]
    max_underrun = max((u for u in d["underruns"] if u is not None), default=0)

    print("--- bench summary: %s ---" % args.csv)
    print("cycles            : %d" % n)
    print("replies           : %d  (%.2f%% missed)" %
          (replies, 100.0 * miss / n))
    print("max |follow err|  : %d steps" % (max(errs) if errs else 0))
    print("underruns (total) : %d" % max_underrun)
    if lat:
        print("latency us  mean  : %.0f" % (sum(lat) / len(lat)))
        print("latency us  p50   : %.0f" % percentile(lat, 50))
        print("latency us  p95   : %.0f" % percentile(lat, 95))
        print("latency us  p99   : %.0f" % percentile(lat, 99))
        print("latency us  max   : %.0f" % lat[-1])

    # --- figure ----------------------------------------------------------------
    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle("ardustep USB bench: %s" % args.csv, fontsize=13)

    # tracking: cmd vs fb
    t_fb = [t for t, v in zip(d["t"], d["fb"]) if v is not None]
    v_fb = [v for v in d["fb"] if v is not None]
    ax[0, 0].plot(d["t"], d["cmd"], label="commanded", lw=1.2)
    ax[0, 0].plot(t_fb, v_fb, label="actual (fb)", lw=1.0, alpha=0.8)
    ax[0, 0].set_title("position tracking")
    ax[0, 0].set_xlabel("time (s)"); ax[0, 0].set_ylabel("steps")
    ax[0, 0].legend(loc="upper right"); ax[0, 0].grid(True, alpha=0.3)

    # following error
    t_err = [t for t, v in zip(d["t"], d["err"]) if v is not None]
    v_err = [v for v in d["err"] if v is not None]
    ax[0, 1].plot(t_err, v_err, lw=1.0, color="tab:red")
    ax[0, 1].axhline(0, color="k", lw=0.6)
    ax[0, 1].set_title("following error (cmd - fb)")
    ax[0, 1].set_xlabel("time (s)"); ax[0, 1].set_ylabel("steps")
    ax[0, 1].grid(True, alpha=0.3)

    # latency over time, with dropped replies marked at the bottom
    t_lat = [t for t, v in zip(d["t"], d["lat"]) if v is not None]
    v_lat = [v for v in d["lat"] if v is not None]
    ax[1, 0].plot(t_lat, v_lat, lw=0.8, color="tab:green")
    ax[1, 0].axhline(args.period * 1000.0, color="tab:orange", ls="--", lw=1.0,
                     label="host period (%.0f us)" % (args.period * 1000.0))
    t_drop = [t for t, lk in zip(d["t"], d["link"]) if lk == 0]
    if t_drop:
        ax[1, 0].scatter(t_drop, [0] * len(t_drop), s=10, color="red",
                         label="dropped (%d)" % len(t_drop))
    ax[1, 0].set_title("round-trip latency")
    ax[1, 0].set_xlabel("time (s)"); ax[1, 0].set_ylabel("us")
    ax[1, 0].legend(loc="upper right"); ax[1, 0].grid(True, alpha=0.3)

    # latency histogram
    if lat:
        ax[1, 1].hist(lat, bins=60, color="tab:blue", alpha=0.8)
        for q, c in ((50, "k"), (95, "tab:orange"), (99, "tab:red")):
            x = percentile(lat, q)
            ax[1, 1].axvline(x, color=c, ls="--", lw=1.0, label="p%d=%.0f" % (q, x))
        ax[1, 1].axvline(lat[-1], color="purple", ls=":", lw=1.0,
                         label="max=%.0f" % lat[-1])
        ax[1, 1].legend(loc="upper right", fontsize=8)
    ax[1, 1].set_title("latency distribution")
    ax[1, 1].set_xlabel("us"); ax[1, 1].set_ylabel("count")
    ax[1, 1].grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    out = args.out or (args.csv.rsplit(".", 1)[0] + ".png")
    fig.savefig(out, dpi=120)
    print("wrote %s" % out)
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
