#!/usr/bin/env python3
"""
probe_status2.py — Empirically find the frame that turns on SPARK MAX
Status 2 (encoder position) without REV Hardware Client.

Background (CLAUDE.md, 2026-05-29):
  A cold-booted SPARK MAX streams Status 0 (once nudge_sparks sends the
  SLCAN "S8/O" channel-open) but NOT Status 2/3/4/5/6 — position never
  arrives, so position-driven teleop can't start. REV Hardware Client
  enables the rest of the telemetry on connect. Sniffing REV's connect
  (sniff_usb.py -> /tmp/rev_cold_connect.log) showed Status 2-6 all turn
  on together ~20 ms after REV sent ONE new command:

      T0205040247c00ffff      (api 0x010, class 1 / index 0)

  That correlation isn't proof — the payload looks like a value+mask
  register write and REV re-sends it, so it might be DRV polling rather
  than a one-shot "enable telemetry". This script settles it empirically:
  it reproduces the cold state (S0 streaming, S2 silent), replays REV's
  captured candidate frame(s) verbatim to the SAME controller, and reports
  whether Status 2 starts. Replaying REV's own bytes is no riskier than
  REV doing it.

How to use:
  1. Power-cycle the SPARK MAX so it's genuinely cold (S2 off). Do NOT
     open REV and do NOT run nudge_sparks first, or the controller will
     already be enabled and the test is meaningless.
  2. python3 probe_status2.py

  A PASS means: replay this frame from SparkSession open and we have a
  fully REV-free path — fold it into nudge_sparks / the C++ manager.
"""
import sys
import time

from nudge_sparks import SparkSession, list_spark_ports

# Candidate frames to replay, in order. The first is REV's prime suspect;
# the others are the sibling api-0x010 variants REV also sent, in case the
# enable needs a specific value. Each entry is (label, raw SLCAN frame).
CANDIDATES = [
    ("api0x010 7c00ffff", "T0205040247c00ffff\r"),
    ("api0x010 0400ffff", "T0205040240400ffff\r"),
    ("api0x010 ffff0000", "T020504024ffff0000\r"),
]

SETTLE_S = 2.5   # time to let Status 0 establish / each candidate to act


def counts(s):
    return s.status_0_count, s.status_2_count


def main() -> int:
    ports = list_spark_ports()
    if not ports:
        print("No SPARK MAX USB devices found.")
        return 1
    port = ports[0]
    print(f"Probing {port}  (open {len(ports)} found, using first)")

    sess = SparkSession(port)
    try:
        # Baseline: channel open + heartbeat only. A cold controller should
        # show S0 climbing and S2 stuck at 0.
        time.sleep(SETTLE_S)
        s0, s2 = counts(sess)
        print(f"\nbaseline after {SETTLE_S:.1f}s heartbeat: "
              f"S0={s0}  S2={s2}  dev={sess.device_id}")
        if s0 == 0:
            print("  !! S0=0 — channel not open or controller dead/cold; "
                  "fix that before probing Status 2.")
            return 2
        if s2 > 0:
            print("  !! S2 already streaming — controller is NOT cold "
                  "(REV or a prior nudge enabled it). Power-cycle and retry.")
            return 2

        # Replay each candidate, watching for S2 to come alive.
        for label, frame in CANDIDATES:
            before0, before2 = counts(sess)
            print(f"\n--> replaying {label}: {frame!r}")
            sess.inject(frame)
            time.sleep(SETTLE_S)
            after0, after2 = counts(sess)
            d2 = after2 - before2
            print(f"    S0 {before0}->{after0} (+{after0-before0})   "
                  f"S2 {before2}->{after2} (+{d2})")
            if d2 > 0:
                print(f"\nPASS — {label} turned Status 2 ON "
                      f"(+{d2} frames). This is the REV-free enable frame.")
                return 0

        print("\nFAIL — none of the candidates started Status 2. "
              "The enable is not a single api-0x010 frame; fall back to "
              "Burn Flash, or widen the candidate set from the log.")
        return 3
    finally:
        sess.close()


if __name__ == "__main__":
    raise SystemExit(main())
