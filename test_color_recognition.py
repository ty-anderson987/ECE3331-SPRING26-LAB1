"""
test_color_recognition.py  –  Color Recognition Test
──────────────────────────────────────────────────────
Scoring category: Color Recognition (5%)

Reads ball color (OUT1) and floor color (OUT2) from the TCS3200
via color.py.  Reports raw R/G/B counts, normalized ratios, and
classified color strings.

Usage:
    import test_color_recognition
    test_color_recognition.run()
"""

import color
import time

# ── Config ────────────────────────────────────────────────────────────────────
DURATION_MS  = 1500000   # 0 = run forever
POLL_MS      = 200     # full update interval (matches color.update() gate)
REPORT_EVERY = 5       # print every N polls even if color unchanged

# Target colors expected during the match — tune as needed
EXPECTED_BALL_COLORS  = {"RED", "BLUE", "GREEN"}
EXPECTED_FLOOR_COLORS = {"WHITE", "BLACK", "BLUE", "YELLOW", "PURPLE"}

def _raw(out_pin_fn, s2v, s3v):
    """Pull one raw channel count directly for display."""
    # color._freq is private but accessible for diagnostics
    return color._freq(out_pin_fn, s2v, s3v, ms=20)

def run(duration_ms=DURATION_MS):
    print("=== COLOR RECOGNITION TEST START ===")
    print(f"{'poll':>5}  {'ball_R':>7} {'ball_G':>7} {'ball_B':>7}  "
          f"{'BALL':>8}  {'FLOOR':>8}")
    print("-" * 60)

    poll            = 0
    last_ball       = None
    last_floor      = None
    seen_balls      = set()
    seen_floors     = set()
    deadline        = time.ticks_ms() + duration_ms if duration_ms > 0 else None

    while True:
        if deadline and time.ticks_diff(time.ticks_ms(), deadline) >= 0:
            break

        # read raw ball sensor channels for diagnostics
        r = color._freq(color.OUT1, 0, 0)
        g = color._freq(color.OUT1, 1, 1)
        b = color._freq(color.OUT1, 0, 1)

        ball_str  = color.ball()
        floor_str = color.floor()

        if ball_str  != "NONE":  seen_balls.add(ball_str)
        if floor_str != "UNKNOWN": seen_floors.add(floor_str)

        changed = (ball_str != last_ball) or (floor_str != last_floor)

        if poll % REPORT_EVERY == 0 or changed:
            marker = " <--" if changed else ""
            print(f"{poll:>5}  {r:>7} {g:>7} {b:>7}  "
                  f"{ball_str:>8}  {floor_str:>8}{marker}")

        last_ball  = ball_str
        last_floor = floor_str
        poll      += 1
        time.sleep_ms(POLL_MS)

    # ── Summary ───────────────────────────────────────────────────────────────
    print("-" * 60)
    print("=== COLOR RECOGNITION TEST DONE ===")
    print(f"  Ball colors seen  : {seen_balls  or 'none'}")
    print(f"  Floor colors seen : {seen_floors or 'none'}")

    ball_pass  = bool(seen_balls  & EXPECTED_BALL_COLORS)
    floor_pass = bool(seen_floors & EXPECTED_FLOOR_COLORS)
    print(f"  Ball  PASS        : {'YES' if ball_pass  else 'NO'}")
    print(f"  Floor PASS        : {'YES' if floor_pass else 'NO'}")

run()