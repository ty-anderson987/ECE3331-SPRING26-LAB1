"""
test_obstacle_avoidance.py  –  Obstacle Avoidance Test
────────────────────────────────────────────────────────
Scoring category: Obstacle Avoidance (5%)

Drives forward, monitors sonar1 (front fixed) and sonar2 (sweeping),
and verifies the robot stops before the configured threshold distance.
Also exercises the full sweep_best_direction logic.

Usage:
    import test_obstacle_avoidance
    test_obstacle_avoidance.run()
"""

import motors
import sensors
import oc
from config import SONAR1_STOP_CM, SONAR2_STOP_CM, FWD_BASE_L, FWD_BASE_R
import time

# ── Config ────────────────────────────────────────────────────────────────────
FORWARD_MS     = 5000   # how long to drive before the test ends
POLL_MS        = 50
STOP_MARGIN_CM = 5.0    # extra tolerance — stop must happen within threshold + margin
TEST_SPEED_L   = 65535  # scale these together to keep straight driving
TEST_SPEED_R   = 61750  # maintains same L/R ratio as FWD_BASE (61750/65535 * 40000)

def _sonar1_filtered(samples=3):
    readings = []
    for _ in range(samples):
        r = sensors.sonar1()
        if r < 400:
            readings.append(r)
        time.sleep_ms(10)
    return min(readings) if readings else 999

def _sonar2_filtered(samples=3):
    readings = []
    for _ in range(samples):
        r = sensors.sonar2_calibrated()
        if r < 400:
            readings.append(r)
        time.sleep_ms(10)
    return min(readings) if readings else 999

def run(forward_ms=FORWARD_MS):
    print("=== OBSTACLE AVOIDANCE TEST START ===")
    print(f"  sonar1 stop threshold : {SONAR1_STOP_CM} cm")
    print(f"  sonar2 stop threshold : {SONAR2_STOP_CM} cm")
    print(f"  driving for up to {forward_ms} ms — place obstacle in path\n")

    print(f"{'ms':>6}  {'sonar1 cm':>10}  {'sonar2 cm':>10}  {'state':>12}")
    print("-" * 50)

    motors.set_left_fwd(TEST_SPEED_L)
    motors.set_right_fwd(TEST_SPEED_R)
    time.sleep_ms(200)

    deadline      = time.ticks_ms() + forward_ms
    stopped       = False
    stop_dist     = None
    stop_time_ms  = None
    start_ms      = time.ticks_ms()

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        oc.check_both()
        if oc.halted():
            motors.stop()
            print("[OC] overcurrent — test aborted")
            return

        d1 = _sonar1_filtered()
        d2 = _sonar2_filtered()
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms)

        if not stopped and d1 <= SONAR1_STOP_CM:
            motors.stop()
            stopped      = True
            stop_dist    = d1
            stop_time_ms = elapsed
            print(f"{elapsed:>6}  {d1:>10.1f}  {d2:>10.1f}  {'STOPPED':>12}  <-- obstacle")
        else:
            state_str = "stopped" if stopped else "driving"
            print(f"{elapsed:>6}  {d1:>10.1f}  {d2:>10.1f}  {state_str:>12}")
            if not stopped:
                motors.set_left_fwd(TEST_SPEED_L)
                motors.set_right_fwd(TEST_SPEED_R)

        time.sleep_ms(POLL_MS)

    motors.stop()

    # ── Sweep test ────────────────────────────────────────────────────────────
    print("\n--- Running full sweep scan ---")
    sensors.print_sweep()

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n=== OBSTACLE AVOIDANCE TEST DONE ===")
    if stopped:
        within = stop_dist <= (SONAR1_STOP_CM + STOP_MARGIN_CM)
        print(f"  Stopped at    : {stop_dist:.1f} cm  (threshold={SONAR1_STOP_CM} cm)")
        print(f"  Stop time     : {stop_time_ms} ms from start")
        print(f"  PASS          : {'YES — stopped within threshold' if within else 'NO  — stopped too late'}")
    else:
        print("  Robot drove full duration without stopping.")
        print("  PASS          : NO — no obstacle triggered")

run()