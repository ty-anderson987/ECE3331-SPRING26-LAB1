"""
test_field_navigation.py  –  Border Patrol + Obstacle Avoidance
────────────────────────────────────────────────────────────────
Drive loop:
  1. Drive forward
  2. sonar1 < OBSTACLE_CM  → obstacle response (sweep + 90° turn)
  3. color.floor() BLACK/BLUE → border response (reverse + 180°)
  4. OC trip → smart reverse + turn based on side IR sensors

OC recovery direction:
  - right IR triggered → obstacle on right → turn left
  - left  IR triggered → obstacle on left  → turn right
  - both triggered     → reverse extra, then 180°
  - neither            → 180° default
"""

import motors
import sensors
import color
import oc
import time
from config import SONAR1_STOP_CM, SWEEP_CENTER

# ── Config ────────────────────────────────────────────────────────────────────
DRIVE_SPEED_L = 65535
DRIVE_SPEED_R = 65535
BORDER_COLORS = {"BLACK", "BLUE"}
OBSTACLE_CM   = SONAR1_STOP_CM
REVERSE_MS    = 600
LONG_REV_MS   = 1000
PAUSE_MS      = 300

# ── Rotate 180 with overshoot compensation ────────────────────────────────────
def _rotate_180(direction='right'):
    actual1 = motors.turn_degrees(90, direction, timeout_ms=5000) or 90.0
    time.sleep_ms(PAUSE_MS)
    overshoot = actual1 - 90.0
    target2   = max(70.0, 90.0 - overshoot)
    motors.turn_degrees(target2, direction, timeout_ms=5000)
    time.sleep_ms(PAUSE_MS)

# ── Reverse helper ────────────────────────────────────────────────────────────
def _reverse(ms=REVERSE_MS):
    motors.set_left_rev(DRIVE_SPEED_L)
    motors.set_right_rev(DRIVE_SPEED_R)
    time.sleep_ms(ms)
    motors.stop()

# ── OC recovery ───────────────────────────────────────────────────────────────
def _oc_recover():
    motors.stop()
    oc.reset_state()
    time.sleep_ms(200)

    left_blocked  = sensors.ir_left_val()  == 0
    right_blocked = sensors.ir_right_val() == 0

    if left_blocked and right_blocked:
        print("[OC] both blocked")
        _reverse(LONG_REV_MS)
        time.sleep_ms(PAUSE_MS)
        _rotate_180('right')
    elif right_blocked:
        print("[OC] right blocked - turn left")
        _reverse()
        time.sleep_ms(PAUSE_MS)
        _rotate_180('left')
    elif left_blocked:
        print("[OC] left blocked - turn right")
        _reverse()
        time.sleep_ms(PAUSE_MS)
        _rotate_180('right')
    else:
        print("[OC] no IR - 180")
        _reverse()
        time.sleep_ms(PAUSE_MS)
        _rotate_180('right')

# ── Obstacle sweep + turn ─────────────────────────────────────────────────────
def _obstacle_response():
    motors.stop()
    time.sleep_ms(PAUSE_MS)
    scan, _ = sensors.sweep_scan(step=15)
    if not scan:
        _rotate_180('right')
        return

    best_angle = max(scan, key=lambda a: scan[a])
    if best_angle < SWEEP_CENTER - 5:
        direction = 'left'
    elif best_angle > SWEEP_CENTER + 5:
        direction = 'right'
    else:
        direction = 'right'

    print(f"[OBS] {best_angle}deg turn {direction}")
    motors.turn_degrees(90, direction, timeout_ms=5000)
    time.sleep_ms(PAUSE_MS)

# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    print("BORDER PATROL START")
    cycle = 0

    try:
        while True:
            cycle += 1
            print(f"cycle {cycle}")

            motors.set_left_fwd(DRIVE_SPEED_L)
            motors.set_right_fwd(DRIVE_SPEED_R)

            while True:
                oc.check_both()
                if oc.halted():
                    _oc_recover()
                    break

                dist = sensors.sonar1()
                if dist < OBSTACLE_CM:
                    print(f"[OBS] {dist:.0f}cm")
                    _obstacle_response()
                    break

                fc = color.floor()
                if fc in BORDER_COLORS:
                    motors.stop()
                    print(f"[BORDER] {fc}")
                    time.sleep_ms(PAUSE_MS)
                    _reverse()
                    time.sleep_ms(PAUSE_MS)
                    _rotate_180('right')
                    break

                time.sleep_ms(20)

    except KeyboardInterrupt:
        motors.stop()
        print("stopped")

run()