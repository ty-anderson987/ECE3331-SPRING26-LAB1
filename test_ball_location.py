"""
test_ball_location.py  –  Ball Location with State Machine
────────────────────────────────────────────────────────────
SEARCHING state (default):
  - Sweeps left/center/right/center looking for IR ball detection
  - sonar1 < OBSTACLE_CM = obstacle (other robots, walls) → avoid
  - sonar1 trigger ignored if IR sees a ball at same time

CHASING state (when IR fires):
  - Drives toward ball
  - sonar1 ignored (expects to see the ball)
  - sonar2 (higher mounted) used for real obstacles
  - If IR loses ball but sonar1 still close → keep going (ball under intake)
  - If both lost for 600ms → exit chase

Priority order in BOTH states:
  1. OC trip → smart reverse + 180
  2. obstacle → sweep + 90 turn
  3. border (BLACK/BLUE floor) → reverse + 180
"""

import motors
import sensors
import color
import oc
import time
from config import SONAR1_STOP_CM, SWEEP_CENTER

# ── Config ────────────────────────────────────────────────────
DRIVE_SPEED_L = 65535
DRIVE_SPEED_R = 65535
BORDER_COLORS = {"BLACK", "BLUE"}
OBSTACLE_CM   = SONAR1_STOP_CM
REVERSE_MS    = 600
LONG_REV_MS   = 1000
PAUSE_MS      = 300
SWEEP_SPEED   = 35000
SWEEP_MS      = 1000
FWD_MS        = 500
CHASE_MS      = 800
LOOP_MS       = 20

# ── State ─────────────────────────────────────────────────────
_state = 'searching'   # or 'chasing'

# ── Helpers ───────────────────────────────────────────────────
def _rotate_180(direction='right'):
    actual1 = motors.turn_degrees(90, direction, timeout_ms=5000) or 90.0
    time.sleep_ms(PAUSE_MS)
    target2 = max(70.0, 90.0 - (actual1 - 90.0))
    motors.turn_degrees(target2, direction, timeout_ms=5000)
    time.sleep_ms(PAUSE_MS)

def _reverse(ms=REVERSE_MS):
    oc.start_grace()
    motors.set_left_rev(DRIVE_SPEED_L)
    motors.set_right_rev(DRIVE_SPEED_R)
    time.sleep_ms(ms)
    motors.stop()

def _oc_recover():
    print("[OC RECOVER]")
    motors.stop()
    oc.reset_state()
    time.sleep_ms(200)
    lb = sensors.ir_left_val()  == 0
    rb = sensors.ir_right_val() == 0
    if lb and rb:
        _reverse(LONG_REV_MS); time.sleep_ms(PAUSE_MS); _rotate_180('right')
    elif rb:
        _reverse();             time.sleep_ms(PAUSE_MS); _rotate_180('left')
    elif lb:
        _reverse();             time.sleep_ms(PAUSE_MS); _rotate_180('right')
    else:
        _reverse();             time.sleep_ms(PAUSE_MS); _rotate_180('right')

def _obstacle_response():
    print("[OBSTACLE]")
    motors.stop()
    time.sleep_ms(PAUSE_MS)
    scan, _ = sensors.sweep_scan(step=15)
    if not scan:
        _rotate_180('right')
        return
    best_angle = max(scan, key=lambda a: scan[a])
    direction = 'left' if best_angle < SWEEP_CENTER - 5 else 'right'
    motors.turn_degrees(90, direction, timeout_ms=5000)
    time.sleep_ms(PAUSE_MS)

def _border_response():
    fc = color.floor()
    print(f"[BORDER] {fc}")
    motors.stop()
    time.sleep_ms(PAUSE_MS)
    _reverse()
    time.sleep_ms(PAUSE_MS)
    _rotate_180('right')

def _priorities():
    """State-aware priority check. Returns True if handled."""
    oc.check_both()
    if oc.halted():
        _oc_recover()
        return True

    # obstacle check depends on state
    if _state == 'chasing':
        # use sonar2 (higher) — won't see ball on floor
        if sensors.sonar2_calibrated() < OBSTACLE_CM:
            _obstacle_response()
            return True
        # ignore sonar1 — that's the ball
    else:
        # SEARCHING — use sonar1 unless IR sees ball
        if sensors.sonar1() < OBSTACLE_CM and not sensors.ir_ball():
            _obstacle_response()
            return True

    if color.floor() in BORDER_COLORS:
        _border_response()
        return True
    return False

# ── Movement ──────────────────────────────────────────────────
def _move(set_fn, ms):
    """Run movement for ms with priority + IR ball checks."""
    set_fn()
    deadline = time.ticks_ms() + ms
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if _priorities():
            return 'priority'
        if sensors.ir_ball():
            motors.stop()
            return 'ball'
        time.sleep_ms(LOOP_MS)
    motors.stop()
    return 'done'

def _turn_left():
    oc.start_grace()
    motors.set_left_rev(SWEEP_SPEED)
    motors.set_right_fwd(SWEEP_SPEED)

def _turn_right():
    oc.start_grace()
    motors.set_left_fwd(SWEEP_SPEED)
    motors.set_right_rev(SWEEP_SPEED)

def _drive_fwd():
    oc.start_grace()
    motors.set_left_fwd(DRIVE_SPEED_L)
    motors.set_right_fwd(DRIVE_SPEED_R)

# ── Ball chase (CHASING state) ───────────────────────────────
def _chase():
    """
    Switches to CHASING state. Faces the ball, drives forward.
    sonar1 ignored, sonar2 used for real obstacles.
    Keeps going if IR lost but sonar1 still sees something close.
    """
    global _state
    _state = 'chasing'
    try:
        ir1 = sensors.ir1_val()
        ir2 = sensors.ir2_val()
        if ir1 == 0 and ir2 != 0:
            motors.turn_degrees(20, 'left',  timeout_ms=2000)
        elif ir2 == 0 and ir1 != 0:
            motors.turn_degrees(20, 'right', timeout_ms=2000)
        time.sleep_ms(100)

        oc.start_grace()
        motors.set_left_fwd(DRIVE_SPEED_L)
        motors.set_right_fwd(DRIVE_SPEED_R)
        deadline      = time.ticks_ms() + CHASE_MS
        ir_lost_since = 0   # 0 = IR currently sees ball

        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if _priorities():
                return

            ir_seeing   = sensors.ir_ball()
            sonar_close = sensors.sonar1() < OBSTACLE_CM

            if ir_seeing:
                ir_lost_since = 0
            else:
                if ir_lost_since == 0:
                    ir_lost_since = time.ticks_ms()
                lost_ms = time.ticks_diff(time.ticks_ms(), ir_lost_since)

                # both lost = ball gone
                if not sonar_close and lost_ms > 200:
                    break
                # IR lost but sonar still close = under intake, keep going
                if lost_ms > 600:
                    break

            time.sleep_ms(LOOP_MS)

        motors.stop()
    finally:
        _state = 'searching'

# ── Main ──────────────────────────────────────────────────────
def run():
    print("BALL LOCATION TEST (state machine)")
    try:
        while True:
            # full sweep: left → right (center) → right → left (center)
            for fn, ms in [
                (_turn_left,  SWEEP_MS),
                (_turn_right, SWEEP_MS),
                (_turn_right, SWEEP_MS),
                (_turn_left,  SWEEP_MS),
            ]:
                result = _move(fn, ms)
                if result == 'ball':
                    _chase()
                    break
                if result == 'priority':
                    break
            else:
                # full sweep finished without finding — drive forward
                result = _move(_drive_fwd, FWD_MS)
                if result == 'ball':
                    _chase()

    except KeyboardInterrupt:
        motors.stop()
        print("stopped")

run()