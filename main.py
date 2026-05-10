"""
main.py  –  Red Raider Soccer — Final System Integration
─────────────────────────────────────────────────────────
SCORING (per rubric):
  RED   = -2   GREEN = +2   BLUE  = +1

CONTROLS (IR remote):
  0          = select ATTACKER mode (1 beep)
  1          = select DEFENDER mode (2 beeps)
  PLAY/PAUSE = start / stop (long beep on start, 3 beeps on stop)

ATTACKER (kicker — full field):
  - Sweeps left/right looking for balls (IR sensors)
  - On ball intake (color sensor):
      GREEN/BLUE → keep, drive forward toward opponent goal
      RED        → eject immediately
  - Obstacle/border/OC interrupts ANY action (highest priority)

DEFENDER (goalie — own half only):
  - Same sweep + ball detection
  - Treats BLUE floor (center line) as a wall — never crosses it
  - On ball intake:
      GREEN/BLUE → eject (don't let opponent score in our goal)
      RED        → eject (we want to keep our goal clean of -2 balls)

Priority order during EVERY movement:
  1. IR remote stop
  2. OC trip → smart reverse + 180 (uses side IR for direction)
  3. sonar1 obstacle → sweep + 90° turn (avoid hitting other robots!)
  4. floor BLACK/BLUE → reverse + 180 (border / center line)
  5. ir_ball detected → chase
"""

from machine import PWM, Pin
import time
import motors
import sensors
import color
import oc
import buzzer
import ir_remote
from config import (
    SONAR1_STOP_CM, SWEEP_CENTER,
    PIN_SERVO1, SERVO1_SPIN, SERVO1_STOP,
    IR_0, IR_1, IR_PLAY_PAUSE,
    FWD_BASE_L, FWD_BASE_R
)

# ── Config ────────────────────────────────────────────────────
DRIVE_SPEED_L = FWD_BASE_L
DRIVE_SPEED_R = FWD_BASE_R
SWEEP_SPEED   = 35000

# Priority thresholds
OBSTACLE_CM   = SONAR1_STOP_CM   # sonar1 stop distance for other robots
BORDER_COLORS = {"BLACK", "BLUE"}   # both modes: black wall, attacker-OK on blue
GOALIE_BORDER = {"BLACK", "BLUE"}   # defender: blue is hard limit

# Timings
REVERSE_MS    = 600
LONG_REV_MS   = 1000
PAUSE_MS      = 250
TURN_REST_MS  = 2000   # longer rest after any turn — lets motors cool, prevents OC
SWEEP_MS      = 1000
FWD_MS        = 500
CHASE_MS      = 800
EJECT_MS      = 700
COOLDOWN_MS   = 800
LOOP_MS       = 20

# ── Servo1 (intake) setup ────────────────────────────────────
_servo1 = PWM(Pin(PIN_SERVO1))
_servo1.freq(50)
sensors.servo2_hold_center()

SERVO1_REVERSE = SERVO1_STOP + (SERVO1_STOP - SERVO1_SPIN)

def _s1(us):
    _servo1.duty_u16(int(us / 20000 * 65535))
    sensors.servo2_hold_center()

def s1_intake(): _s1(SERVO1_SPIN)
def s1_eject():  _s1(SERVO1_REVERSE)
def s1_stop():   _s1(SERVO1_STOP)

# ── Fast ball color classifier ────────────────────────────────
def _fast_ball():
    r = color._freq(color.OUT1, 0, 0, ms=5)
    g = color._freq(color.OUT1, 1, 1, ms=5)
    b = color._freq(color.OUT1, 0, 1, ms=5)
    total = r + g + b
    if total < 30:
        return "NONE"
    rn, gn, bn = r/total, g/total, b/total
    if gn > 0.42:                         return "GREEN"
    if bn > 0.43 and rn < 0.27:           return "BLUE"
    if total > 35 and rn > 0.50 and gn < 0.23: return "RED"
    return "NONE"

# ── State + IR control ────────────────────────────────────────
_mode    = None
_running = False

def _ir_handler(code):
    global _mode, _running
    if code == IR_0 and not _running:
        _mode = 'attacker'
        print("MODE: ATTACKER (1 beep)")
        buzzer.beep(n=1, ms=80)
    elif code == IR_1 and not _running:
        _mode = 'defender'
        print("MODE: DEFENDER (2 beeps)")
        buzzer.beep(n=2, ms=80, gap=80)
    elif code == IR_PLAY_PAUSE:
        if _mode is None:
            print("select mode first: 0=attacker 1=defender")
            return
        _running = not _running
        if _running:
            print(f"START [{_mode.upper()}]")
            buzzer.beep(n=1, ms=200)
        else:
            motors.stop()
            s1_stop()
            print("STOP")
            buzzer.beep(n=3, ms=60, gap=60)

ir_remote.set_callback(_ir_handler)

# ── Motion primitives ─────────────────────────────────────────
def _drive_fwd():
    oc.start_grace()
    motors.set_left_fwd(DRIVE_SPEED_L)
    motors.set_right_fwd(DRIVE_SPEED_R)

def _drive_rev():
    oc.start_grace()
    motors.set_left_rev(DRIVE_SPEED_L)
    motors.set_right_rev(DRIVE_SPEED_R)

def _turn_left_inplace():
    oc.start_grace()
    motors.set_left_rev(SWEEP_SPEED)
    motors.set_right_fwd(SWEEP_SPEED)

def _turn_right_inplace():
    oc.start_grace()
    motors.set_left_fwd(SWEEP_SPEED)
    motors.set_right_rev(SWEEP_SPEED)

def _reverse_for(ms=REVERSE_MS):
    _drive_rev()
    time.sleep_ms(ms)
    motors.stop()

def _rotate_180(direction='right'):
    actual1 = motors.turn_degrees(90, direction, timeout_ms=5000) or 90.0
    time.sleep_ms(PAUSE_MS)
    target2 = max(70.0, 90.0 - (actual1 - 90.0))
    motors.turn_degrees(target2, direction, timeout_ms=5000)
    time.sleep_ms(TURN_REST_MS)   # rest motors after full 180

# ── Recovery handlers ─────────────────────────────────────────
def _oc_recover():
    print("[OC RECOVER] reverse + 180")
    motors.stop()
    s1_stop()
    oc.reset_state()
    time.sleep_ms(200)
    lb = sensors.ir_left_val()  == 0
    rb = sensors.ir_right_val() == 0
    if lb and rb:
        _reverse_for(LONG_REV_MS); time.sleep_ms(PAUSE_MS); _rotate_180('right')
    elif rb:
        _reverse_for();             time.sleep_ms(PAUSE_MS); _rotate_180('left')
    elif lb:
        _reverse_for();             time.sleep_ms(PAUSE_MS); _rotate_180('right')
    else:
        _reverse_for();             time.sleep_ms(PAUSE_MS); _rotate_180('right')
    s1_intake()

def _obstacle_recover():
    print("[OBSTACLE] sweep + turn")
    motors.stop()
    time.sleep_ms(PAUSE_MS)
    scan, _ = sensors.sweep_scan(step=15)
    if not scan:
        _rotate_180('right')
        return
    best_angle = max(scan, key=lambda a: scan[a])
    direction = 'left' if best_angle < SWEEP_CENTER - 5 else 'right'
    motors.turn_degrees(90, direction, timeout_ms=5000)
    time.sleep_ms(TURN_REST_MS)   # rest after 90 turn

def _border_recover():
    fc = color.floor()
    print(f"[BORDER] {fc} reverse + 180")
    motors.stop()
    time.sleep_ms(PAUSE_MS)
    _reverse_for()
    time.sleep_ms(PAUSE_MS)
    _rotate_180('right')
    # _rotate_180 already includes TURN_REST_MS at the end

# ── Priority check (always-running watchdog) ─────────────────
# State: 'searching' = normal obstacle avoidance via sonar1
#        'chasing'   = sonar1 close + IR ball = chase, sonar2 high = real obstacle
_state = 'searching'
_obstacle_strikes = 0   # need 2 consecutive close readings to trigger

def _check_priorities(border_set):
    """
    Returns one of: None, 'stop', 'oc', 'obstacle', 'border'.
    Uses raw sonar1() with a 2-strike debounce — single noise spikes
    don't trigger, but two consecutive close readings do.
    """
    global _obstacle_strikes

    # 1. user stop
    ir_remote.check()
    if not _running:
        motors.stop()
        return 'stop'

    # 2. OC
    oc.check_both()
    if oc.halted():
        return 'oc'

    # 3. obstacle — depends on state
    if _state == 'chasing':
        if sensors.sonar2_calibrated() < OBSTACLE_CM:
            return 'obstacle'
    else:
        if sensors.sonar1() < OBSTACLE_CM and not sensors.ir_ball():
            _obstacle_strikes += 1
            if _obstacle_strikes >= 2:
                _obstacle_strikes = 0
                return 'obstacle'
        else:
            _obstacle_strikes = 0

    # 4. border / center line
    if color.floor() in border_set:
        return 'border'

    return None

def _handle_priority(p):
    if p == 'oc':
        _oc_recover()
    elif p == 'obstacle':
        _obstacle_recover()
    elif p == 'border':
        _border_recover()

# ── Movement runner with priority + ball detection ──────────
def _run_segment(start_motion_fn, ms, border_set):
    """
    Run start_motion_fn() for ms while polling priorities + IR ball.
    Returns: 'stop', 'priority', 'ball', or 'done'.
    """
    start_motion_fn()
    deadline = time.ticks_ms() + ms
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        p = _check_priorities(border_set)
        if p == 'stop':
            return 'stop'
        if p:
            _handle_priority(p)
            return 'priority'
        if sensors.ir_ball():
            motors.stop()
            return 'ball'
        time.sleep_ms(LOOP_MS)
    motors.stop()
    return 'done'

# ── Sweep pattern ─────────────────────────────────────────────
SWEEP_DEG       = 30      # degrees per sweep segment
SWEEP_PWM       = 35000   # speed for sweep turns
SLOW_CHASE_PWM  = 38000   # forward speed once ball is centered (slow + controlled)

def _both_irs_see_ball():
    """Both front IRs firing simultaneously = ball directly ahead."""
    return sensors.ir1_val() == 0 and sensors.ir2_val() == 0

def _sweep_turn_sampling(direction, degrees, border_set):
    """
    Custom gyro-based turn that samples both IRs continuously.
    Ball is considered FOUND only when BOTH IRs fire at the same time —
    eliminates false positives from a single noisy sensor.

    Returns:
      'ball'     - both IRs fired simultaneously, robot stopped facing ball
      'priority' - obstacle/border/OC handled
      'stop'     - user pressed stop
      'done'     - turn completed without finding ball
    """
    from time import ticks_ms, ticks_diff

    oc.reset_state()
    oc.set_mode('turn')
    oc.start_grace()

    if direction == 'right':
        motors.set_left_fwd(SWEEP_PWM)
        motors.set_right_rev(SWEEP_PWM)
    else:
        motors.set_left_rev(SWEEP_PWM)
        motors.set_right_fwd(SWEEP_PWM)

    accumulated = 0.0
    last_t      = ticks_ms()
    timeout     = ticks_ms() + 4000

    try:
        while ticks_diff(timeout, ticks_ms()) > 0:
            # 1. ball check — both IRs simultaneously = centered
            if _both_irs_see_ball():
                motors.stop()
                return 'ball'

            # 2. priority check (sonar obstacle / border / OC / stop)
            p = _check_priorities(border_set)
            if p == 'stop':
                motors.stop()
                return 'stop'
            if p:
                motors.stop()
                _handle_priority(p)
                return 'priority'

            # 3. gyro angle accumulation
            now = ticks_ms()
            dt  = ticks_diff(now, last_t) / 1000.0
            last_t = now
            try:
                gz = sensors.read_gyro_z()
            except OSError:
                gz = 0.0
            accumulated += abs(gz) * dt

            if accumulated >= degrees:
                break

            time.sleep_ms(LOOP_MS)
    finally:
        oc.set_mode('straight')

    motors.stop()
    return 'done'

def _sweep_for_ball(border_set):
    """
    Sweep left → right (back) → right → left (back) at SWEEP_DEG per segment.
    Each segment samples both IRs continuously — stops the moment both IRs
    fire together (ball centered, no separate centering step needed).
    Sonar obstacle takes priority at every check.
    """
    pattern = [
        ('left',  SWEEP_DEG),
        ('right', SWEEP_DEG),
        ('right', SWEEP_DEG),
        ('left',  SWEEP_DEG),
    ]
    for direction, deg in pattern:
        result = _sweep_turn_sampling(direction, deg, border_set)
        if result != 'done':
            return result
    return 'done'

# ── Ball chase ────────────────────────────────────────────────
def _chase_ball(border_set):
    """
    CHASING state. By the time we get here, _sweep_for_ball already centered
    the robot on the ball (both IRs fired simultaneously). Just drive forward
    slowly until intake or ball lost.
    """
    global _state
    _state = 'chasing'
    try:
        oc.start_grace()
        motors.set_left_fwd(SLOW_CHASE_PWM)
        motors.set_right_fwd(SLOW_CHASE_PWM)
        deadline      = time.ticks_ms() + CHASE_MS
        ir_lost_since = 0

        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            p = _check_priorities(border_set)
            if p == 'stop':
                motors.stop()
                return 'stop'
            if p:
                _handle_priority(p)
                return 'priority'

            ir_seeing   = sensors.ir_ball()
            sonar_close = sensors.sonar1() < OBSTACLE_CM

            if ir_seeing:
                ir_lost_since = 0
            else:
                if ir_lost_since == 0:
                    ir_lost_since = time.ticks_ms()
                lost_ms = time.ticks_diff(time.ticks_ms(), ir_lost_since)
                # both IR lost and sonar clear = ball gone
                if not sonar_close and lost_ms > 200:
                    break
                # IR lost but sonar close = under intake, keep going
                if lost_ms > 600:
                    break

            time.sleep_ms(LOOP_MS)

        motors.stop()
        return 'done'
    finally:
        _state = 'searching'


# ── Eject + drain to NONE ────────────────────────────────────
def _eject_until_clear():
    s1_eject()
    deadline = time.ticks_ms() + EJECT_MS
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if _fast_ball() == "NONE":
            break
        time.sleep_ms(20)
    s1_intake()

# ── Mode runners ──────────────────────────────────────────────
def _run_attacker():
    """
    Full field. Sweep → find ball → chase. On intake:
      RED        → eject (avoid -2 in our goal)
      GREEN/BLUE → keep driving forward (toward opponent goal)
    Border = BLACK only (blue center line is OK to cross).
    """
    print("ATTACKER running")
    s1_intake()
    border_set = {"BLACK"}   # blue line OK for kicker
    last_ball_t = 0

    while _running:
        # check what's currently in the intake chamber
        now = time.ticks_ms()
        if time.ticks_diff(now, last_ball_t) > COOLDOWN_MS:
            ball = _fast_ball()
            if ball == "RED":
                buzzer.beep(n=2, ms=80, gap=80)
                _eject_until_clear()
                last_ball_t = time.ticks_ms()
                continue
            elif ball in ("GREEN", "BLUE"):
                buzzer.beep(n=1, ms=200)
                # got a scoring ball — drive forward to push it into goal
                _run_segment(_drive_fwd, 1500, border_set)
                last_ball_t = time.ticks_ms()
                continue

        # no ball in chamber — sweep then advance
        result = _sweep_for_ball(border_set)
        if result == 'stop':
            break
        if result == 'ball':
            _chase_ball(border_set)
        elif result == 'done':
            _run_segment(_drive_fwd, FWD_MS, border_set)
        # 'priority' falls through and re-loops

    motors.stop()
    s1_stop()

def _run_defender():
    """
    Stay on own half. Blue center line is a HARD LIMIT.
    On intake: any ball → eject (keep our goal clean).
    Border = BLACK and BLUE both treated as walls.
    """
    print("DEFENDER running")
    s1_intake()
    border_set = GOALIE_BORDER   # blue + black both stop us
    last_ball_t = 0

    while _running:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_ball_t) > COOLDOWN_MS:
            ball = _fast_ball()
            if ball in ("GREEN", "BLUE", "RED"):
                buzzer.beep(n=2, ms=80, gap=80)
                _eject_until_clear()
                last_ball_t = time.ticks_ms()
                continue

        result = _sweep_for_ball(border_set)
        if result == 'stop':
            break
        if result == 'ball':
            _chase_ball(border_set)
        elif result == 'done':
            # short forward patrol then back
            _run_segment(_drive_fwd, 300, border_set)

    motors.stop()
    s1_stop()

# ── Main entry ────────────────────────────────────────────────
def run():
    print("─" * 40)
    print("  RED RAIDER SOCCER")
    print("    0 = ATTACKER (1 beep)")
    print("    1 = DEFENDER (2 beeps)")
    print("    PLAY/PAUSE  = start / stop")
    print("─" * 40)

    while True:
        ir_remote.check()
        if _running:
            if _mode == 'attacker':
                _run_attacker()
            elif _mode == 'defender':
                _run_defender()
        time.sleep_ms(LOOP_MS)

run()