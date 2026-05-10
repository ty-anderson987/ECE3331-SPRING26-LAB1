"""
test_ball_maneuverability.py  –  Ball Maneuverability Test
───────────────────────────────────────────────────────────
Scoring category: Ball Maneuverability (10%)

Main loop (repeats until ball found or stop condition):
  1. CHECK obstacle + border — highest priority, stop immediately if triggered
  2. CREEP forward CREEP_MS
  3. STOP + cooldown
  4. SWEEP: LEFT → CENTER → RIGHT → CENTER checking IR at each stop
  5. If IR triggers → go to carry mode
  6. Otherwise repeat from 1

Carry mode:
  - Drive forward steering by IR (ir1=left, ir2=right)
  - Obstacle and border checks every loop tick — immediate stop

Usage:
    import test_ball_maneuverability
    test_ball_maneuverability.run()
"""

import motors
import sensors
import oc
import encoders
import color
from machine import PWM, Pin
from config import (SONAR1_STOP_CM, PIN_SERVO1, SERVO1_SPIN, SERVO1_STOP)
import time

# ── Config ────────────────────────────────────────────────────────────────────
CREEP_SPEED_L     = 65535   # forward creep speed
CREEP_SPEED_R     = 65535
CREEP_MS          = 400     # how long to creep forward each cycle

CARRY_SPEED_L     = 65535
CARRY_SPEED_R     = 65535
NUDGE_REDUCE      = 8000    # slow one side to steer

TURN_SPEED        = 65535   # scan turn speed
TURN_MS           = 500     # duration of each scan turn
TURN_COOL_MS      = 300     # motors-off cooldown between moves

SCAN_TIMEOUT_MS   = 800000  # total scan timeout (~13 min)
CARRY_TIMEOUT_MS  = 10000   # max carry time

OBSTACLE_CM       = SONAR1_STOP_CM
BORDER_COLOR      = "BLACK"

# ── Servo1 ────────────────────────────────────────────────────────────────────
_servo1 = PWM(Pin(PIN_SERVO1))
_servo1.freq(50)

def _s1_spin():
    _servo1.duty_u16(int(SERVO1_SPIN / 20000 * 65535))

def _s1_stop():
    _servo1.duty_u16(int(SERVO1_STOP / 20000 * 65535))

# ── Stop condition checks — HIGHEST PRIORITY ─────────────────────────────────
def _obstacle():
    """Returns (blocked, dist). Always check this first."""
    dist = sensors.sonar1()
    return dist <= OBSTACLE_CM, dist

def _border():
    return color.floor() == BORDER_COLOR

def _oc():
    oc.check_both()
    return oc.halted()

def _stop_all():
    motors.stop()
    _s1_stop()

def _priority_stop():
    """
    Check all high-priority stops.
    Returns reason string if triggered, None if clear.
    """
    if _oc():
        _stop_all()
        return 'overcurrent'
    blocked, dist = _obstacle()
    if blocked:
        _stop_all()
        print(f"  [STOP] obstacle at {dist:.1f} cm")
        return 'obstacle'
    if _border():
        _stop_all()
        print("  [STOP] BLACK border")
        return 'border'
    return None

# ── IR check ──────────────────────────────────────────────────────────────────
def _check_ir():
    ir1 = sensors.ir1_val()
    ir2 = sensors.ir2_val()
    print(f"    ir1={ir1}  ir2={ir2}")
    if ir1 == 0 and ir2 == 0:
        return 'center'
    if ir1 == 0:
        return 'left'
    if ir2 == 0:
        return 'right'
    return None

# ── Sweep at current position ─────────────────────────────────────────────────
SWEEP_STEPS = [
    ('LEFT',   'left',  TURN_MS),
    ('CENTER', 'right', TURN_MS),
    ('RIGHT',  'right', TURN_MS),
    ('CENTER', 'left',  TURN_MS),
]

def _sweep():
    """
    Do one full LEFT→CENTER→RIGHT→CENTER sweep.
    Checks IR and priority stops at each position.
    Returns ball position string, stop reason string, or None if nothing found.
    """
    for label, direction, dur in SWEEP_STEPS:

        # priority check before each move
        reason = _priority_stop()
        if reason:
            return reason

        print(f"  @ {label}")
        result = _check_ir()
        if result is not None:
            _stop_all()
            return result   # 'left', 'right', or 'center'

        # turn to next position
        _s1_spin()
        if direction == 'left':
            motors.set_left_rev(TURN_SPEED)
            motors.set_right_fwd(TURN_SPEED)
        else:
            motors.set_left_fwd(TURN_SPEED)
            motors.set_right_rev(TURN_SPEED)

        time.sleep_ms(dur)
        _stop_all()
        time.sleep_ms(TURN_COOL_MS)

    return None   # nothing found this sweep

# ── Creep forward ─────────────────────────────────────────────────────────────
def _creep():
    """
    Drive forward for CREEP_MS, checking priority stops every tick.
    Returns stop reason if triggered, None if completed cleanly.
    """
    _s1_spin()
    motors.set_left_fwd(CREEP_SPEED_L)
    motors.set_right_fwd(CREEP_SPEED_R)

    deadline = time.ticks_ms() + CREEP_MS
    while time.ticks_diff(time.ticks_ms(), deadline) < 0:
        reason = _priority_stop()
        if reason:
            return reason
        time.sleep_ms(20)

    _stop_all()
    time.sleep_ms(TURN_COOL_MS)
    return None

# ── Carry mode ────────────────────────────────────────────────────────────────
def _carry():
    """
    Drive forward steering by IR. Priority stops checked every tick.
    Returns stop reason and distance traveled.
    """
    print("[CARRY] driving with ball...")
    encoders.reset()
    _s1_spin()
    deadline = time.ticks_ms() + CARRY_TIMEOUT_MS

    while time.ticks_diff(time.ticks_ms(), deadline) < 0:

        # priority stop — highest priority
        reason = _priority_stop()
        if reason:
            return reason, (encoders.left_cm() + encoders.right_cm()) / 2

        ir1 = sensors.ir1_val()
        ir2 = sensors.ir2_val()

        if ir1 == 0 and ir2 == 0:
            motors.set_left_fwd(CARRY_SPEED_L)
            motors.set_right_fwd(CARRY_SPEED_R)
        elif ir1 == 0:
            # ball left — curve left
            motors.set_left_fwd(CARRY_SPEED_L)
            motors.set_right_fwd(max(0, CARRY_SPEED_R - NUDGE_REDUCE))
        elif ir2 == 0:
            # ball right — curve right
            motors.set_left_fwd(max(0, CARRY_SPEED_L - NUDGE_REDUCE))
            motors.set_right_fwd(CARRY_SPEED_R)
        else:
            # lost ball — creep and re-sweep next cycle
            _stop_all()
            return 'lost_ball', (encoders.left_cm() + encoders.right_cm()) / 2

        time.sleep_ms(20)

    _stop_all()
    return 'timeout', (encoders.left_cm() + encoders.right_cm()) / 2

# ── Main ──────────────────────────────────────────────────────────────────────
def run():
    print("=== BALL MANEUVERABILITY TEST START ===")
    print("Priority: obstacle > border > ball search\n")

    deadline     = time.ticks_ms() + SCAN_TIMEOUT_MS
    total_dist   = 0.0
    cycle        = 0

    while time.ticks_diff(time.ticks_ms(), deadline) < 0:
        cycle += 1
        print(f"\n── cycle {cycle} ──")

        # 1. priority check before doing anything
        reason = _priority_stop()
        if reason:
            print(f"\n=== STOPPED: {reason} ===")
            print(f"  Total distance driven: {total_dist:.1f} cm")
            return

        # 2. creep forward a little
        print("  creeping forward...")
        reason = _creep()
        if reason:
            print(f"\n=== STOPPED during creep: {reason} ===")
            return

        # 3. sweep and check IR
        print("  sweeping...")
        result = _sweep()

        if result in ('obstacle', 'border', 'overcurrent'):
            print(f"\n=== STOPPED during sweep: {result} ===")
            return

        if result in ('left', 'center', 'right'):
            print(f"  Ball found at: {result} — switching to carry")
            time.sleep_ms(TURN_COOL_MS)

            stop_reason, dist = _carry()
            total_dist += dist

            if stop_reason == 'lost_ball':
                print("  Ball lost — resuming search")
                continue   # go back to creep+sweep loop

            # hard stop
            passed = stop_reason in ('obstacle', 'border') and total_dist > 5.0
            print(f"\n=== BALL MANEUVERABILITY TEST DONE ===")
            print(f"  Stop reason    : {stop_reason}")
            print(f"  Distance driven: {total_dist:.1f} cm")
            print(f"  PASS           : {'YES' if passed else 'NO'}")
            return

        # result is None — nothing found this sweep, loop again

    _stop_all()
    print("\n=== BALL MANEUVERABILITY TEST DONE ===")
    print("  PASS : NO — scan timeout")

run()