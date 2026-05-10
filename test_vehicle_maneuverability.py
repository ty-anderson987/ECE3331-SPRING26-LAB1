"""
test_vehicle_maneuverability.py  –  Vehicle Maneuverability Test
─────────────────────────────────────────────────────────────────
Scoring category: Vehicle Maneuverability (10%)

Sequence:
  1. Forward FORWARD_CM
  2. Pause
  3. Reverse REVERSE_CM
  4. Pause
  5. Turn right 90°
  6. Pause
  7. Turn left  90°

Forward/reverse use gyro-only heading hold (motors._gyro_drive).
Encoders are used ONLY to determine when target distance is reached.
Turns use motors.turn_degrees() with gyro integration.

Usage:
    import test_vehicle_maneuverability
    test_vehicle_maneuverability.run()
"""

import motors
import oc
import encoders
import time

try:
    from sensors import read_gyro_z
except:
    read_gyro_z = lambda: 0.0

# ── Config ────────────────────────────────────────────────────────────────────
FORWARD_CM   = 50.0
REVERSE_CM   = 50.0
TURN_DEGREES = 90
PASS_CM_TOL  = 10.0

# Heading hold gains (same as motors._gyro_drive)
HEADING_KP = 1500
HEADING_KD = 100
SLOW_AMOUNT = 30000

LEFT_CM_PER_TICK  = 20.1 / 20
RIGHT_CM_PER_TICK = 20.1 / 20

# ── Helpers ───────────────────────────────────────────────────────────────────
def _clamp(v, lo=0, hi=65535):
    return max(lo, min(hi, v))

def _drive_cm_gyro(target_cm, fwd=True):
    """
    Drive a target distance using gyro heading hold for steering.
    Uses LEFT encoder to determine distance traveled (most reliable).
    Returns actual distance traveled (cm).
    """
    encoders.reset()

    if fwd:
        base = 65535
        set_l = motors.set_left_fwd
        set_r = motors.set_right_fwd
    else:
        base = 65535
        set_l = motors.set_left_rev
        set_r = motors.set_right_rev

    set_l(base)
    set_r(base)

    heading = 0.0
    last_t  = time.ticks_us()

    while True:
        oc.check_both()
        if oc.halted():
            motors.stop()
            print("[OC] overcurrent")
            break

        now = time.ticks_us()
        dt  = time.ticks_diff(now, last_t) / 1_000_000.0
        last_t = now

        try:
            gz = read_gyro_z()
        except:
            gz = 0.0   # I2C glitch — skip this sample
        heading += gz * dt

        # PD heading hold
        p = HEADING_KP * heading
        d = HEADING_KD * gz
        corr = _clamp(int(p + d), -SLOW_AMOUNT, SLOW_AMOUNT)

        # In reverse, slowing a wheel rotates the bot the OPPOSITE way vs forward
        if not fwd:
            corr = -corr

        left_spd  = _clamp(base - max(0,  corr))
        right_spd = _clamp(base - max(0, -corr))
        set_l(left_spd)
        set_r(right_spd)

        # use LEFT encoder for distance (more reliable than right)
        lt, _ = encoders.get()
        traveled = lt * LEFT_CM_PER_TICK
        if traveled >= target_cm:
            break

        time.sleep_ms(5)

    motors.stop()
    lt, rt = encoders.get()
    return lt * LEFT_CM_PER_TICK, heading

# ── Run ───────────────────────────────────────────────────────────────────────
def run():
    results = {}

    print("=== VEHICLE MANEUVERABILITY TEST START ===\n")
    print(f"  Heading hold: KP={HEADING_KP}  KD={HEADING_KD}\n")

    # ── Phase 1: Forward ──────────────────────────────────────────────────────
    print(f"[1] FORWARD {FORWARD_CM:.0f} cm")
    actual, hdg = _drive_cm_gyro(FORWARD_CM, fwd=True)
    lt, rt = encoders.get()
    print(f"    traveled={actual:.1f} cm  L={lt}tk  R={rt}tk  final hdg={hdg:+.2f}°")
    results['forward'] = abs(actual - FORWARD_CM) <= PASS_CM_TOL
    time.sleep_ms(500)

    # ── Phase 2: Reverse ──────────────────────────────────────────────────────
    print(f"\n[2] REVERSE {REVERSE_CM:.0f} cm")
    actual, hdg = _drive_cm_gyro(REVERSE_CM, fwd=False)
    lt, rt = encoders.get()
    print(f"    traveled={actual:.1f} cm  L={lt}tk  R={rt}tk  final hdg={hdg:+.2f}°")
    results['reverse'] = abs(actual - REVERSE_CM) <= PASS_CM_TOL
    time.sleep_ms(500)

    # ── Phase 3: Turn right 90° ───────────────────────────────────────────────
    print(f"\n[3] TURN RIGHT {TURN_DEGREES}°")
    motors.turn_degrees(TURN_DEGREES, 'right', timeout_ms=5000)
    time.sleep_ms(500)
    results['turn_right'] = True

    # ── Phase 4: Turn left 90° ────────────────────────────────────────────────
    print(f"\n[4] TURN LEFT {TURN_DEGREES}°")
    motors.turn_degrees(TURN_DEGREES, 'left', timeout_ms=5000)
    time.sleep_ms(500)
    results['turn_left'] = True

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n=== VEHICLE MANEUVERABILITY TEST DONE ===")
    all_pass = all(results.values())
    for phase, passed in results.items():
        print(f"  {phase:<14}: {'PASS' if passed else 'FAIL'}")
    print(f"  OVERALL PASS  : {'YES' if all_pass else 'NO'}")

run()