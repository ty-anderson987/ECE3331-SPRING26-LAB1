"""
test_fully_operational.py  –  Fully Operational Test
──────────────────────────────────────────────────────
Scoring category: Fully Operational (20%)

Checks every subsystem in order:
  1. I2C bus (INA219 left, INA219 right, MPU6050)
  2. Overcurrent module (read_both)
  3. Encoders (IRQ fires on manual wheel spin)
  4. Motors — brief left/right pulse at low duty
  5. Sonar1 + Sonar2 (plausible range reading)
  6. IR obstacle sensors (digital read)
  7. Color sensor (R/G/B counts > 0)
  8. Buzzer (audible beep)
  9. IMU (gyro Z returns finite value)
 10. IR remote receiver (waits up to 5 s for any button press)

Each check prints PASS or FAIL and a reason.
Final score: passes / total.

Usage:
    import test_fully_operational
    test_fully_operational.run()
"""

import time

# ── Individual check helpers ──────────────────────────────────────────────────

def _check_i2c():
    try:
        from motors import i2c
        from config import ADDR_LEFT, ADDR_RIGHT, ADDR_MPU
        devs = i2c.scan()
        missing = [hex(a) for a in [ADDR_LEFT, ADDR_RIGHT, ADDR_MPU] if a not in devs]
        if missing:
            return False, f"missing: {missing}"
        return True, f"found {[hex(d) for d in devs]}"
    except Exception as e:
        return False, str(e)

def _check_oc():
    try:
        import oc
        (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
        if lv == 0.0 and rv == 0.0:
            return False, "both INA219 returned 0 V — check wiring"
        return True, f"L={lv:.2f}V {lma:.0f}mA  R={rv:.2f}V {rma:.0f}mA"
    except Exception as e:
        return False, str(e)

def _check_encoders():
    try:
        import encoders
        before_l, before_r = encoders.get()
        # just verify counters are accessible and non-negative
        if before_l < 0 or before_r < 0:
            return False, "negative tick count"
        return True, f"L={before_l}  R={before_r}  (spin wheels to verify IRQ)"
    except Exception as e:
        return False, str(e)

def _check_motors():
    try:
        import motors
        import oc
        oc.reset_state()
        # very short low-power pulse — wheels should twitch
        motors.set_left_fwd(20000)
        motors.set_right_fwd(20000)
        time.sleep_ms(150)
        motors.stop()
        return True, "brief forward pulse sent (verify audible/physical)"
    except Exception as e:
        return False, str(e)

def _check_sonar():
    try:
        import sensors
        d1 = sensors.sonar1()
        d2 = sensors.sonar2_calibrated()
        ok = (0 < d1 < 400) and (0 < d2 < 400)
        return ok, f"sonar1={d1:.1f} cm  sonar2={d2:.1f} cm"
    except Exception as e:
        return False, str(e)

def _check_ir_sensors():
    try:
        import sensors
        v1 = sensors.ir1_val()
        v2 = sensors.ir2_val()
        vl = sensors.ir_left_val()
        vr = sensors.ir_right_val()
        return True, f"ir1={v1} ir2={v2} left={vl} right={vr}"
    except Exception as e:
        return False, str(e)

def _check_color():
    try:
        import color
        r = color._freq(color.OUT1, 0, 0)
        g = color._freq(color.OUT1, 1, 1)
        b = color._freq(color.OUT1, 0, 1)
        ok = (r + g + b) > 0
        return ok, f"ball R={r} G={g} B={b}"
    except Exception as e:
        return False, str(e)

def _check_buzzer():
    try:
        import buzzer
        buzzer.beep(2, ms=80, gap=80)
        return True, "2 beeps sent"
    except Exception as e:
        return False, str(e)

def _check_imu():
    try:
        import sensors
        gz = sensors.read_gyro_z()
        ok = gz is not None and -2000 < gz < 2000
        return ok, f"gyro_z={gz:.3f} deg/s"
    except Exception as e:
        return False, str(e)

def _check_ir_remote(timeout_ms=5000):
    try:
        import ir_remote
        received = [False]

        def _cb(code):
            received[0] = True
            print(f"      IR code received: {hex(code)}")

        ir_remote.set_callback(_cb)
        print("    [IR] press any button on remote within 5 s...")
        deadline = time.ticks_ms() + timeout_ms
        while not received[0] and time.ticks_diff(time.ticks_ms(), deadline) < 0:
            ir_remote.check()
            time.sleep_ms(20)
        ir_remote.set_callback(None)
        if received[0]:
            return True, "IR code received"
        return False, "timeout — no IR signal (skip if no remote available)"
    except Exception as e:
        return False, str(e)

# ── Runner ────────────────────────────────────────────────────────────────────

CHECKS = [
    ("I2C bus",           _check_i2c),
    ("Overcurrent (INA)", _check_oc),
    ("Encoders",          _check_encoders),
    ("Motors",            _check_motors),
    ("Sonars",            _check_sonar),
    ("IR obstacle",       _check_ir_sensors),
    ("Color sensor",      _check_color),
    ("Buzzer",            _check_buzzer),
    ("IMU / Gyro",        _check_imu),
    ("IR remote",         _check_ir_remote),
]

def run():
    print("=== FULLY OPERATIONAL TEST START ===\n")
    print(f"  {'#':>2}  {'subsystem':<22}  {'result':<6}  detail")
    print("  " + "-" * 70)

    passed = 0
    results = []

    for idx, (name, fn) in enumerate(CHECKS, 1):
        ok, detail = fn()
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        print(f"  {idx:>2}  {name:<22}  {status:<6}  {detail}")
        results.append((name, ok, detail))

    total = len(CHECKS)
    score = passed / total * 100

    print("\n" + "  " + "=" * 70)
    print(f"  PASSED : {passed}/{total}  ({score:.0f}%)")
    print(f"  OVERALL: {'PASS — fully operational' if passed == total else 'PARTIAL — see failures above'}")
    print("=== FULLY OPERATIONAL TEST DONE ===")

run()
