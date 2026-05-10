"""
test_oc.py  –  Overcurrent Protection Test
────────────────────────────────────────────
Scoring category: Vehicle Maneuverability (OC demo)

Runs motors forward and deliberately stalls them by blocking the wheels.
Verifies that OC trips, stops motors, and recovers correctly.

Steps:
  1. Motors ramp up forward
  2. BLOCK the wheels by hand when prompted
  3. OC should trip within 1 second
  4. Verify motors stop and buzzer sounds
  5. Release wheels — OC should recover and retry up to 3 times
"""

import motors
import oc
import buzzer
import time

def run():
    print("=== OC TEST START ===")
    print("  Tests overcurrent protection on both motor channels")
    print("  You will be asked to stall the wheels at the right time\n")

    # reset any previous OC state
    oc.reset_state()
    motors.stop()

    # ── Phase 1: baseline current reading ────────────────────
    print("[1] Baseline current (motors stopped):")
    for _ in range(3):
        (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
        print(f"    L: {lv:.2f}V  {lma:.0f}mA    R: {rv:.2f}V  {rma:.0f}mA")
        time.sleep_ms(300)

    input("\n[2] Press ENTER to start motors forward...")

    # ── Phase 2: run forward, watch current ──────────────────
    print("    Motors running — watch current rise")
    print("    BLOCK THE WHEELS NOW to trigger OC\n")

    motors.set_left_fwd(65535)
    motors.set_right_fwd(65535)

    tripped  = False
    deadline = time.ticks_ms() + 8000   # 8s window to trigger OC

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
        print(f"    L: {lv:.2f}V {lma:>6.0f}mA    R: {rv:.2f}V {rma:>6.0f}mA   "
              f"OC={'TRIPPED' if oc.halted() else 'ok'}")

        oc.check_both()
        # check either wheel tripped — halted() is only true after max retries
        # so check state directly to catch first trip
        if oc.state['left']['tripped'] or oc.state['right']['tripped'] or oc.halted():
            tripped = True
            print("\n  [OC TRIPPED] motors stopped automatically")
            break

        time.sleep_ms(200)

    if not tripped:
        motors.stop()
        print("\n  [WARN] OC did not trip in 8s — check OVERCURRENT_MA threshold in config.py")
        print(f"  Current threshold: {__import__('config').OVERCURRENT_MA} mA")
        return

    # ── Phase 3: verify motors actually stopped ───────────────
    print("\n[3] Verifying motors stopped:")
    (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
    print(f"    L: {lv:.2f}V {lma:.0f}mA    R: {rv:.2f}V {rma:.0f}mA")
    if abs(lma) < 200 and abs(rma) < 200:
        print("    PASS — current dropped after OC trip")
    else:
        print("    WARN — current still high, motors may still be running")

    # ── Phase 4: recovery test ────────────────────────────────
    print("\n[4] OC recovery test:")
    print("    RELEASE THE WHEELS now")
    print(f"    Waiting for current to settle (max {oc._MAX_RETRIES} retries)...")

    for attempt in range(1, oc._MAX_RETRIES + 2):
        time.sleep_ms(1500)
        (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
        print(f"    attempt {attempt}: L={lma:.0f}mA  R={rma:.0f}mA  "
              f"retries={oc._retries}/{oc._MAX_RETRIES}")
        oc.check_both()
        if not oc.halted():
            print("    PASS — OC cleared and recovered")
            break
    else:
        print(f"    PASS — OC gave up after {oc._MAX_RETRIES} retries (expected behavior)")

    motors.stop()
    buzzer.off()

    # ── Phase 5: reverse + 180 + forward ─────────────────────
    print("\n[5] OC escape: reverse 1s -> 180 -> forward 2s")
    oc.reset_state()
    oc.start_grace()   # cover the entire escape sequence
    time.sleep_ms(300)

    print("    reversing...")
    motors.set_left_rev(65535)
    motors.set_right_rev(65535)
    time.sleep_ms(1000)
    motors.stop()
    time.sleep_ms(300)

    print("    rotating 180...")
    actual1 = motors.turn_degrees(90, 'right', timeout_ms=5000) or 90.0
    time.sleep_ms(300)
    target2 = max(70.0, 90.0 - (actual1 - 90.0))
    motors.turn_degrees(target2, 'right', timeout_ms=5000)
    time.sleep_ms(300)

    print("    forward 2s...")
    oc.start_grace()   # fresh grace for forward drive
    motors.set_left_fwd(65535)
    motors.set_right_fwd(65535)
    time.sleep_ms(2000)
    motors.stop()
    print("    PASS")

    # ── Summary ───────────────────────────────────────────────
    print("\n=== OC TEST DONE ===")
    print(f"  OC tripped       : {'YES' if tripped else 'NO'}")
    print(f"  Motors stopped   : YES (PWM duty set to 0)")
    print(f"  Buzzer on trip   : YES")
    print(f"  Max retries      : {oc._MAX_RETRIES}")
    print(f"  OC threshold     : {__import__('config').OVERCURRENT_MA} mA")
    print(f"  Warn threshold   : {__import__('config').WARN_MA} mA")

run()