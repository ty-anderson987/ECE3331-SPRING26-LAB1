"""
test_system.py  –  Full system diagnostic (no H-bridge required)
─────────────────────────────────────────────────────────────────
Tests every sensor and peripheral one at a time.
Press ENTER in Thonny to advance to the next test.
Reports PASS / FAIL for each.
"""

from machine import PWM, Pin, I2C
import time

results = {}

def wait():
    input("  → press ENTER to continue...")

def header(name):
    print(f"\n{'─'*40}")
    print(f"  TEST: {name}")
    print(f"{'─'*40}")

def ok(name):
    results[name] = "PASS"
    print(f"  [PASS] {name}")

def fail(name, reason=""):
    results[name] = f"FAIL: {reason}"
    print(f"  [FAIL] {name}  {reason}")

# ── 1. I2C bus scan ───────────────────────────────────────────
header("I2C bus")
try:
    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
    devices = i2c.scan()
    print(f"  found: {[hex(d) for d in devices]}")
    expected = {0x40: "INA219 right", 0x41: "INA219 left", 0x68: "MPU6050"}
    for addr, name in expected.items():
        if addr in devices:
            ok(name)
        else:
            fail(name, f"not found at {hex(addr)}")
except Exception as e:
    fail("I2C", str(e))

wait()

# ── 2. IMU ────────────────────────────────────────────────────
header("IMU (MPU6050)")
try:
    from sensors import read_imu, read_gyro_z
    for _ in range(5):
        d = read_imu()
        print(f"  ax={d['ax']:+.2f} ay={d['ay']:+.2f} az={d['az']:+.2f}  gz={d['gz']:+.2f}  temp={d['temp']:.1f}C")
        time.sleep_ms(200)
    ok("IMU")
except Exception as e:
    fail("IMU", str(e))

wait()

# ── 3. Sonar 1 (forward) ──────────────────────────────────────
header("Sonar 1 — fixed forward")
try:
    from sensors import sonar1
    for _ in range(5):
        d = sonar1()
        print(f"  sonar1: {d:.1f} cm")
        time.sleep_ms(300)
    ok("Sonar1")
except Exception as e:
    fail("Sonar1", str(e))

wait()

# ── 4. Sonar 2 + servo sweep ──────────────────────────────────
header("Sonar 2 + servo2 sweep")
try:
    from sensors import sweep_scan
    print("  sweeping...")
    scan, closest = sweep_scan(step=20)
    for angle, dist in sorted(scan.items()):
        print(f"    {angle:>4}°  {dist:>6.1f} cm")
    print(f"  closest: {closest[1]:.1f} cm at {closest[0]}°")
    ok("Sonar2 + Servo2")
except Exception as e:
    fail("Sonar2 + Servo2", str(e))

wait()

# ── 5. IR obstacle sensors ────────────────────────────────────
header("IR sensors (front + side)")
print("  wave hand in front of each sensor — watch values change")
try:
    from sensors import ir1_val, ir2_val, ir_left_val, ir_right_val
    for _ in range(20):
        print(f"  IR1={ir1_val()}  IR2={ir2_val()}  L={ir_left_val()}  R={ir_right_val()}")
        time.sleep_ms(300)
    ok("IR sensors")
except Exception as e:
    fail("IR sensors", str(e))

wait()

# ── 6. Color sensor ───────────────────────────────────────────
header("Color sensor (ball=OUT1, floor=OUT2)")
try:
    import color
    for _ in range(10):
        r = color._freq(color.OUT1, 0, 0)
        g = color._freq(color.OUT1, 1, 1)
        b = color._freq(color.OUT1, 0, 1)
        total = r + g + b
        ball = color.ball()
        floor = color.floor()
        print(f"  ball r={r} g={g} b={b} total={total} → {ball}  |  floor → {floor}")
        time.sleep_ms(300)
    ok("Color sensor")
except Exception as e:
    fail("Color sensor", str(e))

wait()

# ── 7. INA219 current sensors ─────────────────────────────────
header("INA219 (current/voltage)")
try:
    import oc
    for _ in range(5):
        (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
        print(f"  L: {lv:.2f}V  {lma:.0f}mA  {lmw:.0f}mW    R: {rv:.2f}V  {rma:.0f}mA  {rmw:.0f}mW")
        time.sleep_ms(400)
    ok("INA219")
except Exception as e:
    fail("INA219", str(e))

wait()

# ── 8. Buzzer ─────────────────────────────────────────────────
header("Buzzer")
try:
    import buzzer
    buzzer.beep(n=3, ms=100, gap=100)
    ok("Buzzer")
except Exception as e:
    fail("Buzzer", str(e))

wait()

# ── 9. Encoders ───────────────────────────────────────────────
header("Encoders")
print("  spin each wheel by hand — watch tick counts")
try:
    import encoders
    encoders.reset()
    for _ in range(10):
        lt, rt = encoders.get()
        print(f"  L={lt}  R={rt}")
        time.sleep_ms(400)
    ok("Encoders")
except Exception as e:
    fail("Encoders", str(e))

wait()

# ── 10. Servo 1 ───────────────────────────────────────────────
header("Servo 1 (intake)")
try:
    from config import PIN_SERVO1, SERVO1_SPIN, SERVO1_STOP
    s1 = PWM(Pin(PIN_SERVO1))
    s1.freq(50)
    def _s1(us): s1.duty_u16(int(us / 20000 * 65535))
    print("  spinning forward 2s...")
    _s1(SERVO1_SPIN)
    time.sleep_ms(2000)
    print("  stopping...")
    _s1(SERVO1_STOP)
    time.sleep_ms(500)
    rev = SERVO1_STOP + (SERVO1_STOP - SERVO1_SPIN)
    print("  spinning reverse 2s...")
    _s1(rev)
    time.sleep_ms(2000)
    _s1(SERVO1_STOP)
    ok("Servo1")
except Exception as e:
    fail("Servo1", str(e))

wait()

# ── 11. IR remote ─────────────────────────────────────────────
header("IR remote")
print("  press any button on the remote within 5 seconds...")
try:
    import ir_remote
    received = [None]
    def _cb(code):
        received[0] = code
        print(f"  received: {hex(code)}")
    ir_remote.set_callback(_cb)
    deadline = time.ticks_ms() + 5000
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        ir_remote.check()
        time.sleep_ms(20)
    if received[0]:
        ok("IR remote")
    else:
        fail("IR remote", "no signal received in 5s")
except Exception as e:
    fail("IR remote", str(e))

# ── Summary ───────────────────────────────────────────────────
print(f"\n{'═'*40}")
print("  SYSTEM TEST SUMMARY")
print(f"{'═'*40}")
for name, result in results.items():
    print(f"  {'✓' if result == 'PASS' else '✗'}  {name:<25} {result}")
print(f"{'═'*40}")
passed = sum(1 for r in results.values() if r == "PASS")
print(f"  {passed}/{len(results)} passed")