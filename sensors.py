from machine import Pin, I2C, PWM
from config import (PIN_TRIG1, PIN_ECHO1, PIN_TRIG2, PIN_ECHO2,
                    PIN_IR1, PIN_IR2, PIN_IR_LEFT, PIN_IR_RIGHT, PIN_SERVO2,
                    SWEEP_CENTER, SWEEP_RANGE, SWEEP_MS,
                    SONAR_OFFSET_CM, SONAR1_STOP_CM, SONAR2_STOP_CM,
                    ADDR_MPU, GYRO_OFFSET)
import time
import struct

# ── Sweep dwell ───────────────────────────────────────────────
try:
    from config import SWEEP_DWELL_MS
except ImportError:
    SWEEP_DWELL_MS = 1000

# ── Sonars ────────────────────────────────────────────────────
trig1 = Pin(PIN_TRIG1, Pin.OUT)
echo1 = Pin(PIN_ECHO1, Pin.IN)
trig2 = Pin(PIN_TRIG2, Pin.OUT)
echo2 = Pin(PIN_ECHO2, Pin.IN)

# ── IR obstacle sensors ───────────────────────────────────────
ir1      = Pin(PIN_IR1,     Pin.IN)
ir2      = Pin(PIN_IR2,     Pin.IN)
ir_left  = Pin(PIN_IR_LEFT,  Pin.IN)
ir_right = Pin(PIN_IR_RIGHT, Pin.IN)

# ── Servo2 sweep ──────────────────────────────────────────────
_servo2        = PWM(Pin(PIN_SERVO2))
_servo2.freq(50)
_current_angle = SWEEP_CENTER

def _angle_to_duty(angle):
    pulse = int(544 + (angle / 180) * (2400 - 544))
    return int(pulse / 20000 * 65535)

def _servo2_set(angle):
    global _current_angle
    _servo2.duty_u16(_angle_to_duty(angle))
    _current_angle = angle

# center on boot and hold — re-asserts duty so PWM slice conflicts don't nudge it
_servo2_set(SWEEP_CENTER)
time.sleep_ms(300)
_servo2_set(SWEEP_CENTER)  # write twice to lock the slice value

def servo2_hold_center():
    """Call this after any other servo on the same PWM slice is written."""
    _servo2_set(SWEEP_CENTER)

# ── MPU6050 ───────────────────────────────────────────────────
_imu_i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

_MPU_PWR   = 0x6B
_MPU_CFG   = 0x1A
_MPU_GCFG  = 0x1B
_MPU_ACFG  = 0x1C
_MPU_ACC_H = 0x3B
_MPU_GZ_H  = 0x47
_MPU_TEMP  = 0x41

_GYRO_SCALE = 131.0
_ACC_SCALE  = 16384.0

def _mpu_init():
    _imu_i2c.writeto_mem(ADDR_MPU, _MPU_PWR,  bytes([0x00]))
    _imu_i2c.writeto_mem(ADDR_MPU, _MPU_CFG,  bytes([0x01]))
    _imu_i2c.writeto_mem(ADDR_MPU, _MPU_GCFG, bytes([0x00]))
    _imu_i2c.writeto_mem(ADDR_MPU, _MPU_ACFG, bytes([0x00]))

_mpu_init()

# ── Sonar helpers ─────────────────────────────────────────────
def _sonar_read(trig, echo):
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    start = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > 30000:
            return 999
    t1 = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), t1) > 30000:
            return 999
    return (time.ticks_diff(time.ticks_us(), t1) * 0.0343) / 2

def _sonar_filtered(trig, echo, samples=3):
    readings = []
    for _ in range(samples):
        r = _sonar_read(trig, echo)
        if r < 400:
            readings.append(r)
        time.sleep_ms(10)
    return min(readings) if readings else 999

# ── Sonar1 fixed forward ──────────────────────────────────────
def sonar1():
    return _sonar_read(trig1, echo1)

def sonar1_filtered():
    return _sonar_filtered(trig1, echo1)

def sonar1_clear():
    return sonar1() > SONAR1_STOP_CM

# ── Sonar2 sweeping ───────────────────────────────────────────
def sonar2():
    return _sonar_read(trig2, echo2)

def sonar2_calibrated():
    return sonar2() + SONAR_OFFSET_CM

def sonar2_clear():
    return sonar2_calibrated() > SONAR2_STOP_CM

# ── Sweep scan ────────────────────────────────────────────────
def sweep_scan(step=10, settle_ms=None, dwell_ms=None):
    if settle_ms is None:
        settle_ms = max(SWEEP_MS, 50)
    if dwell_ms is None:
        dwell_ms = SWEEP_DWELL_MS

    lo      = SWEEP_CENTER - SWEEP_RANGE
    hi      = SWEEP_CENTER + SWEEP_RANGE
    scan    = {}
    closest = (SWEEP_CENTER, 999)

    def _read_and_store(angle):
        nonlocal closest
        dist = sonar2_calibrated()
        scan[angle] = dist
        if dist < closest[1]:
            closest = (angle, dist)

    # center
    _servo2_set(SWEEP_CENTER)
    time.sleep_ms(settle_ms)
    _read_and_store(SWEEP_CENTER)

    # sweep left — dwell at extreme
    for angle in range(SWEEP_CENTER - step, lo - 1, -step):
        _servo2_set(angle)
        time.sleep_ms(dwell_ms if angle == lo else settle_ms)
        _read_and_store(angle)

    # smooth return to center before sweeping right
    for angle in range(lo, SWEEP_CENTER + step, step):
        _servo2_set(angle)
        time.sleep_ms(settle_ms)

    # sweep right — dwell at extreme
    for angle in range(SWEEP_CENTER + step, hi + 1, step):
        _servo2_set(angle)
        time.sleep_ms(dwell_ms if angle == hi else settle_ms)
        _read_and_store(angle)

    # return to center
    for angle in range(hi, SWEEP_CENTER - step, -step):
        _servo2_set(angle)
        time.sleep_ms(settle_ms)

    return scan, closest

def best_direction(step=10):
    scan, _ = sweep_scan(step=step)
    best    = max(scan.items(), key=lambda x: x[1])
    return best

def print_sweep(step=10):
    scan, closest = sweep_scan(step=step)
    print("---- SWEEP SCAN ----")
    for angle, dist in sorted(scan.items()):
        bar    = '█' * min(int(dist / 5), 30)
        marker = ' <-- closest' if angle == closest[0] else ''
        print(f"  {angle:>4}°  {dist:>6.1f} cm  {bar}{marker}")
    print(f"  closest: {closest[1]:.1f} cm at {closest[0]}°")
    print("--------------------")

# ── IR sensors ────────────────────────────────────────────────
def ir_ball():
    return ir1.value() == 0 or ir2.value() == 0

def ir1_val():
    return ir1.value()

def ir2_val():
    return ir2.value()

def ir_left_val():
    return ir_left.value()

def ir_right_val():
    return ir_right.value()

# ── IMU ───────────────────────────────────────────────────────
def read_gyro_z():
    raw = _imu_i2c.readfrom_mem(ADDR_MPU, _MPU_GZ_H, 2)
    gz  = struct.unpack('>h', raw)[0]
    return -(gz / _GYRO_SCALE - GYRO_OFFSET)

def read_imu():
    raw_a      = _imu_i2c.readfrom_mem(ADDR_MPU, _MPU_ACC_H, 6)
    ax, ay, az = struct.unpack('>hhh', raw_a)
    raw_g      = _imu_i2c.readfrom_mem(ADDR_MPU, _MPU_GZ_H, 2)
    gz         = struct.unpack('>h', raw_g)[0]
    raw_t      = _imu_i2c.readfrom_mem(ADDR_MPU, _MPU_TEMP, 2)
    temp_raw   = struct.unpack('>h', raw_t)[0]
    return dict(
        ax   = ax / _ACC_SCALE,
        ay   = ay / _ACC_SCALE,
        az   = az / _ACC_SCALE,
        gz   = -(gz / _GYRO_SCALE - GYRO_OFFSET),
        temp = temp_raw / 340.0 + 36.53,
    )

def print_imu():
    d = read_imu()
    print("---- IMU ----")
    print(f"  Accel  X: {d['ax']:+.3f} g   Y: {d['ay']:+.3f} g   Z: {d['az']:+.3f} g")
    print(f"  Gyro   Z: {d['gz']:+.3f} deg/s")
    print(f"  Temp      {d['temp']:.1f} C")
    print("-------------")