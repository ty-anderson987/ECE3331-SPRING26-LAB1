"""
gyro_drive.py — Gyro-only straight-line driving
─────────────────────────────────────────────────
Drops the encoder error PID entirely. Uses ONLY the gyro to hold heading.

Logic:
  - Track heading angle by integrating gyro yaw rate
  - If heading drifts off zero, slow whichever wheel pulls back to zero
  - That's it. No encoder math, no ratio learning, no modes.

Drive:
    import gyro_drive
    gyro_drive.run()
"""

from machine import PWM, Pin, I2C
import time

# ── Pins ────────────────────────────────────────────────────────────────────
PIN_LF = 5
PIN_LR = 4
PIN_RF = 3
PIN_RR = 2
PWM_FREQ = 20000

# ── Speeds ──────────────────────────────────────────────────────────────────
FWD_BASE = 65535
SLOW_AMOUNT = 30000        # max correction either side

# ── Gyro PID ────────────────────────────────────────────────────────────────
KP_HEADING = 1500          # how hard to correct heading angle (degrees)
KD_HEADING = 100           # how hard to correct rotation rate

# ── Gyro hardware ───────────────────────────────────────────────────────────
ADDR_MPU = 0x68
GYRO_OFFSET = 2.75         # bias when still
GYRO_INVERTED = True       # your hardware reports right rotation as negative

# ── Test ────────────────────────────────────────────────────────────────────
DURATION_MS = 4000
LOG_MS = 100

# ═══════════════════════════════════════════════════════════════════════════════
# SETUP
# ═══════════════════════════════════════════════════════════════════════════════
_LF = PWM(Pin(PIN_LF)); _LF.freq(PWM_FREQ); _LF.duty_u16(0)
_LR = PWM(Pin(PIN_LR)); _LR.freq(PWM_FREQ); _LR.duty_u16(0)
_RF = PWM(Pin(PIN_RF)); _RF.freq(PWM_FREQ); _RF.duty_u16(0)
_RR = PWM(Pin(PIN_RR)); _RR.freq(PWM_FREQ); _RR.duty_u16(0)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# wake MPU
try:
    i2c.writeto_mem(ADDR_MPU, 0x6B, b'\x00')
    i2c.writeto_mem(ADDR_MPU, 0x1B, b'\x00')
except:
    pass

def read_gyro_z():
    """+ve = rotating right. Bias subtracted. Inverted if hardware needs it."""
    try:
        data = i2c.readfrom_mem(ADDR_MPU, 0x47, 2)
        raw = (data[0] << 8) | data[1]
        if raw & 0x8000:
            raw -= 0x10000
        gz = (raw / 131.0) - GYRO_OFFSET
        return -gz if GYRO_INVERTED else gz
    except:
        return 0.0

def _clamp(v, lo=0, hi=65535):
    return max(lo, min(hi, v))

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def run(duration_ms=DURATION_MS):
    print("=" * 80)
    print("GYRO-ONLY STRAIGHT DRIVE")
    print("=" * 80)
    print(f"  KP_HEADING={KP_HEADING}  KD_HEADING={KD_HEADING}")
    print(f"  Holding heading at 0°")
    print()
    print(f"{'ms':>5}  {'gz':>+6}  {'heading':>+8}  "
          f"{'P':>+7}  {'D':>+7}  {'corr':>+7}  "
          f"{'L-pwm':>6} {'R-pwm':>6}")
    print("-" * 80)

    # start motors
    _LF.duty_u16(FWD_BASE)
    _RF.duty_u16(FWD_BASE)

    heading = 0.0    # accumulated angle in degrees
    last_t  = time.ticks_us()
    deadline = time.ticks_ms() + duration_ms
    next_log = time.ticks_ms()
    start    = time.ticks_ms()

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        now = time.ticks_us()
        dt  = time.ticks_diff(now, last_t) / 1_000_000.0
        last_t = now

        gz = read_gyro_z()
        heading += gz * dt   # integrate to get angle

        # PD on heading (P = angle, D = rate)
        # heading +ve = drifted right → slow LEFT to come back → corr +ve
        # heading -ve = drifted left  → slow RIGHT to come back → corr -ve
        p = KP_HEADING * heading
        d = KD_HEADING * gz
        corr = _clamp(int(p + d), -SLOW_AMOUNT, SLOW_AMOUNT)

        # bidirectional: corr +ve = slow LEFT, -ve = slow RIGHT
        left_pwm  = _clamp(FWD_BASE - max(0,  corr))
        right_pwm = _clamp(FWD_BASE - max(0, -corr))

        _LF.duty_u16(left_pwm)
        _RF.duty_u16(right_pwm)

        # log
        if time.ticks_diff(next_log, time.ticks_ms()) <= 0:
            next_log += LOG_MS
            elapsed = time.ticks_diff(time.ticks_ms(), start)
            print(f"{elapsed:>5}  {gz:>+6.1f}  {heading:>+8.2f}  "
                  f"{int(p):>+7}  {int(d):>+7}  {corr:>+7}  "
                  f"{left_pwm:>6} {right_pwm:>6}")

        time.sleep_ms(5)

    _LF.duty_u16(0)
    _RF.duty_u16(0)
    print()
    print(f"Final heading: {heading:+.2f}°")

run()