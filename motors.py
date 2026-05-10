"""
motors.py  –  Main motor module.
All tunable values come from config.py — nothing hardcoded here.

Boot order:
    import motors          ← sets up PWM, encoders, inits OC with callbacks
    motors.run_check()     ← optional live drift/PID check
    motors.forward(2000)   ← drive helpers
"""

from machine import PWM, Pin, I2C
from time    import sleep_ms, ticks_ms, ticks_diff
from config  import (PIN_LF, PIN_LR, PIN_RF, PIN_RR,
                     PWM_FREQ, FWD_BASE_L, FWD_BASE_R,
                     REV_BASE_L, REV_BASE_R, TURN_SPEED)
import encoders
import oc
import motor_check as _mc

# ── I2C ───────────────────────────────────────────────────────────────────────
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)

# ── PWM outputs ───────────────────────────────────────────────────────────────
_LF = PWM(Pin(PIN_LF)); _LF.freq(PWM_FREQ); _LF.duty_u16(0)
_LR = PWM(Pin(PIN_LR)); _LR.freq(PWM_FREQ); _LR.duty_u16(0)
_RF = PWM(Pin(PIN_RF)); _RF.freq(PWM_FREQ); _RF.duty_u16(0)
_RR = PWM(Pin(PIN_RR)); _RR.freq(PWM_FREQ); _RR.duty_u16(0)

# ── OC callbacks ─────────────────────────────────────────────────────────────
def stop_left():
    _LF.duty_u16(0); _LR.duty_u16(0)

def stop_right():
    _RF.duty_u16(0); _RR.duty_u16(0)

def stop():
    stop_left(); stop_right()

oc.init(i2c, stop_left, stop_right)

# ── Helpers ───────────────────────────────────────────────────────────────────
def _clamp(v, lo=0, hi=65535):
    return max(lo, min(hi, v))

def set_left_fwd(speed):
    _LR.duty_u16(0);  _LF.duty_u16(_clamp(speed))

def set_right_fwd(speed):
    _RR.duty_u16(0);  _RF.duty_u16(_clamp(speed))

def set_left_rev(speed):
    _LF.duty_u16(0);  _LR.duty_u16(_clamp(speed))

def set_right_rev(speed):
    _RF.duty_u16(0);  _RR.duty_u16(_clamp(speed))

# ── Telemetry loop ────────────────────────────────────────────────────────────
def _telem_loop(duration_ms, label='FWD', report_ms=200, oc_check=True):
    try:
        from sensors import read_gyro_z
    except:
        read_gyro_z = lambda: 0.0

    deadline    = ticks_ms() + duration_ms
    next_report = ticks_ms() + report_ms

    while ticks_diff(deadline, ticks_ms()) > 0:

        if oc_check:
            oc.check_both()
            if oc.halted():
                stop()
                return

        sleep_ms(10)

# ── Soft start ────────────────────────────────────────────────────────────────
def soft_start(ramp_steps=10, step_ms=30):
    encoders.reset()
    oc.start_grace()   # suppress startup current spike for 2s
    for i in range(1, ramp_steps + 1):
        spd_l = int(20000 + (FWD_BASE_L - 20000) * i / ramp_steps)
        spd_r = int(20000 + (FWD_BASE_R - 20000) * i / ramp_steps)
        set_left_fwd(spd_l)
        set_right_fwd(spd_r)
        sleep_ms(step_ms)
        oc.check_both()
        if oc.halted():
            stop()
            print("[SOFT START] OC halt")
            return

# ── Drive helpers ─────────────────────────────────────────────────────────────
# Gyro-only heading hold — KP/KD operate on integrated yaw angle (degrees)
HEADING_KP = 1500
HEADING_KD = 100
HEADING_SLOW_AMOUNT = 30000
HEADING_GYRO_DEADBAND = 3.0   # °/s — ignore below this, filters out bump vibration

def _gyro_drive(ms, base_l, base_r, set_l, set_r, label, reverse=False):
    try:
        from sensors import read_gyro_z
    except:
        read_gyro_z = lambda: 0.0
    from time import ticks_us, ticks_diff as _tdiff

    encoders.reset()
    set_l(base_l)
    set_r(base_r)

    heading  = 0.0
    last_t   = ticks_us()
    deadline = ticks_ms() + ms

    while ticks_diff(deadline, ticks_ms()) > 0:
        oc.check_both()
        if oc.halted():
            stop()
            print("[OC] fwd halt")
            return

        now = ticks_us()
        dt  = _tdiff(now, last_t) / 1_000_000.0
        last_t = now

        try:
            gz = read_gyro_z()
        except:
            gz = 0.0

        gz_filtered = gz if abs(gz) >= HEADING_GYRO_DEADBAND else 0.0
        heading += gz_filtered * dt

        p = HEADING_KP * heading
        d = HEADING_KD * gz
        corr = max(-HEADING_SLOW_AMOUNT, min(HEADING_SLOW_AMOUNT, int(p + d)))

        if reverse:
            corr = -corr

        set_l(_clamp(base_l - max(0,  corr)))
        set_r(_clamp(base_r - max(0, -corr)))

        sleep_ms(5)

    stop()

def forward(ms):
    """Forward with gyro heading hold (no encoder PID)."""
    soft_start()
    sleep_ms(100)
    _gyro_drive(ms, FWD_BASE_L, FWD_BASE_R,
                set_left_fwd, set_right_fwd, "FORWARD")

def backward(ms):
    """Reverse with gyro heading hold (no encoder PID)."""
    _gyro_drive(ms, REV_BASE_L, REV_BASE_R,
                set_left_rev, set_right_rev, "BACKWARD", reverse=True)

def oc_backward(ms=3000):
    """Backward escape after OC trip. Skips OC first 500ms for chassis tilt spike."""
    print("=== OC ESCAPE ===")
    encoders.reset()
    set_left_rev(REV_BASE_L)
    set_right_rev(REV_BASE_R)
    sleep_ms(500)
    _telem_loop(ms - 500, 'ESC', oc_check=True)
    stop()
    lt, rt = encoders.get()
    print(f"=== OC ESCAPE DONE ===  L={lt}  R={rt}")

def turn_left(ms):
    print("=== TURN LEFT ===")
    encoders.reset()
    set_left_rev(TURN_SPEED)
    set_right_fwd(TURN_SPEED)
    _telem_loop(ms, 'L')
    stop()
    print("=== TURN LEFT DONE ===")

def turn_right(ms):
    print("=== TURN RIGHT ===")
    encoders.reset()
    set_left_fwd(TURN_SPEED)
    set_right_rev(TURN_SPEED)
    _telem_loop(ms, 'R')
    stop()
    print("=== TURN RIGHT DONE ===")

def turn_degrees(degrees, direction='right', timeout_ms=8000):
    """
    Pivot turn — opposite wheels spin in opposite directions.
    Uses abs(gz) accumulation so gyro sign doesn't matter.
    """
    try:
        from sensors import read_gyro_z
    except:
        print("[TURN] sensors.py not found")
        if direction == 'right':
            turn_right(1000)
        else:
            turn_left(1000)
        return

    FAST_SPEED = 55000
    SLOW_SPEED = 35000
    RAMP_START = 35000
    SLOW_AT    = 0.55

    encoders.reset()
    oc.reset_state()
    oc.set_mode('turn')
    oc.start_grace()

    def _set_spd(spd):
        if direction == 'right':
            set_left_fwd(spd); set_right_rev(spd)
        else:
            set_left_rev(spd); set_right_fwd(spd)

    # ramp up to full speed
    for i in range(1, 21):
        _set_spd(int(RAMP_START + (FAST_SPEED - RAMP_START) * i / 20))
        sleep_ms(15)

    accumulated = 0.0
    deadline    = ticks_ms() + timeout_ms
    last_t      = ticks_ms()
    slowed      = False

    while ticks_diff(deadline, ticks_ms()) > 0:
        now    = ticks_ms()
        dt     = ticks_diff(now, last_t) / 1000.0
        last_t = now

        try:
            gz = read_gyro_z()
        except OSError:
            gz = 0.0
        accumulated += abs(gz) * dt

        if accumulated >= degrees:
            break

        # past halfway — drop to slow speed so momentum doesn't carry past target
        if not slowed and accumulated >= degrees * SLOW_AT:
            _set_spd(SLOW_SPEED)
            slowed = True

        oc.check_both()
        if oc.halted():
            stop()
            oc.set_mode('straight')
            return accumulated

        sleep_ms(10)

    stop()
    oc.set_mode('straight')
    lt, rt = encoders.get()
    return accumulated

# ── Standalone status snapshot ────────────────────────────────────────────────
def print_status():
    lt, rt = encoders.get()
    (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()

    try:
        from sensors import read_gyro_z
        imu_str = f"{read_gyro_z():+.2f} deg/s"
    except:
        imu_str = "n/a"

    print("---- MOTOR STATUS ----")
    print(f"  ENC  L: {lt:>5} ticks  {encoders.left_cm():>6.1f} cm")
    print(f"       R: {rt:>5} ticks  {encoders.right_cm():>6.1f} cm")
    print(f"  INA  L: {lma:>7.1f} mA  {lv:.2f} V  {lmw:>7.1f} mW")
    print(f"       R: {rma:>7.1f} mA  {rv:.2f} V  {rmw:>7.1f} mW")
    print(f"  IMU  yaw: {imu_str}")
    print(f"  OC   retries: {oc._retries}/{oc._MAX_RETRIES}")
    print("----------------------")

# ── Motor check ───────────────────────────────────────────────────────────────
def run_check(duration_ms=3000, report_ms=200):
    """Full PID forward run with telemetry. All gains from config.py # PID section."""
    _mc.run(duration_ms=duration_ms, report_ms=report_ms)