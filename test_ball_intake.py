"""
test_ball_intake.py  –  Ball intake test
─────────────────────────────────────────
servo1 (pin 8) spins continuously to intake balls.
After init, servo2 is re-asserted to center so the shared
PWM slice doesn't cause it to twitch.

  GREEN / BLUE  → keep spinning, beep (score)
  RED           → reverse servo until ball clears, resume
  NONE          → keep spinning
"""

from machine import PWM, Pin
from config  import PIN_SERVO1, SERVO1_SPIN, SERVO1_STOP
import sensors
import color
import buzzer
import time

# ── Servo1 setup ──────────────────────────────────────────────
_servo1 = PWM(Pin(PIN_SERVO1))
_servo1.freq(50)

# re-assert servo2 center after creating servo1 on the shared PWM slice
sensors.servo2_hold_center()

def _s1_duty(pulse_us):
    _servo1.duty_u16(int(pulse_us / 20000 * 65535))

SERVO1_REVERSE = SERVO1_STOP + (SERVO1_STOP - SERVO1_SPIN)

def s1_forward():
    _s1_duty(SERVO1_SPIN)
    sensors.servo2_hold_center()

def s1_reverse():
    _s1_duty(SERVO1_REVERSE)
    sensors.servo2_hold_center()

def s1_stop():
    _s1_duty(SERVO1_STOP)
    sensors.servo2_hold_center()

# ── Fast ball read — 5ms per channel ─────────────────────────
def _fast_ball():
    r = color._freq(color.OUT1, 0, 0, ms=5)
    g = color._freq(color.OUT1, 1, 1, ms=5)
    b = color._freq(color.OUT1, 0, 1, ms=5)
    total = r + g + b
    if total == 0:
        return "NONE"
    rn = r / total
    gn = g / total
    bn = b / total
    if gn > 0.40:
        return "GREEN"
    elif bn > 0.40 and rn < 0.25:
        return "BLUE"
    elif rn > 0.50 and gn < 0.23:
        return "RED"
    return "NONE"

EJECT_MS    = 600
COOLDOWN_MS = 800

def run():
    print("BALL INTAKE — servo1 spinning")
    s1_forward()
    last_event_t = 0

    try:
        while True:
            now = time.ticks_ms()
            if time.ticks_diff(now, last_event_t) < COOLDOWN_MS:
                continue

            ball = _fast_ball()

            if ball in ("GREEN", "BLUE"):
                print(f"SCORE {ball}")
                buzzer.beep(n=2, ms=200, gap=80)
                last_event_t = time.ticks_ms()

            elif ball == "RED":
                print("RED eject")
                s1_reverse()
                deadline = time.ticks_ms() + EJECT_MS
                while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                    if _fast_ball() == "NONE":
                        break
                s1_forward()
                last_event_t = time.ticks_ms()

    except KeyboardInterrupt:
        s1_stop()
        buzzer.off()
        print("stopped")

run()