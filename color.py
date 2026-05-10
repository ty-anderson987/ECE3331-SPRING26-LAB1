from machine import Pin
from config import *
import time

# ======================
# PIN SETUP
# ======================
S0   = Pin(PIN_S0, Pin.OUT)
S1   = Pin(PIN_S1, Pin.OUT)
S2   = Pin(PIN_S2, Pin.OUT)
S3   = Pin(PIN_S3, Pin.OUT)

OUT1 = Pin(PIN_OUT1, Pin.IN)  # ball sensor
OUT2 = Pin(PIN_OUT2, Pin.IN)  # floor sensor

# frequency scaling (20%)
S0.value(1)
S1.value(0)


# ======================
# CORE FREQUENCY READER
# ======================
def _freq(out_pin, s2v, s3v, ms=20):
    S2.value(s2v)
    S3.value(s3v)

    time.sleep_ms(2)

    count = 0
    start = time.ticks_ms()

    last = 1

    while time.ticks_diff(time.ticks_ms(), start) < ms:
        v = out_pin.value()

        # count falling edges only
        if last == 1 and v == 0:
            count += 1

        last = v

    return count


# ======================
# BALL COLOR DETECTION
# ======================
last_ball = "NONE"
last_seen_time = 0
LOST_TIMEOUT_MS = 200    # how long to hold last classification before going to NONE

def ball():
    """
    Ball color classifier — recalibrated readings:
      NO_BALL: total=23-25  rn=0.43 gn=0.29 bn=0.27  (dim floor reflection)
      RED:     total=45-49  rn=0.55 gn=0.20 bn=0.24  (bright, high red)
      GREEN:   total=36-38  rn=0.24 gn=0.47 bn=0.28  (high green)
      BLUE:    total=20-21  rn=0.20 gn=0.30 bn=0.48  (high blue, dim)

    Strategy:
      1. total < 30 → too dim to be a ball (floor reflection) → NONE
      2. GREEN: gn > 0.42
      3. BLUE:  bn > 0.43 and rn < 0.27
      4. RED:   total > 35 and rn > 0.50 and gn < 0.23
         (total gate separates red ball from floor which has similar ratios)
    """
    global last_ball, last_seen_time

    r = _freq(OUT1, 0, 0)
    g = _freq(OUT1, 1, 1)
    b = _freq(OUT1, 0, 1)

    total = r + g + b
    if total == 0:
        return last_ball

    now = time.ticks_ms()
    classified = None

    # too dim — floor reflection, no ball present
    if total < 30:
        if time.ticks_diff(now, last_seen_time) > LOST_TIMEOUT_MS:
            last_ball = "NONE"
        return last_ball

    rn = r / total
    gn = g / total
    bn = b / total

    # GREEN — strong green channel
    if gn > 0.42:
        classified = "GREEN"

    # BLUE — strong blue, low red
    elif bn > 0.43 and rn < 0.27:
        classified = "BLUE"

    # RED — bright total AND high red ratio (floor is dim so total gate blocks it)
    elif total > 35 and rn > 0.50 and gn < 0.23:
        classified = "RED"

    if classified is not None:
        last_ball = classified
        last_seen_time = now
    else:
        if time.ticks_diff(now, last_seen_time) > LOST_TIMEOUT_MS:
            last_ball = "NONE"

    return last_ball

# ======================
# FLOOR COLOR DETECTION
# ======================
def floor():
    """
    Floor color classifier based on calibration data:
      GREY:   R=180 G=174 B=182  total=537   rn=0.33 gn=0.33 bn=0.34
      BLACK:  R=59  G=64  B=71   total=195   rn=0.30 gn=0.33 bn=0.37
      BLUE:   R=133 G=240 B=406  total=779   rn=0.17 gn=0.31 bn=0.52
      YELLOW: R=656 G=404 B=279  total=1339  rn=0.49 gn=0.30 bn=0.21
      PURPLE: R=516 G=526 B=697  total=1739  rn=0.30 gn=0.30 bn=0.40
    """
    r = _freq(OUT2, 0, 0)
    g = _freq(OUT2, 1, 1)
    b = _freq(OUT2, 0, 1)

    total = r + g + b
    if total == 0:
        return "UNKNOWN"

    rn = r / total
    gn = g / total
    bn = b / total

    # BLACK — much darker than any other surface.
    # Calibration: grey total~44, black total~16. Threshold 30 splits them.
    if total < 30:
        return "BLACK"

    # YELLOW — strong red, low blue
    if rn > 0.40 and bn < 0.27:
        return "YELLOW"

    # BLUE — very low red, very high blue
    if rn < 0.22 and bn > 0.45:
        return "BLUE"

    # PURPLE — high blue but bright (distinguishes from BLUE which is darker)
    if bn > 0.36 and total > 1200:
        return "PURPLE"

    # GREY — balanced ratios, mid brightness
    if abs(rn - 0.33) < 0.04 and abs(gn - 0.33) < 0.04 and abs(bn - 0.34) < 0.04:
        return "GREY"

    return "UNKNOWN"


# ======================
# OPTIONAL: QUICK UPDATE WRAPPER
# (use this in main loop if you want both at once)
# ======================
ball_color = "NONE"
floor_color = "UNKNOWN"
last_update = 0


def update():
    """
    Call this in main loop.
    Non-blocking style (spread cost over time if needed later).
    """

    global ball_color, floor_color, last_update

    now = time.ticks_ms()

    # limit full scan rate (prevents loop slowdown)
    if time.ticks_diff(now, last_update) < 120:
        return

    ball_color = ball()
    floor_color = floor()

    last_update = now