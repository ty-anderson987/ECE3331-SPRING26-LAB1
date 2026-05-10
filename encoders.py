"""
encoders.py  –  Encoder tick counters
No dependencies except config + machine.
"""

from machine import Pin
from config  import PIN_ENC_L, PIN_ENC_R, LEFT_CM_PER_TICK, RIGHT_CM_PER_TICK

left_ticks  = 0
right_ticks = 0

_enc_left  = Pin(PIN_ENC_L, Pin.IN, Pin.PULL_UP)
_enc_right = Pin(PIN_ENC_R, Pin.IN, Pin.PULL_UP)

def _lcb(p):
    global left_ticks
    left_ticks += 1

def _rcb(p):
    global right_ticks
    right_ticks += 1

_enc_left.irq( trigger=Pin.IRQ_RISING, handler=_lcb)
_enc_right.irq(trigger=Pin.IRQ_RISING, handler=_rcb)

def reset():
    global left_ticks, right_ticks
    left_ticks = right_ticks = 0

def get():
    """Return (left_ticks, right_ticks)."""
    return left_ticks, right_ticks

def left_cm():
    return left_ticks  * LEFT_CM_PER_TICK

def right_cm():
    return right_ticks * RIGHT_CM_PER_TICK