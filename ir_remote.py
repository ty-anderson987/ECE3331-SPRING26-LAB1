from machine import Pin
from config import *
import time

ir_pin    = Pin(PIN_IR_RX, Pin.IN)
_last_t   = 0
_callback = None

def set_callback(fn):
    global _callback
    _callback = fn

def _read_pulse():
    start = time.ticks_us()
    while ir_pin.value() == 1:
        if time.ticks_diff(time.ticks_us(), start) > 100000:
            return None
    pulses = []
    for _ in range(67):
        start = time.ticks_us()
        while ir_pin.value() == 0:
            if time.ticks_diff(time.ticks_us(), start) > 10000:
                return None
        low = time.ticks_diff(time.ticks_us(), start)
        start = time.ticks_us()
        while ir_pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start) > 10000:
                break
        high = time.ticks_diff(time.ticks_us(), start)
        pulses.append((low, high))
    return pulses

def _decode(pulses):
    if not pulses:
        return None
    bits = []
    for low, high in pulses[1:33]:
        bits.append(1 if high > 1000 else 0)
    if len(bits) < 32:
        return None
    value = 0
    for bit in bits:
        value = (value << 1) | bit
    return value

def check():
    global _last_t
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_t) < 500:
        return
    pulses = _read_pulse()
    if not pulses:
        return
    code = _decode(pulses)
    if not code:
        return
    _last_t = now
    if _callback:
        _callback(code)