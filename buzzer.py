from machine import Pin
from config import PIN_BUZZER
import time

_buz = Pin(PIN_BUZZER, Pin.OUT)

def on():
    _buz.value(1)

def off():
    _buz.value(0)

def beep(n=1, ms=100, gap=100):
    for _ in range(n):
        _buz.value(1)
        time.sleep_ms(ms)
        _buz.value(0)
        time.sleep_ms(gap)

def command():
    _buz.value(1)
    time.sleep_ms(50)
    _buz.value(0)

def warning(current_ma):
    if abs(current_ma) >= 550:
        _buz.value(1)
        return
    if abs(current_ma) >= 400:
        ratio    = (abs(current_ma) - 400) / 150
        delay_ms = int(500 - ratio * 450)
        _buz.value(not _buz.value())
        time.sleep_ms(delay_ms)
    else:
        _buz.value(0)