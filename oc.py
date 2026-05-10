"""
oc.py  –  INA219 overcurrent protection
No import of motors. Stop callbacks injected via init().
Both wheels share one retry counter — either wheel tripping increments it.
"""

from config import ADDR_LEFT, ADDR_RIGHT, OVERCURRENT_MA, WARN_MA, OVERCURRENT_TURN_MA, WARN_TURN_MA
import buzzer
import time

_i2c        = None
_stop_left  = None
_stop_right = None

# active thresholds — call set_mode() to switch
_oc_ma   = OVERCURRENT_MA
_warn_ma = WARN_MA

# startup grace — motors spike on start, ignore strict OC for this window
_grace_until = 0
GRACE_MS     = 4000   # 4 second grace — covers two back-to-back turns
GRACE_OC_MA  = 900    # permissive threshold during grace

def set_mode(mode='straight'):
    """'straight' or 'turn' — sets active OC thresholds."""
    global _oc_ma, _warn_ma
    if mode == 'turn':
        _oc_ma   = OVERCURRENT_TURN_MA
        _warn_ma = WARN_TURN_MA
    else:
        _oc_ma   = OVERCURRENT_MA
        _warn_ma = WARN_MA

def start_grace():
    """Call when motors first start to suppress startup current spike."""
    global _grace_until
    _grace_until = time.ticks_ms() + GRACE_MS

def _active_oc_ma():
    if time.ticks_diff(time.ticks_ms(), _grace_until) < 0:
        return GRACE_OC_MA
    return _oc_ma

def _active_warn_ma():
    if time.ticks_diff(time.ticks_ms(), _grace_until) < 0:
        return GRACE_OC_MA
    return _warn_ma

state = {
    'left':  {'trips': 0, 'clear': 0, 'tripped': False},
    'right': {'trips': 0, 'clear': 0, 'tripped': False},
}

# shared retry counter — both wheels draw from the same 3 attempts
_retries = 0
_MAX_RETRIES = 3

def init(i2c, stop_left_fn, stop_right_fn):
    global _i2c, _stop_left, _stop_right
    _i2c        = i2c
    _stop_left  = stop_left_fn
    _stop_right = stop_right_fn
    CAL = 4096
    for addr in [ADDR_LEFT, ADDR_RIGHT]:
        _i2c.writeto_mem(addr, 0x00, bytes([0x39, 0x9F]))
        _i2c.writeto_mem(addr, 0x05, bytes([CAL >> 8, CAL & 0xFF]))

def read(addr):
    try:
        bus_raw    = _i2c.readfrom_mem(addr, 0x02, 2)
        voltage    = ((bus_raw[0] << 8 | bus_raw[1]) >> 3) * 0.004
        shunt      = _i2c.readfrom_mem(addr, 0x01, 2)
        shunt_raw  = shunt[0] << 8 | shunt[1]
        if shunt_raw > 32767:
            shunt_raw -= 65536
        current_ma = -(shunt_raw * 0.1)
        power      = _i2c.readfrom_mem(addr, 0x03, 2)
        power_mw   = (power[0] << 8 | power[1]) * 2.0
        return voltage, current_ma, power_mw
    except:
        return 0.0, 0.0, 0.0

def read_both():
    return read(ADDR_LEFT), read(ADDR_RIGHT)

def check(wheel, current_ma):
    global _retries
    s = state[wheel]

    oc_limit   = _active_oc_ma()
    warn_limit = _active_warn_ma()

    if abs(current_ma) > oc_limit:
        s['trips'] += 1
        s['clear']  = 0
        if s['trips'] >= 1:
            s['tripped'] = True
            _stop_left()
            _stop_right()
            buzzer.on()
            print(f"[OC TRIP] {wheel} {current_ma:.0f}mA")
    elif abs(current_ma) > warn_limit:
        print(f"[OC WARN] {wheel} {current_ma:.0f}mA")
        s['trips'] = 0
    else:
        s['trips'] = 0
        if s['tripped']:
            s['clear'] += 1
            if s['clear'] >= 5:
                _retries += 1
                if _retries <= _MAX_RETRIES:
                    s['tripped'] = False
                    s['clear']   = 0
                    buzzer.off()
                    print(f"[OC CLEAR] {wheel}  retry {_retries}/{_MAX_RETRIES}")
                    time.sleep_ms(500)
                else:
                    buzzer.off()
                    print(f"[OC GIVING UP] {wheel} — max retries reached")

def check_both():
    (lv, lma, lmw), (rv, rma, rmw) = read_both()
    check('left',  lma)
    check('right', rma)
    return (lv, lma, lmw), (rv, rma, rmw)

def halted():
    """True when shared retry limit exceeded."""
    return _retries > _MAX_RETRIES

def reset_state():
    global _retries
    for s in state.values():
        s['trips']   = 0
        s['clear']   = 0
        s['tripped'] = False
    _retries = 0
    buzzer.off()