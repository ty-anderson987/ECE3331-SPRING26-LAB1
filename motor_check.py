"""
motor_check.py  –  Live forward run with PID drift correction
                   and INA219 current/voltage/power reporting.

PID Controller
──────────────
  Uses cm-based error (not raw ticks) so different encoder resolutions
  per wheel don't cause false drift readings.

  left_cm  = lt * LEFT_CM_PER_TICK
  right_cm = rt * RIGHT_CM_PER_TICK
  enc_err  = left_cm - right_cm   (+ = left ahead in cm)

  P  =  KP * enc_err
  I  =  KI * integral
  D  =  KD * gyro_z

  All gains in config.py under # PID section.
  Set KP=KI=KD=0 to run on base speeds only.

OC protection
─────────────
  check_both() called every 10ms loop tick.
  Motors stop immediately on trip.
"""

from time   import sleep_ms, ticks_ms, ticks_diff
from config import (FWD_BASE_L, FWD_BASE_R, SLOW_AMOUNT,
                    THRESHOLD, KP, KI, KD, INTEGRAL_CAP,
                    GYRO_DEADBAND, LEFT_CM_PER_TICK, RIGHT_CM_PER_TICK)

def _clamp(v, lo=0, hi=65535):
    return max(lo, min(hi, v))

def _pid(lt, rt, gyro_z, integral):
    """
    Full PID correction using cm-based error.
    Includes conditional integration with sign-flip decay to prevent windup.

    lt, rt   = raw encoder ticks
    gyro_z   = yaw rate °/s  (+ = turning right)
    integral = accumulated error

    Returns (raw_corr, integral, p_term, i_term, d_term, enc_err_cm)
    raw_corr is signed: +ve = slow LEFT, -ve = slow RIGHT
    """
    if abs(gyro_z) < GYRO_DEADBAND:
        gyro_z = 0.0

    left_cm  = lt * LEFT_CM_PER_TICK
    right_cm = rt * RIGHT_CM_PER_TICK
    enc_err  = left_cm - right_cm       # +ve = left ahead

    # sign-flip decay — when err crosses zero, integral did its job
    if (enc_err > 0 and integral < 0) or (enc_err < 0 and integral > 0):
        integral *= 0.5

    # conditional integration — accumulate during moderate err, decay during huge
    if abs(enc_err) < 10.0:
        integral = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, integral + enc_err))
    else:
        integral *= 0.95

    p_term   = KP * enc_err
    i_term   = KI * integral
    d_term   = KD * gyro_z   # NOT negated — gz +ve = going right = slow left = raw +ve

    raw_corr = p_term + i_term + d_term
    raw_corr = _clamp(int(raw_corr), -SLOW_AMOUNT, SLOW_AMOUNT)

    return raw_corr, integral, p_term, i_term, d_term, enc_err

def run(duration_ms=3000, report_ms=200):
    """
    Drives both sides forward for duration_ms.
    Every report_ms prints a telemetry row with full PID breakdown.
    OC checked every 10ms — stops immediately on stall/wall hit.
    Call encoders.reset() mid-run to zero ticks (e.g. on IR_PLAY_PAUSE).
    """
    import encoders
    import oc
    import motors as m

    try:
        from sensors import read_gyro_z
    except:
        read_gyro_z = lambda: 0.0

    integral  = 0
    left_spd  = FWD_BASE_L
    right_spd = FWD_BASE_R

    encoders.reset()
    m.soft_start()
    integral = 0        # reset after ramp
    sleep_ms(200)       # let motors settle
    encoders.reset()    # zero ticks after settle

    deadline    = ticks_ms() + duration_ms
    next_report = ticks_ms() + report_ms

    _gyro_acc   = 0.0
    _gyro_count = 0

    lv, lma, lmw = 0.0, 0.0, 0.0
    rv, rma, rmw = 0.0, 0.0, 0.0

    print("\n=== MOTOR CHECK START ===")
    print(f"{'ms':>6}  {'L-tk':>5}  {'R-tk':>5}  {'Ecm':>6}  "
          f"{'Gyr':>6}  {'P':>6}  {'I':>6}  {'D':>6}  {'Corr':>5}  "
          f"{'L-mA':>6}  {'R-mA':>6}  "
          f"{'L-V':>5}  {'R-V':>5}  "
          f"{'L-mW':>6}  {'R-mW':>6}  OC")

    while ticks_diff(deadline, ticks_ms()) > 0:

        # OC every tick
        oc.check_both()
        if oc.halted():
            m.stop()
            sleep_ms(300)
            print("[OC] waiting for current to settle...")
            _settle = ticks_ms() + 1500
            while ticks_diff(_settle, ticks_ms()) > 0:
                (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()
                if abs(lma) < 350 and abs(rma) < 350:
                    break
                sleep_ms(50)
            oc.reset_state()
            print("[OC] backing up")
            m.oc_backward()
            lt, rt = encoders.get()
            print(f"=== MOTOR CHECK DONE ===  L={lt}  R={rt}  drift={lt - rt:+}")
            return

        # gyro accumulate every tick
        try:
            _gyro_acc   += read_gyro_z()
            _gyro_count += 1
        except:
            pass

        # report + correction every report_ms
        if ticks_diff(next_report, ticks_ms()) <= 0:
            next_report += report_ms

            lt, rt = encoders.get()
            gyro_z = (_gyro_acc / _gyro_count) if _gyro_count > 0 else 0.0
            _gyro_acc   = 0.0
            _gyro_count = 0

            # INA219 read every report interval
            (lv, lma, lmw), (rv, rma, rmw) = oc.read_both()

            raw_corr, integral, p, i, d, enc_err_cm = _pid(lt, rt, gyro_z, integral)

            # bidirectional: raw_corr +ve = slow left, -ve = slow right
            left_spd  = _clamp(FWD_BASE_L - max(0,  raw_corr))
            right_spd = _clamp(FWD_BASE_R - max(0, -raw_corr))

            if not oc.state['left']['tripped']:
                m.set_left_fwd(left_spd)
            if not oc.state['right']['tripped']:
                m.set_right_fwd(right_spd)

            elapsed = duration_ms - ticks_diff(deadline, ticks_ms())
            oc_str  = f"{'T' if oc.state['left']['tripped'] else 'ok'}/" \
                      f"{'T' if oc.state['right']['tripped'] else 'ok'}"

            print(
                f"{elapsed:>6}  {lt:>5}  {rt:>5}  {enc_err_cm:>+6.2f}  "
                f"{gyro_z:>+6.2f}  {int(p):>+6}  {int(i):>+6}  {int(d):>+6}  {raw_corr:>+6}  "
                f"{lma:>6.0f}  {rma:>6.0f}  "
                f"{lv:>5.2f}  {rv:>5.2f}  "
                f"{lmw:>6.0f}  {rmw:>6.0f}  {oc_str}"
            )

        sleep_ms(10)

    m.stop()
    lt, rt = encoders.get()
    left_cm  = lt * LEFT_CM_PER_TICK
    right_cm = rt * RIGHT_CM_PER_TICK
    print(f"\n=== MOTOR CHECK DONE ===  L={lt}tk({left_cm:.1f}cm)  R={rt}tk({right_cm:.1f}cm)  drift={left_cm - right_cm:+.1f}cm")