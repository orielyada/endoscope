import time
import supervisor


PCF_ADDR = 0x20
ADDR_NAU7802 = 0x2A
REG_ADCO_B2 = 0x12
ADDR_LSM6DSOX = 0x6A
ADDR_PCF8574 = 0x20
ADDR_BNO085 = 0x4A

LOAD_SHIFT = 5              #>> 5 means downshift raw by 32 to fit int16 range with some headroom for overscale.
LOAD_ALPHA_SHIFT = 0        #>> 0 means no IIR smoothing, just downshift raw to fit int16 range.
PCF_DEBOUNCE_SAMPLES = 3    # Number of consecutive stable samples required to update debounced PCF state.

ALLOWED_TORSO = {ADDR_NAU7802, ADDR_LSM6DSOX}
ALLOWED_HANDSET = {ADDR_PCF8574, ADDR_BNO085}


class MaskDebounce:
    """Edge-immediate debouncer with hold window to suppress chatter."""

    def __init__(self, hold_ms=10, initial=False):
        self.hold_ns = int(hold_ms * 1_000_000)
        self.state = bool(initial)
        self._lock_until_ns = 0

    def update(self, raw_pressed: bool, now_ns: int) -> bool:
        """Apply hold-time debouncing and return stable pressed state."""
        if now_ns < self._lock_until_ns:
            return self.state

        if bool(raw_pressed) != self.state:
            self.state = bool(raw_pressed)
            self._lock_until_ns = now_ns + self.hold_ns

        return self.state


_pins = None
_i2c_torso = None
_i2c_handset = None
_nau = None
_nau_ok_runtime = False
_bno = None
_bno_ok_runtime = False

_load16 = 0
_load_filt = 0
_load_filt_inited = False

_pcf_last_raw = 0xFF
_pcf_stable = 0xFF
_pcf_same_count = 0

_pcf_ok = False
_pcf_io_ok = False
_bno_ok = False
_lsm6_ok = False
_nau_ok = False
_last_i2c_scan = 0.0

_db_torso_l = MaskDebounce(hold_ms=10, initial=False)
_db_torso_r = MaskDebounce(hold_ms=10, initial=False)
_db_pedal = MaskDebounce(hold_ms=10, initial=False)
_db_pedsense = MaskDebounce(hold_ms=10, initial=False)

_BNO_MAX_HZ = 250
_BNO_PERIOD_NS = int(1_000_000_000 // _BNO_MAX_HZ)
_bno_last_sample_ns = 0
# Identity quaternion mapped from [-1, 1] to int16 in w, x, y, z order.
_quat_i16 = (32767, 0, 0, 0)

# LSM6 fused roll state
_lsm_roll_rad = 0.0
_lsm_gyro_angle_rad = 0.0
_lsm_last_sample_ns = 0
_lsm_roll16 = 0
_lsm6 = None
_lsm6_ok_runtime = False

# 1D Kalman filter state for X-Z angle (roll) fused with gyro Y-rate.
_kf_angle = 0.0
_kf_bias = 0.0
_kf_P00 = 1.0
_kf_P01 = 0.0
_kf_P10 = 0.0
_kf_P11 = 1.0
_KF_Q_ANGLE = 0.02
_KF_Q_BIAS = 0.002
_KF_R_MEASURE = 0.08


def _i2c_try_lock_with_timeout(i2c, timeout_s=0.05) -> bool:
    """Attempt to lock I2C bus for up to timeout_s seconds."""
    if i2c is None:
        return False
    t0 = time.monotonic()
    while not i2c.try_lock():
        if time.monotonic() - t0 > timeout_s:
            return False
    return True


def setup_readout(pins, i2c_torso, i2c_handset, nau, nau_ok_runtime, bno=None, bno_ok_runtime=False, lsm6=None, lsm6_ok_runtime=False):
    """Inject hardware handles/state so readout functions stay module-local."""
    global _pins, _i2c_torso, _i2c_handset, _nau, _nau_ok_runtime, _bno, _bno_ok_runtime, _lsm6, _lsm6_ok_runtime
    _pins = pins
    _i2c_torso = i2c_torso
    _i2c_handset = i2c_handset
    _nau = nau
    _nau_ok_runtime = bool(nau_ok_runtime)
    _bno = bno
    _bno_ok_runtime = bool(bno_ok_runtime)
    _lsm6 = lsm6
    _lsm6_ok_runtime = bool(lsm6_ok_runtime)

    # LSM6 fused roll state
    global _lsm_roll_rad, _lsm_gyro_angle_rad, _lsm_roll16, _lsm_last_sample_ns
    global _kf_angle, _kf_bias, _kf_P00, _kf_P01, _kf_P10, _kf_P11
    _lsm_roll_rad = 0.0
    _lsm_gyro_angle_rad = 0.0
    _lsm_roll16 = 0
    _lsm_last_sample_ns = time.monotonic_ns()
    _kf_angle = 0.0
    _kf_bias = 0.0
    _kf_P00 = 1.0
    _kf_P01 = 0.0
    _kf_P10 = 0.0
    _kf_P11 = 1.0


def _kalman_update_1d(angle_meas_rad: float, gyro_rate_rad_s: float, dt_s: float) -> float:
    """1D Kalman update for angle+bias model."""
    global _kf_angle, _kf_bias, _kf_P00, _kf_P01, _kf_P10, _kf_P11

    # Predict
    rate = gyro_rate_rad_s - _kf_bias
    _kf_angle += dt_s * rate

    _kf_P00 += dt_s * (dt_s * _kf_P11 - _kf_P01 - _kf_P10 + _KF_Q_ANGLE)
    _kf_P01 -= dt_s * _kf_P11
    _kf_P10 -= dt_s * _kf_P11
    _kf_P11 += _KF_Q_BIAS * dt_s

    # Update
    innovation = angle_meas_rad - _kf_angle
    S = _kf_P00 + _KF_R_MEASURE
    if S <= 0.0:
        return _kf_angle
    K0 = _kf_P00 / S
    K1 = _kf_P10 / S

    _kf_angle += K0 * innovation
    _kf_bias += K1 * innovation

    P00 = _kf_P00
    P01 = _kf_P01
    _kf_P00 -= K0 * P00
    _kf_P01 -= K0 * P01
    _kf_P10 -= K1 * P00
    _kf_P11 -= K1 * P01

    return _kf_angle


def _wrap_pi(a: float) -> float:
    """Wrap radians to [-pi, pi]."""
    import math
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _quat_f32_to_i16(v: float) -> int:
    """Map quaternion float from [-1.0, 1.0] to int16 [-32768, 32767]."""
    x = float(v)
    if x < -1.0:
        x = -1.0
    elif x > 1.0:
        x = 1.0
    if x <= -1.0:
        return -32768
    if x >= 1.0:
        return 32767
    return int(round(x * 32767.0))


def update_bno_quat_i16(now_ns: int) -> tuple[int, int, int, int]:
    """
    Return quaternion as int16 tuple (w, x, y, z).
    Sampling is rate-limited to 250 Hz max; faster callers receive cached values.
    """
    global _quat_i16, _bno_last_sample_ns

    if (now_ns - _bno_last_sample_ns) < _BNO_PERIOD_NS:
        return _quat_i16
    _bno_last_sample_ns = now_ns

    if not _bno_ok_runtime or _bno is None:
        return _quat_i16

    try:
        # Direct read like Adafruit's example - simple and straightforward
        q = _bno.quaternion
        
        if not q or len(q) < 4:
            return _quat_i16

        # Adafruit BNO08X order is (x, y, z, w); HID stream uses (w, x, y, z).
        qx_f, qy_f, qz_f, qw_f = q[0], q[1], q[2], q[3]
        
        _quat_i16 = (
            _quat_f32_to_i16(qw_f),
            _quat_f32_to_i16(qx_f),
            _quat_f32_to_i16(qy_f),
            _quat_f32_to_i16(qz_f),
        )
    except Exception:
        # Keep last good quaternion on transient bus/sensor errors.
        pass

    return _quat_i16


def pcf_init():
    """Set PCF8574 lines high so they behave as inputs (quasi-bidirectional)."""
    global _pcf_io_ok
    if not _i2c_try_lock_with_timeout(_i2c_handset):
        _pcf_io_ok = False
        return
    try:
        _i2c_handset.writeto(PCF_ADDR, bytes([0xFF]))
        _pcf_io_ok = True
    except Exception:
        _pcf_io_ok = False
    finally:
        _i2c_handset.unlock()


def pcf_read_raw() -> int:
    """Read raw PCF8574 byte where 1=high and 0=low."""
    global _pcf_io_ok
    buf = bytearray(1)
    if not _i2c_try_lock_with_timeout(_i2c_handset):
        _pcf_io_ok = False
        return _pcf_stable
    try:
        _i2c_handset.readfrom_into(PCF_ADDR, buf)
        _pcf_io_ok = True
    except Exception:
        _pcf_io_ok = False
        return _pcf_stable
    finally:
        _i2c_handset.unlock()
    return buf[0]


def pcf_read_debounced() -> int:
    """Return debounced raw PCF8574 byte using N-sample stability."""
    global _pcf_last_raw, _pcf_stable, _pcf_same_count

    raw = pcf_read_raw()

    if raw == _pcf_last_raw:
        if _pcf_same_count < 255:
            _pcf_same_count += 1
    else:
        _pcf_last_raw = raw
        _pcf_same_count = 0

    if _pcf_same_count >= (PCF_DEBOUNCE_SAMPLES - 1):
        _pcf_stable = _pcf_last_raw

    return _pcf_stable


def _read_local_inputs(now_ns: int):
    """Read and debounce torso/pedal GPIO lines (active-low wiring)."""
    raw_torso_l_pressed = (_pins["torso_l"].value is False)
    raw_torso_r_pressed = (_pins["torso_r"].value is False)
    raw_pedal_pressed = (_pins["pedal"].value is False)
    raw_pedal_connected = (_pins["pedal_sense"].value is False)

    torso_l_pressed = _db_torso_l.update(raw_torso_l_pressed, now_ns)
    torso_r_pressed = _db_torso_r.update(raw_torso_r_pressed, now_ns)
    pedal_pressed = _db_pedal.update(raw_pedal_pressed, now_ns)
    pedal_connected = _db_pedsense.update(raw_pedal_connected, now_ns)

    return torso_l_pressed, torso_r_pressed, pedal_pressed, pedal_connected


def sample_buttons(now_ns: int) -> tuple[int, bool]:
    """Build HID buttons bitfield and return pedal-connected state."""
    torso_l_pressed, torso_r_pressed, pedal_pressed, pedal_connected = _read_local_inputs(now_ns)

    # PCF bits are active-low, so invert to pressed=1.
    pcf_raw = pcf_read_debounced()
    pcf_pressed = (~pcf_raw) & 0xFF

    # Map: PCF P0..P7 -> bits0..7, torso L/R -> bits9/10, pedal -> bit11.
    buttons_word = pcf_pressed
    if torso_l_pressed:
        buttons_word |= (1 << 9)
    if torso_r_pressed:
        buttons_word |= (1 << 10)
    if pedal_pressed:
        buttons_word |= (1 << 11)

    return buttons_word, pedal_connected


def _nau_read_raw24() -> int:
    """Read NAU7802 24-bit signed sample from ADCO registers."""
    buf = bytearray(3)

    if not _i2c_try_lock_with_timeout(_i2c_torso):
        raise RuntimeError("NAU I2C lock timeout")
    try:
        _i2c_torso.writeto_then_readfrom(ADDR_NAU7802, bytes([REG_ADCO_B2]), buf)
    finally:
        _i2c_torso.unlock()

    v = (buf[0] << 16) | (buf[1] << 8) | buf[2]
    if v & 0x800000:
        v -= 0x1000000
    return v


def update_nau_load16() -> int:
    """Update filtered load value and return signed 16-bit output sample."""
    global _load16, _load_filt, _load_filt_inited

    if not _nau_ok_runtime:
        return _load16

    # Keep main loop non-blocking when ADC has no fresh sample.
    if not _nau.available():
        return _load16

    raw_i = _nau_read_raw24()

    # First-order integer IIR for basic smoothing.
    if not _load_filt_inited:
        _load_filt = raw_i
        _load_filt_inited = True
    else:
        _load_filt = _load_filt + ((raw_i - _load_filt) >> LOAD_ALPHA_SHIFT)

    v = int(_load_filt >> LOAD_SHIFT)
    if v > 32767:
        v = 32767
    elif v < -32768:
        v = -32768
    _load16 = v
    return _load16


def update_lsm_roll16(now_ns: int) -> int:
    """Fuse LSM6 accel X/Z and gyro Y to produce signed int16 roll.

    Zero corresponds to X+ pointing down (aligned with gravity). Returns
    degrees*10 as signed int16 mapped to range +/-1800 (0.1 deg resolution).
    """
    global _lsm_roll_rad, _lsm_gyro_angle_rad, _lsm_roll16, _lsm_last_sample_ns, _lsm6, _lsm6_ok_runtime, _kf_angle

    if not _lsm6_ok_runtime or _lsm6 is None:
        return _lsm_roll16

    if _lsm_last_sample_ns == 0:
        dt = 0.004
    else:
        dt = (now_ns - _lsm_last_sample_ns) / 1_000_000_000
    if dt < 1e-6:
        dt = 1e-6
    elif dt > 0.05:
        dt = 0.05
    _lsm_last_sample_ns = now_ns

    try:
        ax, ay, az = _lsm6.acceleration  # m/s^2
        gx, gy, gz = _lsm6.gyro          # rad/s on Adafruit LSM6 drivers
    except Exception:
        return _lsm_roll16

    # Board mounting convention: invert Y gyro to match X-Z accel angle direction.
    gy = -gy

    # X-Z tilt angle: 0 when +X points down and Z ~= 0.
    import math
    accel_angle = math.atan2(az, -ax)

    if _lsm_roll16 == 0 and abs(_kf_angle) < 1e-6:
        # Initialize filter to measured angle to avoid startup transient.
        _kf_angle = accel_angle

    # Gyro Y is the rotation rate orthogonal to X-Z tilt plane.
    _lsm_gyro_angle_rad = _wrap_pi(_lsm_gyro_angle_rad + (gy * dt))
    fused_angle = _kalman_update_1d(accel_angle, gy, dt)
    _lsm_roll_rad = fused_angle

    roll_deg10 = int(round((fused_angle * 180.0 / 3.14159265) * 10.0))
    if roll_deg10 > 1800:
        roll_deg10 = 1800
    elif roll_deg10 < -1800:
        roll_deg10 = -1800
    _lsm_roll16 = roll_deg10

    return _lsm_roll16


def _scan_i2c_set(i2c, lock_timeout_s=0.05):
    """Scan one I2C bus and return addresses as a set."""
    if i2c is None:
        raise RuntimeError("I2C bus unavailable")
    t0 = time.monotonic()
    while not i2c.try_lock():
        if time.monotonic() - t0 > lock_timeout_s:
            raise RuntimeError("I2C lock timeout")
    try:
        return set(i2c.scan())
    finally:
        i2c.unlock()


def update_i2c_presence(period_s=1.0):
    """Refresh presence flags with strict allow-list policy per bus."""
    global _last_i2c_scan, _pcf_ok, _pcf_io_ok, _bno_ok, _lsm6_ok, _nau_ok

    now = time.monotonic()
    if now - _last_i2c_scan < period_s:
        return
    _last_i2c_scan = now

    _nau_ok = False
    _lsm6_ok = False
    _pcf_ok = False
    _bno_ok = False

    try:
        addrs_t = _scan_i2c_set(_i2c_torso)
        # Any unexpected torso address fails torso OK flags for this cycle.
        if addrs_t.issubset(ALLOWED_TORSO):
            _nau_ok = (ADDR_NAU7802 in addrs_t)
            _lsm6_ok = (ADDR_LSM6DSOX in addrs_t)
    except Exception:
        _nau_ok = False
        _lsm6_ok = False

    # Avoid bus scans on handset while BNO085 SH2 transport is active.
    # Address scans can interfere with BNO packet framing on shared I2C.
    if _bno_ok_runtime and _bno is not None:
        _bno_ok = True
        _pcf_ok = _pcf_io_ok
    else:
        try:
            addrs_h = _scan_i2c_set(_i2c_handset)
            # Any unexpected handset address fails handset OK flags for this cycle.
            if addrs_h.issubset(ALLOWED_HANDSET):
                _pcf_ok = (ADDR_PCF8574 in addrs_h)
                _bno_ok = (ADDR_BNO085 in addrs_h)
        except Exception:
            _pcf_ok = False
            _bno_ok = False


def make_status_bits(running_flag: bool, pedal_connected: bool = False) -> int:
    """Compose 16-bit status word matching the locked HID contract."""
    status = 0
    status |= (1 << 0)
    status |= (1 << 1)

    if running_flag:
        status |= (1 << 2)

    if _pcf_ok or _bno_ok:
        status |= (1 << 3)

    if pedal_connected:
        status |= (1 << 4)

    if _lsm6_ok:
        status |= (1 << 6)
    if _nau_ok:
        status |= (1 << 7)
    if _bno_ok:
        status |= (1 << 8)
    if _pcf_ok:
        status |= (1 << 9)

    bit_passed = supervisor.runtime.usb_connected and _lsm6_ok and _nau_ok and _bno_ok and _pcf_ok
    if bit_passed:
        status |= (1 << 5)

    return status


def get_load_debug_raw():
    """Expose filtered internal load accumulator for debug prints."""
    return _load_filt
