import time
import supervisor


PCF_ADDR = 0x20
ADDR_NAU7802 = 0x2A
REG_ADCO_B2 = 0x12
ADDR_LSM6DSOX = 0x6A
ADDR_PCF8574 = 0x20
ADDR_BNO085 = 0x4A

LOAD_SHIFT = 5
LOAD_ALPHA_SHIFT = 3
PCF_DEBOUNCE_SAMPLES = 3

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

_load16 = 0
_load_filt = 0
_load_filt_inited = False

_pcf_last_raw = 0xFF
_pcf_stable = 0xFF
_pcf_same_count = 0

_pcf_ok = False
_bno_ok = False
_lsm6_ok = False
_nau_ok = False
_last_i2c_scan = 0.0

_db_torso_l = MaskDebounce(hold_ms=10, initial=False)
_db_torso_r = MaskDebounce(hold_ms=10, initial=False)
_db_pedal = MaskDebounce(hold_ms=10, initial=False)
_db_pedsense = MaskDebounce(hold_ms=10, initial=False)


def setup_readout(pins, i2c_torso, i2c_handset, nau, nau_ok_runtime):
    """Inject hardware handles/state so readout functions stay module-local."""
    global _pins, _i2c_torso, _i2c_handset, _nau, _nau_ok_runtime
    _pins = pins
    _i2c_torso = i2c_torso
    _i2c_handset = i2c_handset
    _nau = nau
    _nau_ok_runtime = bool(nau_ok_runtime)


def pcf_init():
    """Set PCF8574 lines high so they behave as inputs (quasi-bidirectional)."""
    while not _i2c_handset.try_lock():
        pass
    try:
        _i2c_handset.writeto(PCF_ADDR, bytes([0xFF]))
    finally:
        _i2c_handset.unlock()


def pcf_read_raw() -> int:
    """Read raw PCF8574 byte where 1=high and 0=low."""
    buf = bytearray(1)
    while not _i2c_handset.try_lock():
        pass
    try:
        _i2c_handset.readfrom_into(PCF_ADDR, buf)
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

    while not _i2c_torso.try_lock():
        pass
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


def _scan_i2c_set(i2c, lock_timeout_s=0.05):
    """Scan one I2C bus and return addresses as a set."""
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
    global _last_i2c_scan, _pcf_ok, _bno_ok, _lsm6_ok, _nau_ok

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
