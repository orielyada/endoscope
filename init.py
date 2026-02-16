import board
import busio
import digitalio
import time
from cedargrove_nau7802 import NAU7802


def init_gpio_inputs():
    """Configure local GPIO inputs used for torso buttons and pedal lines."""
    pin_torso_l = digitalio.DigitalInOut(board.D10)
    pin_torso_l.switch_to_input(pull=digitalio.Pull.UP)

    pin_torso_r = digitalio.DigitalInOut(board.D9)
    pin_torso_r.switch_to_input(pull=digitalio.Pull.UP)

    pin_pedal = digitalio.DigitalInOut(board.D5)
    pin_pedal.switch_to_input(pull=digitalio.Pull.UP)

    pin_pedal_sense = digitalio.DigitalInOut(board.D6)
    pin_pedal_sense.switch_to_input(pull=digitalio.Pull.UP)

    return {
        "torso_l": pin_torso_l,
        "torso_r": pin_torso_r,
        "pedal": pin_pedal,
        "pedal_sense": pin_pedal_sense,
    }


def init_i2c_buses():
    """Create torso and handset I2C buses on their dedicated pin pairs."""
    def _attempt_i2c_bus_recovery(scl, sda):
        """Try to recover a stuck I2C bus by clocking SCL while monitoring SDA."""
        try:
            scl_pin = digitalio.DigitalInOut(scl)
            sda_pin = digitalio.DigitalInOut(sda)
            scl_pin.switch_to_output(value=True)
            sda_pin.switch_to_output(value=True)
            
            # Clock SCL up to 9 times to try to clear stuck slave
            for _ in range(9):
                scl_pin.value = False
                time.sleep(0.01)  # 10ms low time
                scl_pin.value = True
                time.sleep(0.01)  # 10ms high time
            
            # Send STOP condition: SDA goes low then high while SCL is high
            scl_pin.value = True
            sda_pin.value = False
            time.sleep(0.01)
            sda_pin.value = True
            time.sleep(0.01)
            
            scl_pin.deinit()
            sda_pin.deinit()
            time.sleep(0.1)  # Wait after recovery
        except Exception as e:
            pass

    def _init_i2c_with_retry(scl, sda, label, retries=20, delay_s=0.05):
        last_exc = None
        for attempt in range(retries):
            try:
                # Keep bus speed at 100 kHz for BNO085 compatibility
                i2c = busio.I2C(scl, sda, frequency=100000)
                # Success - wait a bit before returning
                time.sleep(0.2)
                return i2c
            except Exception as e:
                last_exc = e
                if attempt == retries // 2:
                    # Try bus recovery halfway through retries
                    _attempt_i2c_bus_recovery(scl, sda)
                time.sleep(delay_s)
        return None

    def _get_pin(name):
        return getattr(board, name, None)

    def _init_handset_candidates():
        # Try likely handset mappings in order; first success wins.
        candidates = []

        d25 = _get_pin("D25")
        d24 = _get_pin("D24")
        if d25 is not None and d24 is not None:
            candidates.append(("D25/D24", d25, d24))
            candidates.append(("D24/D25", d24, d25))

        scl1 = _get_pin("SCL1")
        sda1 = _get_pin("SDA1")
        if scl1 is not None and sda1 is not None:
            candidates.append(("SCL1/SDA1", scl1, sda1))

        # De-duplicate by object identity while preserving order.
        uniq = []
        seen = set()
        for label, scl, sda in candidates:
            key = (id(scl), id(sda))
            if key in seen:
                continue
            seen.add(key)
            uniq.append((label, scl, sda))

        for label, scl, sda in uniq:
            i2c = _init_i2c_with_retry(scl, sda, f"handset({label})")
            if i2c is not None:
                return i2c
        return None

    i2c_torso = None
    i2c_handset = None
    i2c_torso = _init_i2c_with_retry(board.SCL, board.SDA, "torso")
    i2c_handset = _init_handset_candidates()
    if i2c_handset is None:
        if i2c_torso is not None:
            # Fallback for builds where handset peripherals are actually on the main I2C chain.
            i2c_handset = i2c_torso
    return i2c_torso, i2c_handset


def init_nau7802(i2c_torso):
    """Initialize NAU7802 and return (device, runtime_ok_flag)."""
    if i2c_torso is None:
        print("NAU init skipped: torso I2C unavailable")
        return None, False
    nau = NAU7802(i2c_torso, address=0x2A, active_channels=2)
    nau_ok_runtime = False
    try:
        # Enable analog/digital path before any reads.
        nau_ok_runtime = bool(nau.enable(True))
        # Channel numbering is 1-based in this driver.
        nau.channel = 1
        # Run baseline calibration with no load.
        nau.calibrate("INTERNAL")
        nau.calibrate("OFFSET")
    except Exception as e:
        print("NAU init failed:", repr(e))
        nau_ok_runtime = False
    return nau, nau_ok_runtime


def init_lsm6(i2c_torso):
    """Initialize LSM6DSOX and return (device, runtime_ok_flag)."""
    if i2c_torso is None:
        print("LSM6 init skipped: torso I2C unavailable")
        return None, False
    try:
        from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
    except Exception as e:
        print("LSM6 lib missing:", repr(e))
        return None, False

    try:
        lsm = LSM6DSOX(i2c_torso)
        return lsm, True
    except Exception as e:
        print("LSM6 init failed:", repr(e))
        return None, False


def init_bno085(i2c_handset, report_hz=250):
    """Initialize BNO085 rotation-vector output and return (device, runtime_ok_flag)."""
    if i2c_handset is None:
        print("BNO085 init skipped: handset I2C unavailable")
        return None, False
    
    # Give BNO085 time to be ready after bus initialization
    time.sleep(0.5)
    
    try:
        from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
        from adafruit_bno08x.i2c import BNO08X_I2C
    except ImportError as e:
        print("BNO085 lib missing:", repr(e))
        return None, False

    bno = None
    bno_ok_runtime = False
    try:
        bno = BNO08X_I2C(i2c_handset, address=0x4A)
        # Don't pass report_interval_us - use library defaults
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        
        # Wait for sensor to stabilize
        time.sleep(0.5)
        bno_ok_runtime = True
    except Exception as e:
        print("BNO085 init failed:", repr(e))
        bno = None
        bno_ok_runtime = False
    return bno, bno_ok_runtime


def find_vendor_hid_device(usb_hid, usage_page=0xFF00, usage=0x0001):
    """Return matching vendor HID device or None if not present."""
    for dev in usb_hid.devices:
        if dev.usage_page == usage_page and dev.usage == usage:
            return dev
    return None
