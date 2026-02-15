import board
import busio
import digitalio
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
    i2c_torso = busio.I2C(board.SCL, board.SDA)
    i2c_handset = busio.I2C(board.D25, board.D24)
    return i2c_torso, i2c_handset


def init_nau7802(i2c_torso):
    """Initialize NAU7802 and return (device, runtime_ok_flag)."""
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


def find_vendor_hid_device(usb_hid, usage_page=0xFF00, usage=0x0001):
    """Return matching vendor HID device or None if not present."""
    for dev in usb_hid.devices:
        if dev.usage_page == usage_page and dev.usage == usage:
            return dev
    return None
