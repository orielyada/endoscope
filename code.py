"""
CircuitPython vendor HID firmware (device side).

Purpose:
- Expose a vendor-specific HID interface (usage_page 0xFF00, usage 0x0001)
- Stream a 32-byte INPUT report to the host at a configurable rate
- Accept a small OUTPUT report from the host to control:
    - START / STOP streaming
    - sample rate (Hz)  **Option A: 8-bit rate**
"""

import time
import struct
import usb_hid
import supervisor
import board
import digitalio
import busio

import digitalio
import board

pin_torso_l = digitalio.DigitalInOut(board.D10)
pin_torso_l.switch_to_input(pull=digitalio.Pull.UP)

pin_torso_r = digitalio.DigitalInOut(board.D9)
pin_torso_r.switch_to_input(pull=digitalio.Pull.UP)

pin_pedal = digitalio.DigitalInOut(board.D5)
pin_pedal.switch_to_input(pull=digitalio.Pull.UP)

pin_pedal_sense = digitalio.DigitalInOut(board.D6)
pin_pedal_sense.switch_to_input(pull=digitalio.Pull.UP)

# --- Torso bus (STEMMA QT): SCL=board.SCL, SDA=board.SDA ---

i2c_torso = busio.I2C(board.SCL, board.SDA)

# --- Handset bus: D25=SCL, D24=SDA ---

i2c_handset = busio.I2C(board.D25, board.D24)

# --- PCF8574 on handset bus ---
PCF_ADDR = 0x20

# Debounce state for PCF P0..P7
_pcf_last_raw = 0xFF          # last raw byte read (active-low typical, so idle high=1)
_pcf_stable = 0xFF            # debounced stable raw
_pcf_same_count = 0           # how many consecutive identical reads
PCF_DEBOUNCE_SAMPLES = 3      # change accepted after N same samples


class MaskDebounce:
    """
    Edge-immediate debouncer with symmetric minimum-state hold time.

    - Output changes immediately on an input edge.
    - After each change, further changes are ignored for hold_ms.
    """
    def __init__(self, hold_ms=10, initial=False):
        self.hold_ns = int(hold_ms * 1_000_000)
        self.state = bool(initial)          # debounced output (True=pressed)
        self._lock_until_ns = 0             # until this time, ignore further edges

    def update(self, raw_pressed: bool, now_ns: int) -> bool:
        # If we're in the mask window, ignore changes
        if now_ns < self._lock_until_ns:
            return self.state

        # If input differs from output and we're not locked, accept immediately
        if bool(raw_pressed) != self.state:
            self.state = bool(raw_pressed)
            self._lock_until_ns = now_ns + self.hold_ns

        return self.state

db_torso_l = MaskDebounce(hold_ms=10, initial=False)
db_torso_r = MaskDebounce(hold_ms=10, initial=False)
db_pedal   = MaskDebounce(hold_ms=10, initial=False)
db_pedsense= MaskDebounce(hold_ms=10, initial=False)  # for status "connected"


def pcf_init(i2c):
    """Put PCF8574 in 'input' mode by writing 1s to all pins."""
    # Writing 1 configures pin as input (quasi-bidirectional). Buttons then pull low.
    while not i2c.try_lock():
        pass
    try:
        i2c.writeto(PCF_ADDR, bytes([0xFF]))
    finally:
        i2c.unlock()

def pcf_read_raw(i2c) -> int:
    """Read raw PCF8574 port byte (bit=1 means high)."""
    buf = bytearray(1)
    while not i2c.try_lock():
        pass
    try:
        i2c.readfrom_into(PCF_ADDR, buf)
    finally:
        i2c.unlock()
    return buf[0]

def pcf_read_debounced(i2c) -> int:
    """
    Return debounced *raw* byte (still active-low).
    Debounce based on consecutive identical reads.
    """
    global _pcf_last_raw, _pcf_stable, _pcf_same_count

    raw = pcf_read_raw(i2c)

    if raw == _pcf_last_raw:
        if _pcf_same_count < 255:
            _pcf_same_count += 1
    else:
        _pcf_last_raw = raw
        _pcf_same_count = 0

    # Accept change after N consistent samples
    if _pcf_same_count >= (PCF_DEBOUNCE_SAMPLES - 1):
        _pcf_stable = _pcf_last_raw

    return _pcf_stable


# -------- Expected I2C addresses (verified / from your scans + datasheets) --------
ADDR_NAU7802 = 0x2A                 # fixed address :contentReference[oaicite:2]{index=2}
ADDR_LSM6DSOX = 0x6A                # 0x6A or 0x6B depending on SA0/SDO; you scanned 0x6A :contentReference[oaicite:3]{index=3}

ADDR_PCF8574 = 0x20                 # your handset scan showed 0x20 (typical base)
ADDR_BNO085  = 0x4A                 # your handset scan showed 0x4A

# -------- Cached presence flags (updated periodically) --------
pcf_ok = False
bno_ok = False
lsm6_ok = False
nau_ok = False

_last_i2c_scan = 0.0

def _scan_i2c_set(i2c, lock_timeout_s=0.05):
    t0 = time.monotonic()
    while not i2c.try_lock():
        if time.monotonic() - t0 > lock_timeout_s:
            raise RuntimeError("I2C lock timeout")
    try:
        return set(i2c.scan())
    finally:
        i2c.unlock()


# Required I2C addresses (strict)
ADDR_NAU7802 = 0x2A
ADDR_LSM6DSOX = 0x6A     
ADDR_PCF8574 = 0x20
ADDR_BNO085  = 0x4A

ALLOWED_TORSO   = {ADDR_NAU7802, ADDR_LSM6DSOX}
ALLOWED_HANDSET = {ADDR_PCF8574, ADDR_BNO085}

def update_i2c_presence(i2c_torso, i2c_handset, period_s=1.0):
    """Scan both buses ~1 Hz. Any 'extra junk' address on a bus => that bus fails."""
    global _last_i2c_scan, pcf_ok, bno_ok, lsm6_ok, nau_ok

    now = time.monotonic()
    if now - _last_i2c_scan < period_s:
        return
    _last_i2c_scan = now

    # Default to fail until proven otherwise
    nau_ok = False
    lsm6_ok = False
    pcf_ok = False
    bno_ok = False

    # ---- Torso bus ----
    try:
        addrs_t = _scan_i2c_set(i2c_torso)

        # If any address outside allowed set appears, treat as junk bus => fail both
        if not addrs_t.issubset(ALLOWED_TORSO):
            nau_ok = False
            lsm6_ok = False
        else:
            nau_ok  = (ADDR_NAU7802 in addrs_t)
            lsm6_ok = (ADDR_LSM6DSOX in addrs_t)
    except Exception:
        nau_ok = False
        lsm6_ok = False

    # ---- Handset bus ----
    try:
        addrs_h = _scan_i2c_set(i2c_handset)


        # Any address outside allowed set => junk bus => fail both
        if not addrs_h.issubset(ALLOWED_HANDSET):
            pcf_ok = False
            bno_ok = False
        else:
            pcf_ok = (ADDR_PCF8574 in addrs_h)
            bno_ok = (ADDR_BNO085  in addrs_h)
    except Exception:
        pcf_ok = False
        bno_ok = False



def make_status_bits(running_flag: bool) -> int:
    status = 0
    status |= (1 << 0)  # ready
    status |= (1 << 1)  # init_ok

    if running_flag:
        status |= (1 << 2)  # running

    # handset connected if at least one handset device is present
    handset_connected = (pcf_ok or bno_ok)
    if handset_connected:
        status |= (1 << 3)

    # device OK bits
    if lsm6_ok: status |= (1 << 6)
    if nau_ok:  status |= (1 << 7)
    if bno_ok:  status |= (1 << 8)
    if pcf_ok:  status |= (1 << 9)

    # BIT passed only if all devices are OK and USB is connected
    bit_passed = supervisor.runtime.usb_connected and lsm6_ok and nau_ok and bno_ok and pcf_ok
    if bit_passed:
        status |= (1 << 5)

    return status




# -----------------------------
# Debug / heartbeat printing
# -----------------------------

# Master switch for periodic debug prints.
DIAG = True

# Internal time gate to avoid flooding the serial console.
_last_diag = 0.0


def diag(msg: str, period_s: float = 0.5) -> None:
    """
    Print a periodic debug message no faster than `period_s`.

    This is intentionally rate-limited so your serial monitor stays readable.
    """
    global _last_diag
    if not DIAG:
        return

    now = time.monotonic()
    if now - _last_diag >= period_s:
        _last_diag = now
        print(msg)


# -----------------------------
# Find the HID device instance
# -----------------------------

# `dev` will hold the usb_hid.Device matching our vendor usage page/usage.
dev = None

# CircuitPython exposes all enabled HID interfaces in usb_hid.devices.
# We select the one we defined in boot.py:
#   usage_page = 0xFF00
#   usage      = 0x0001
for d in usb_hid.devices:
    if d.usage_page == 0xFF00 and d.usage == 0x0001:
        dev = d
        break

# If we didn't find it, something is wrong with boot.py USB config.
# In that case, just idle forever (so we don't crash/reboot loop).
if dev is None:
    while True:
        time.sleep(1)


# -----------------------------
# HID INPUT report definition
# -----------------------------

# Struct format for the 32-byte IN report.
# Little-endian ("<"), packed fields.
#
# Layout (as currently used):
#   seq:        H     (2 bytes)
#   qw,qx,qy,qz: hhhh  (8 bytes)   quaternion placeholder, signed 16-bit
#   knob0/1:    H H   (4 bytes)    unsigned 16-bit
#   slider0/1:  H H   (4 bytes)    unsigned 16-bit
#   load16:     h     (2 bytes)    signed 16-bit
#   roll16:     h     (2 bytes)    signed 16-bit
#   buttons:    H     (2 bytes)    bitfield
#   status:     H     (2 bytes)    debug signature
#   tail:       6s    (6 bytes)    magic + errcount + last OUT bytes
#
# Total = 2 + 8 + 4 + 4 + 2 + 2 + 2 + 2 + 6 = 32 bytes
FMT_IN = "<HhhhhHHHHhhHH6s"


# Pre-allocated buffer to avoid heap churn at high rates.
in_buf = bytearray(32)


# -----------------------------
# OUTPUT control protocol
# -----------------------------

# Command values for the first byte of the OUT payload.
# (Your host sends these.)
CMD_START = 0x01
CMD_STOP = 0x02

# IMPORTANT (Option A):
# - OUT payload is 3 bytes total: [cmd, rate_hz, reserved]
# - `rate_hz` is 8-bit Hz (0..255), we clamp it.
# - reserved byte is currently unused; we still capture it for debugging.


def clamp_sr(hz: int) -> int:
    """
    Clamp sample rate in Hz to a safe range.

    Notes:
    - If hz==0, we treat it as a request for default 500 Hz (your old behavior).
    - Otherwise we clamp 1..1000.
    """
    if hz == 0:
        return 500
    if hz < 1:
        return 1
    if hz > 1000:
        return 1000
    return int(hz)


# -----------------------------
# State variables
# -----------------------------

# Sequence counter (16-bit) increments every time we send an IN report.
seq = 0

# Whether the device considers streaming "running".
# (Right now you still send reports even when running=False;
#  you just change the status signature. If you later want "STOP"
#  to stop sending, we can gate send_report() on this flag.)
running = False

# Current report rate (Hz). Start slow so it's easy to observe changes.
report_hz = 100

# Scheduler timestamp for next report send (nanoseconds).
next_tick_ns = time.monotonic_ns()

# Sensor/inputs placeholders (currently fixed values).
# Keeping them as separate named variables makes it easy to later replace
# with real sensor reads.
qw, qx, qy, qz = 32767, 0, 0, 0  # quaternion (q_w near 1.0 scaled)
knob0 = knob1 = 0                 # 16-bit knobs
slider0 = slider1 = 0             # 16-bit sliders
load16 = 0                        # signed 16-bit load
roll16 = 0                        # unsigned 16-bit roll
buttons = 0                       # 16-bit bitfield for buttons

# Debug tail fields in last 6 bytes of IN report:
# tail = [magic, err_lo, err_hi, out0, out1, out2]
magic = 0xA5
errcount = 0

# Last 3 bytes of OUT payload we received (for host-side visibility).
# Initialize to all zeros.
out3 = b"\x00\x00\x00"

# Cache of last raw OUT report bytes to suppress duplicates.
last_out = None


def make_status(running_flag: bool) -> int:
    """
    Produce a simple status signature.
    Useful to verify the device received START/STOP, even if you don't
    change any other behavior yet.
    """
    return 0x5678 if running_flag else 0x1234


def poll_out_and_apply() -> None:
    """
    Poll the HID OUT report (control packet) and apply it to device state.

    This reads the most recent OUT report using dev.get_last_received_report().
    If there is no new report, it returns quickly.

    Option A fix here:
    - sample rate is 8-bit (one byte), NOT 16-bit.
    """
    global running, report_hz, last_out, out3

    # Get the last OUT report (if any).
    r = dev.get_last_received_report()
    if not r:
        return

    # Convert to immutable bytes so it can be compared/stored safely.
    rb = bytes(r)

    # Ignore if it's identical to the last report (de-dupe).
    if rb == last_out:
        return
    last_out = rb

    # Print raw bytes for visibility while debugging.
    # (You can disable/remove later.)
    print("OUT rb=", list(rb))

    # ---- Parse payload (best effort) ----
    #
    # Some host APIs include a leading "report ID" byte even when you
    # did not explicitly define a report ID. In your observed setup with
    # hidapitester, you receive exactly 3 bytes (no leading 0).
    #
    # We'll support both formats:
    #   A) [0, cmd, rate, reserved]  (len>=4, leading 0)
    #   B) [cmd, rate, reserved]     (len>=3)
    #
    # Option A: `rate` is a SINGLE byte in Hz.
    if len(rb) >= 4 and rb[0] == 0:
        # Skip the leading report-id byte.
        out3 = bytes(rb[1:4])

        cmd = rb[1]
        sr = rb[2]  # <-- FIX: 8-bit Hz (Option A)
    elif len(rb) >= 3:
        out3 = bytes(rb[0:3])

        cmd = rb[0]
        sr = rb[1]  # <-- FIX: 8-bit Hz (Option A)
    else:
        # Too short. Capture what we can into out3 for debugging and exit.
        out3 = (rb + b"\x00\x00\x00")[:3]
        return

    # Print parsed payload for confirmation.
    print("OUT raw:", rb, "parsed:", out3, "cmd:", cmd, "sr8:", sr)

    # ---- Apply command ----
    if cmd == CMD_START:
        report_hz = clamp_sr(sr)
        running = True
        print("APPLY START: running=True report_hz=", report_hz)
    elif cmd == CMD_STOP:
        running = False
        print("APPLY STOP: running=False (report_hz stays=", report_hz, ")")
    else:
        # Unknown command: ignore but keep out3 visible in the IN tail.
        print("APPLY: unknown cmd", cmd, "(ignored)")


# -----------------------------
# initialize devices
# -----------------------------

pcf_init(i2c_handset)




# -----------------------------
# Main loop
# -----------------------------

while True:
    try:
        # 1) Poll host control packet (OUT report).
        poll_out_and_apply()

        update_i2c_presence(i2c_torso, i2c_handset, period_s=1.0)
        status_bits = make_status_bits(running)

        diag(f"flags lsm6={lsm6_ok} nau={nau_ok} bno={bno_ok} pcf={pcf_ok} status=0x{status_bits:04X}", period_s=1.0)

        reserved = b"\x00" * 6

        
        # 2) Compute scheduling for periodic IN report sends.
        now_ns = time.monotonic_ns()

        # Avoid division by zero (report_hz is clamped anyway).
        period_ns = int(1_000_000_000 // report_hz)

        # raw reads: pull-up => pressed/connected is low (0)
        raw_torso_l_pressed = (pin_torso_l.value == False)
        raw_torso_r_pressed = (pin_torso_r.value == False)
        raw_pedal_pressed   = (pin_pedal.value == False)
        # pedal_sense: connected pulls low
        raw_pedal_connected = (pin_pedal_sense.value == False)

        torso_l_pressed = db_torso_l.update(raw_torso_l_pressed, now_ns)
        torso_r_pressed = db_torso_r.update(raw_torso_r_pressed, now_ns)
        pedal_pressed   = db_pedal.update(raw_pedal_pressed, now_ns)
        pedal_connected = db_pedsense.update(raw_pedal_connected, now_ns)



        # 3) If it's time, build and send the next IN report.
        if now_ns >= next_tick_ns:
            next_tick_ns = now_ns + period_ns

            # Sequence increments each report.
            seq = (seq + 1) & 0xFFFF

            # ---- Build buttons_bits from scratch (deterministic) ----

            # PCF buttons (P0..P7) => bits 0..7, pressed=1
            pcf_raw = pcf_read_debounced(i2c_handset)
            pcf_pressed = (~pcf_raw) & 0xFF

            buttons_word = pcf_pressed  # bits0..7

            # ---- Build buttons_bits (uint16) deterministically for THIS report ----

            # PCF P0..P7 -> bits0..7 (pressed=1)
            pcf_raw = pcf_read_debounced(i2c_handset)      # raw: 1=high, 0=low
            pcf_pressed = (~pcf_raw) & 0xFF                # pressed pulls low

            buttons_word = pcf_pressed                     # bits0..7

            # torso L/R -> bits9..10
            if torso_l_pressed:
                buttons_word |= (1 << 9)
            if torso_r_pressed:
                buttons_word |= (1 << 10)

            # pedal -> bit11
            if pedal_pressed:
                buttons_word |= (1 << 11)

            # status bit4 = pedal_connected
            if pedal_connected:
                status_bits |= (1 << 4)
            else:
                status_bits &= ~(1 << 4)



            # Pack the report into the 32-byte buffer.
            struct.pack_into(
                FMT_IN, in_buf, 0,
                seq,
                qw, qx, qy, qz,
                knob0, knob1,
                slider0, slider1,
                load16,
                roll16,
                buttons_word,
                status_bits,
                reserved
            )

            # If running, send to host as HID INPUT report (32 bytes).
            if running:
                dev.send_report(in_buf)

        # 4) Low-rate heartbeat so you know code is alive + USB is connected.
        diag(f"alive t={time.monotonic():.1f} usb={supervisor.runtime.usb_connected}")
        diag(
            "GPIO raw: TL=%d TR=%d P=%d PS=%d | debounced: TL=%d TR=%d P=%d PS=%d" % (
                int(pin_torso_l.value), int(pin_torso_r.value),
                int(pin_pedal.value), int(pin_pedal_sense.value),
                int(torso_l_pressed), int(torso_r_pressed),
                int(pedal_pressed), int(pedal_connected),
            ),
        )


        # 5) Yield to USB stack / scheduler.
        time.sleep(0)

    except Exception as e:
        # If anything goes wrong, don't crash the firmware.
        # Count errors and keep looping.
        errcount = (errcount + 1) & 0xFFFF

        print("EXC:", repr(e))


        # Small delay prevents tight exception loops from starving USB.
        time.sleep(0.01)
