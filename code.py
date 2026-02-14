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
#   roll16:     H     (2 bytes)    unsigned 16-bit (as you had it; keep)
#   buttons:    H     (2 bytes)    bitfield
#   status:     H     (2 bytes)    debug signature
#   tail:       6s    (6 bytes)    magic + errcount + last OUT bytes
#
# Total = 2 + 8 + 4 + 4 + 2 + 2 + 2 + 2 + 6 = 32 bytes
FMT_IN = "<HhhhhHHHHhHHH6s"

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
# Main loop
# -----------------------------

while True:
    try:
        # 1) Poll host control packet (OUT report).
        poll_out_and_apply()

        # 2) Compute scheduling for periodic IN report sends.
        now_ns = time.monotonic_ns()

        # Avoid division by zero (report_hz is clamped anyway).
        period_ns = int(1_000_000_000 // report_hz)

        # 3) If it's time, build and send the next IN report.
        if now_ns >= next_tick_ns:
            next_tick_ns = now_ns + period_ns

            # Sequence increments each report.
            seq = (seq + 1) & 0xFFFF

            # Status signature shows whether we've applied START/STOP.
            status = make_status(running)

            # Tail: magic + errcount + last OUT payload (3 bytes).
            tail = bytes((
                magic,
                errcount & 0xFF,
                (errcount >> 8) & 0xFF,
                out3[0], out3[1], out3[2],
            ))

            # Pack the report into the 32-byte buffer.
            struct.pack_into(
                FMT_IN, in_buf, 0,
                seq,
                qw, qx, qy, qz,
                knob0, knob1,
                slider0, slider1,
                load16,
                roll16,
                buttons,
                status,
                tail
            )

            # Send to host as HID INPUT report (32 bytes).
            dev.send_report(in_buf)

        # 4) Low-rate heartbeat so you know code is alive + USB is connected.
        diag(f"alive t={time.monotonic():.1f} usb={supervisor.runtime.usb_connected}")

        # 5) Yield to USB stack / scheduler.
        time.sleep(0)

    except Exception as e:
        # If anything goes wrong, don't crash the firmware.
        # Count errors and keep looping.
        errcount = (errcount + 1) & 0xFFFF

        # Small delay prevents tight exception loops from starving USB.
        time.sleep(0.01)
