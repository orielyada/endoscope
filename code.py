"""
CircuitPython vendor HID firmware (device side).

Purpose:
- Expose a vendor-specific HID interface (usage_page 0xFF00, usage 0x0001)
- Stream a 32-byte INPUT report to the host at a configurable rate
- Accept a small OUTPUT report from the host to control:
    - START / STOP streaming
    - sample rate (Hz)
"""

import struct
import time
import supervisor
import usb_hid

from init import find_vendor_hid_device, init_gpio_inputs, init_i2c_buses, init_nau7802
import readout


# -----------------------------
# Debug / heartbeat printing
# -----------------------------

DIAG = True
_last_diag = 0.0


def diag(msg: str, period_s: float = 0.5) -> None:
    """Rate-limited debug print so serial output stays readable."""
    global _last_diag
    if not DIAG:
        return

    now = time.monotonic()
    if now - _last_diag >= period_s:
        _last_diag = now
        print(msg)


# -----------------------------
# Hardware init
# -----------------------------

pins = init_gpio_inputs()
i2c_torso, i2c_handset = init_i2c_buses()
nau, nau_ok_runtime = init_nau7802(i2c_torso)

# Match vendor HID endpoint created in boot.py.
dev = find_vendor_hid_device(usb_hid, usage_page=0xFF00, usage=0x0001)
if dev is None:
    # Safe idle if USB config is not present.
    while True:
        time.sleep(1)

readout.setup_readout(
    pins=pins,
    i2c_torso=i2c_torso,
    i2c_handset=i2c_handset,
    nau=nau,
    nau_ok_runtime=nau_ok_runtime,
)
readout.pcf_init()


# -----------------------------
# HID INPUT report definition
# -----------------------------

FMT_IN = "<HhhhhHHHHhhHH6s"
in_buf = bytearray(32)
# Contract requires reserved bytes to remain zero.
RESERVED_BYTES = b"\x00" * 6


# -----------------------------
# OUTPUT control protocol
# -----------------------------

CMD_START = 0x01
CMD_STOP = 0x02


def clamp_sr(hz: int) -> int:
    """Clamp sample rate to supported 1..1000 Hz (0 defaults to 500)."""
    if hz == 0:
        return 500
    if hz < 1:
        return 1
    if hz > 1000:
        return 1000
    return int(hz)


# -----------------------------
# Runtime state
# -----------------------------

seq = 0
running = False
report_hz = 100
next_tick_ns = time.monotonic_ns()

qw, qx, qy, qz = 32767, 0, 0, 0
knob0 = knob1 = 0
slider0 = slider1 = 0
load16 = 0
roll16 = 0

errcount = 0
last_out = None


def poll_out_and_apply() -> None:
    """Parse latest OUT report and apply START/STOP + sample-rate changes."""
    global running, report_hz, last_out

    # CircuitPython returns most recent OUT report or None.
    r = dev.get_last_received_report()
    if not r:
        return

    # De-duplicate repeated host writes.
    rb = bytes(r)
    if rb == last_out:
        return
    last_out = rb

    print("OUT rb=", list(rb))

    # Support both styles:
    # [0, cmd, rate, ...] from hidapitester and [cmd, rate, ...] raw payload.
    if len(rb) >= 4 and rb[0] == 0:
        cmd = rb[1]
        sr = rb[2]
    elif len(rb) >= 3:
        cmd = rb[0]
        sr = rb[1]
    else:
        return

    print("OUT raw:", rb, "cmd:", cmd, "sr8:", sr)

    if cmd == CMD_START:
        report_hz = clamp_sr(sr)
        running = True
        print("APPLY START: running=True report_hz=", report_hz)
    elif cmd == CMD_STOP:
        # Hard gate is in main loop send path.
        running = False
        print("APPLY STOP: running=False (report_hz stays=", report_hz, ")")
    else:
        print("APPLY: unknown cmd", cmd, "(ignored)")


# -----------------------------
# Main loop
# -----------------------------

while True:
    try:
        # 1) Apply host control commands.
        poll_out_and_apply()

        # 2) Refresh readout-side state and latest load sample.
        readout.update_i2c_presence(period_s=1.0)
        load16 = readout.update_nau_load16()

        # 3) Stream scheduler based on current rate.
        now_ns = time.monotonic_ns()
        period_ns = int(1_000_000_000 // report_hz)

        if now_ns >= next_tick_ns:
            next_tick_ns = now_ns + period_ns
            seq = (seq + 1) & 0xFFFF

            buttons_word, pedal_connected = readout.sample_buttons(now_ns)
            status_bits = readout.make_status_bits(running, pedal_connected)

            # 4) Pack one 32-byte report exactly per locked HID layout.
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
                RESERVED_BYTES
            )

            if running:
                dev.send_report(in_buf)

        # 5) Heartbeat + occasional load debug.
        diag(f"alive t={time.monotonic():.1f} usb={supervisor.runtime.usb_connected}")

        if (seq & 0x03FF) == 0:
            print("NAU raw24:", readout.get_load_debug_raw(), " load16:", load16)

        time.sleep(0)

    except Exception as e:
        # Keep firmware alive through transient read/USB errors.
        errcount = (errcount + 1) & 0xFFFF
        print("EXC:", repr(e))
        time.sleep(0.01)
