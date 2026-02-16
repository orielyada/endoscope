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

from init import find_vendor_hid_device, init_adcs, init_bno085, init_gpio_inputs, init_i2c_buses, init_nau7802, init_lsm6
import readout


# -----------------------------
# Debug / heartbeat printing
# -----------------------------

DIAG = True
ENABLE_BNO085 = True
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

print("BOOT: code.py starting")
try:
    print("BOOT: init gpio")
    pins = init_gpio_inputs()
    print("BOOT: init ADCs")
    adcs = init_adcs()
    print("BOOT: init i2c")
    i2c_torso, i2c_handset = init_i2c_buses()
    print("BOOT: init NAU7802")
    nau, nau_ok_runtime = init_nau7802(i2c_torso)
    print("BOOT: init LSM6DSOX")
    lsm6, lsm6_ok_runtime = init_lsm6(i2c_torso)
    if ENABLE_BNO085:
        print("BOOT: init BNO085")
        bno, bno_ok_runtime = init_bno085(i2c_handset, report_hz=250)
        print("BOOT: init BNO085 done ok=", bno_ok_runtime)
    else:
        bno, bno_ok_runtime = None, False
        print("BOOT: BNO085 disabled")

    # Match vendor HID endpoint created in boot.py.
    dev = find_vendor_hid_device(usb_hid, usage_page=0xFF00, usage=0x0001)
    if dev is None:
        raise RuntimeError("Vendor HID endpoint not found")

    readout.setup_readout(
        pins=pins,
        i2c_torso=i2c_torso,
        i2c_handset=i2c_handset,
        nau=nau,
        nau_ok_runtime=nau_ok_runtime,
        bno=bno,
        bno_ok_runtime=bno_ok_runtime,
        lsm6=lsm6,
        lsm6_ok_runtime=lsm6_ok_runtime,
    )
    readout.pcf_init()
    print("BOOT: readout setup done")
except Exception as e:
    print("BOOT EXC:", repr(e))
    while True:
        time.sleep(1)


# -----------------------------
# HID INPUT report definition
# -----------------------------

# Quaternion fields are int16 in (w, x, y, z) order.
FMT_IN = "<HhhhhHHHHhhHH6s"
in_buf = bytearray(32)
# Tail bytes are reserved except [1:0] which carry current report_hz (uint16 LE).
RESERVED_BYTES = bytearray(6)


# -----------------------------
# OUTPUT control protocol
# -----------------------------

CMD_START = 0x01
CMD_STOP = 0x02
# OUT payload (without optional host-side leading report-id/dummy byte):
# [cmd, sr_lo, sr_hi, flags]
ADC_FULL_SCALE = 65535


def clamp_sr(hz: int) -> int:
    """Clamp sample rate to supported 1..1000 Hz (0 defaults to 500)."""
    if hz == 0:
        return 500
    if hz < 1:
        return 1
    if hz > 1000:
        return 1000
    return int(hz)


def slider2_transform(raw: int, full_scale: int = ADC_FULL_SCALE) -> int:
    """
    Linearize rheostat-to-GND with 10k pullup:
      y = Vout/Vcc = R/(R+10k)
      x = R/10k = y/(1-y)
    Return x mapped to uint16 [0..65535].
    """
    fs = int(full_scale)
    r = int(raw)
    if r <= 0:
        return 0
    den = int(full_scale) - int(raw)
    if den <= 0:
        return 65535
    # x in [0..1] over ideal usable range (raw up to fs/2), scaled to uint16.
    v = (float(r) / float(den)) * float(fs)
    if v >= float(fs):
        return 65535
    if v <= 0.0:
        return 0
    return int(v)


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

    # Support both styles while preserving the known leading dummy-byte quirk:
    # [0, cmd, sr_lo, sr_hi, flags, ...] and [cmd, sr_lo, sr_hi, flags, ...].
    # Keep fallbacks for older packet shapes.
    if len(rb) >= 5 and rb[0] == 0:
        cmd = rb[1]
        sr = rb[2] | (rb[3] << 8)
    elif len(rb) >= 4:
        if rb[0] == 0:
            cmd = rb[1]
            sr = rb[2] | (rb[3] << 8)
        else:
            cmd = rb[0]
            sr = rb[1] | (rb[2] << 8)
    elif len(rb) >= 3:
        if rb[0] == 0:
            cmd = rb[1]
            sr = rb[2]
        else:
            cmd = rb[0]
            sr = rb[1] | (rb[2] << 8)
    elif len(rb) >= 2:
        cmd = rb[0]
        sr = rb[1]
    else:
        return

    print("OUT raw:", rb, "cmd:", cmd, "sr16:", sr)

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
        # Keep fusion running at loop speed; HID rate only controls transmit cadence.
        roll16 = readout.update_lsm_roll16(now_ns)
        period_ns = int(1_000_000_000 // report_hz)

        if now_ns >= next_tick_ns:
            next_tick_ns = now_ns + period_ns
            seq = (seq + 1) & 0xFFFF

            qw, qx, qy, qz = readout.update_bno_quat_i16(now_ns)
            knob0 = adcs[0].value if adcs[0] is not None else 0
            knob1 = adcs[1].value if adcs[1] is not None else 0
            slider0 = adcs[2].value if adcs[2] is not None else 0
            slider1_raw = adcs[3].value if adcs[3] is not None else 0
            slider1 = slider2_transform(slider1_raw)

            buttons_word, pedal_connected = readout.sample_buttons(now_ns)
            status_bits = readout.make_status_bits(running, pedal_connected)
            RESERVED_BYTES[0] = report_hz & 0xFF
            RESERVED_BYTES[1] = (report_hz >> 8) & 0xFF

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

        if (seq & 0x0F) == 0:
            # Heartbeat only; no verbose quaternion debug printing
            pass

        time.sleep(0)

    except Exception as e:
        # Keep firmware alive through transient read/USB errors.
        errcount = (errcount + 1) & 0xFFFF
        print("EXC:", repr(e))
        time.sleep(0.01)
