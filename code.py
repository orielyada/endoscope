"""
CircuitPython joystick HID firmware (device side).

Purpose:
- Expose a custom joystick HID interface (usage_page 0x01, usage 0x04)
- Stream INPUT report ID 1 (16 bytes) and report ID 2 (10 bytes)
- Accept OUTPUT report ID 3 (4-byte payload) to control START/STOP + report_hz
"""

import struct
import time
import supervisor
import usb_hid
import board
import digitalio

from init import find_hid_device, init_adcs, init_bno085, init_gpio_inputs, init_i2c_buses, init_nau7802, init_lsm6
import readout


# -----------------------------
# Debug / heartbeat printing
# -----------------------------

DIAG = True
ENABLE_BNO085 = True
_last_diag = 0.0
_led_red = None
_led_green = None


def diag(msg: str, period_s: float = 0.5) -> None:
    """Rate-limited debug print so serial output stays readable."""
    global _last_diag
    if not DIAG:
        return

    now = time.monotonic()
    if now - _last_diag >= period_s:
        _last_diag = now
        print(msg)


def init_status_leds() -> None:
    """Init dual-color status LED: red on TX, green on RX (active high)."""
    global _led_red, _led_green
    try:
        _led_red = digitalio.DigitalInOut(board.TX)
        _led_red.switch_to_output(value=False)
    except Exception as e:
        _led_red = None
        print("LED red(TX) init failed:", repr(e))
    try:
        _led_green = digitalio.DigitalInOut(board.RX)
        _led_green.switch_to_output(value=False)
    except Exception as e:
        _led_green = None
        print("LED green(RX) init failed:", repr(e))


def _set_leds(red_on: bool, green_on: bool) -> None:
    if _led_red is not None:
        _led_red.value = bool(red_on)
    if _led_green is not None:
        _led_green.value = bool(green_on)


def update_status_leds(now_s: float, status_bits: int) -> None:
    """
    LED logic:
    - bit0=1 and bit2=0 -> orange (red+green)
    - bit0=1 and bit2=1 -> green
    - else -> red blinking at 1 Hz
    """
    b0 = bool(status_bits & (1 << 0))
    b2 = bool(status_bits & (1 << 2))
    if b0 and not b2:
        _set_leds(True, True)
    elif b0 and b2:
        _set_leds(False, True)
    else:
        red_on = (int(now_s * 2.0) & 1) == 0
        _set_leds(red_on, False)


# -----------------------------
# HID constants
# -----------------------------

RID_INPUT_MAIN = 0x01
RID_INPUT_QUAT = 0x02
RID_OUTPUT_CTRL = 0x03

CMD_START = 0x01
CMD_STOP = 0x02

ADC_FULL_SCALE = 65535
MAX_R1_BURST = 8


# -----------------------------
# Runtime state
# -----------------------------

running = False
report_hz = 100

r1_counter = 0
r2_counter = 0

next_tick_r1_ns = time.monotonic_ns()
next_tick_r2_ns = time.monotonic_ns()

load16 = 0
roll16 = 0

last_out = None
last_status_bits = None
errcount = 0

r1_buf = bytearray(16)
r2_buf = bytearray(10)


# -----------------------------
# Helpers
# -----------------------------


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
    den = fs - r
    if den <= 0:
        return 65535
    v = (float(r) / float(den)) * float(fs)
    if v >= float(fs):
        return 65535
    if v <= 0.0:
        return 0
    return int(v)


def u16_to_i16(v: int) -> int:
    """Map unsigned 16-bit ADC values to signed int16 centered at zero."""
    x = int(v) - 32768
    if x < -32768:
        return -32768
    if x > 32767:
        return 32767
    return x


def _send_input_report(report_id: int, payload: bytearray) -> None:
    """Send HID INPUT report with compatibility fallback for older APIs."""
    try:
        dev.send_report(payload, report_id=report_id)
    except TypeError:
        raw = bytearray(1 + len(payload))
        raw[0] = report_id & 0xFF
        raw[1:] = payload
        dev.send_report(raw)


def _read_out_report() -> bytes | None:
    """Read most recent OUT report, preferring report-id filtered reads."""
    try:
        r = dev.get_last_received_report(report_id=RID_OUTPUT_CTRL)
    except TypeError:
        r = dev.get_last_received_report()
    if not r:
        return None
    return bytes(r)


def _decode_out_cmd(rb: bytes) -> tuple[int, int] | None:
    """Decode OUT command from report bytes, tolerating old host packet shapes."""
    if len(rb) < 2:
        return None

    offset = 0
    if rb[0] in (0x00, RID_OUTPUT_CTRL) and len(rb) >= 4:
        offset = 1

    remain = len(rb) - offset
    if remain >= 3:
        cmd = rb[offset]
        sr = rb[offset + 1] | (rb[offset + 2] << 8)
        return cmd, sr
    if remain >= 2:
        cmd = rb[offset]
        sr = rb[offset + 1]
        return cmd, sr
    return None


def _compose_status_from_core(core_status: int) -> int:
    """
    Recompute derived status bits from a core mask.
    Derived bits:
    - bit3 handset_connected from bit8/bit9
    - bit5 peripherals_all_ok from bits6..9
    - bit0 fully_ready from bit1 and bits3..9
    """
    s = core_status
    s &= ~((1 << 0) | (1 << 3) | (1 << 5))

    if s & ((1 << 8) | (1 << 9)):
        s |= (1 << 3)

    periph_mask = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)
    if (s & periph_mask) == periph_mask:
        s |= (1 << 5)

    required_mask = (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)
    if (s & required_mask) == required_mask:
        s |= (1 << 0)

    return s


def compose_buttons24(status_bits: int, buttons_word: int) -> int:
    """
    24-button map (LSB first):
    [7:0]   = status[7:0]
    [15:8]  = P[7:0]
    [16]    = torso_R
    [17]    = torso_L
    [18]    = pedal
    [23:19] = reserved
    """
    status8 = status_bits & 0xFF
    pcf8 = buttons_word & 0xFF
    torso_l = 1 if (buttons_word & (1 << 9)) else 0
    torso_r = 1 if (buttons_word & (1 << 10)) else 0
    pedal = 1 if (buttons_word & (1 << 11)) else 0

    buttons24 = status8
    buttons24 |= (pcf8 << 8)
    buttons24 |= (torso_r << 16)
    buttons24 |= (torso_l << 17)
    buttons24 |= (pedal << 18)
    return buttons24


def pack_report1(counter: int, buttons24: int, axis_vals: tuple[int, int, int, int, int, int]) -> bytearray:
    """Pack report ID 1 payload (16 bytes)."""
    r1_buf[0] = counter & 0xFF
    r1_buf[1] = buttons24 & 0xFF
    r1_buf[2] = (buttons24 >> 8) & 0xFF
    r1_buf[3] = (buttons24 >> 16) & 0xFF
    struct.pack_into("<6h", r1_buf, 4, *axis_vals)
    return r1_buf


def pack_report2(counter_q: int, quat_stat: int, qvals: tuple[int, int, int, int]) -> bytearray:
    """Pack report ID 2 payload (10 bytes)."""
    r2_buf[0] = counter_q & 0xFF
    r2_buf[1] = quat_stat & 0xFF
    struct.pack_into("<4h", r2_buf, 2, *qvals)
    return r2_buf


def send_zero_packet_with_status(status_bits: int) -> None:
    """Send one report-1 packet with zero axes and current status mapped into buttons."""
    buttons24 = compose_buttons24(status_bits, 0)
    payload = pack_report1(0, buttons24, (0, 0, 0, 0, 0, 0))
    _send_input_report(RID_INPUT_MAIN, payload)


def emit_status_transition_packets(prev_status: int, new_status: int) -> None:
    """
    Emit status-only packets for a stopped-state transition.
    If multiple peripheral OK bits (6..9) rise together, emit progressive
    intermediate packets so host sees accumulation.
    """
    if new_status == prev_status:
        return

    periph_bits = [6, 7, 8, 9]
    rising = [b for b in periph_bits if (new_status & (1 << b)) and not (prev_status & (1 << b))]

    target_core = new_status & ~((1 << 0) | (1 << 3) | (1 << 5))
    curr_core = (target_core & ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9))) | (prev_status & ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)))

    if rising:
        for b in periph_bits:
            if b in rising:
                curr_core |= (1 << b)
                send_zero_packet_with_status(_compose_status_from_core(curr_core))
        return

    send_zero_packet_with_status(new_status)


def poll_out_and_apply() -> None:
    """Parse latest OUT report and apply START/STOP + sample-rate changes."""
    global running, report_hz, last_out, next_tick_r1_ns, next_tick_r2_ns

    rb = _read_out_report()
    if rb is None:
        return

    if rb == last_out:
        return
    last_out = rb

    decoded = _decode_out_cmd(rb)
    if decoded is None:
        return

    cmd, sr = decoded
    print("OUT raw:", list(rb), "cmd:", cmd, "sr16:", sr)

    if cmd == CMD_START:
        report_hz = clamp_sr(sr)
        running = True
        now_ns = time.monotonic_ns()
        next_tick_r1_ns = now_ns
        next_tick_r2_ns = now_ns
        print("APPLY START: running=True report_hz=", report_hz)
    elif cmd == CMD_STOP:
        running = False
        print("APPLY STOP: running=False (report_hz stays=", report_hz, ")")
    else:
        print("APPLY: unknown cmd", cmd, "(ignored)")


# -----------------------------
# Hardware init
# -----------------------------

print("BOOT: code.py starting")
try:
    print("BOOT: init gpio")
    pins = init_gpio_inputs()
    print("BOOT: init status LEDs")
    init_status_leds()
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

    dev = find_hid_device(usb_hid, usage_page=0x01, usage=0x0004)
    if dev is None:
        raise RuntimeError("Joystick HID endpoint not found")

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


# Post-init status packet: all-zero payload with boot_ok (bit1) high.
send_zero_packet_with_status(1 << 1)
last_status_bits = (1 << 1)


# -----------------------------
# Main loop
# -----------------------------

while True:
    try:
        poll_out_and_apply()

        readout.update_i2c_presence(period_s=1.0)
        load16 = readout.update_nau_load16()

        now_ns = time.monotonic_ns()
        roll16 = readout.update_lsm_roll16(now_ns)
        buttons_word, pedal_connected = readout.sample_buttons(now_ns)
        status_bits = readout.make_status_bits(running, pedal_connected)
        update_status_leds(time.monotonic(), status_bits)

        if status_bits != last_status_bits:
            prev_status = last_status_bits
            last_status_bits = status_bits
            if not running:
                emit_status_transition_packets(prev_status, status_bits)

        if running:
            report1_hz = report_hz
            report2_hz = report_hz if report_hz <= 250 else 250
            period1_ns = int(1_000_000_000 // report1_hz)
            period2_ns = int(1_000_000_000 // report2_hz)

            due_r2 = 0
            while now_ns >= next_tick_r2_ns:
                next_tick_r2_ns += period2_ns
                due_r2 += 1

            if due_r2 > 0:
                qx, qy, qz, qw, quat_stat = readout.update_bno_quat_q14(now_ns)
                r2_counter = (r2_counter + 1) & 0xFF
                payload2 = pack_report2(r2_counter, quat_stat, (qx, qy, qz, qw))
                _send_input_report(RID_INPUT_QUAT, payload2)

            due_r1 = 0
            while now_ns >= next_tick_r1_ns and due_r1 < MAX_R1_BURST:
                next_tick_r1_ns += period1_ns
                due_r1 += 1

            if due_r1 > 0:
                knob1_u16 = adcs[0].value if adcs[0] is not None else 0
                knob2_u16 = adcs[1].value if adcs[1] is not None else 0
                slider1_u16 = adcs[2].value if adcs[2] is not None else 0
                slider2_raw = adcs[3].value if adcs[3] is not None else 0
                slider2_u16 = slider2_transform(slider2_raw)

                buttons24 = compose_buttons24(status_bits, buttons_word)
                axis_vals = (
                    u16_to_i16(knob1_u16),
                    u16_to_i16(knob2_u16),
                    int(load16),
                    int(roll16),
                    u16_to_i16(slider1_u16),
                    u16_to_i16(slider2_u16),
                )

                for _ in range(due_r1):
                    r1_counter = (r1_counter + 1) & 0xFF
                    payload1 = pack_report1(r1_counter, buttons24, axis_vals)
                    _send_input_report(RID_INPUT_MAIN, payload1)

        diag(f"alive t={time.monotonic():.1f} usb={supervisor.runtime.usb_connected}")
        time.sleep(0)

    except Exception as e:
        errcount = (errcount + 1) & 0xFFFF
        print("EXC:", repr(e))
        time.sleep(0.01)
