import time
import struct
import usb_hid
import supervisor

DIAG = True
_last_diag = 0.0

def diag(msg, period_s=0.5):
    global _last_diag
    if not DIAG:
        return
    now = time.monotonic()
    if now - _last_diag >= period_s:
        _last_diag = now
        print(msg)


dev = None
for d in usb_hid.devices:
    if d.usage_page == 0xFF00 and d.usage == 0x0001:
        dev = d
        break
if dev is None:
    while True:
        time.sleep(1)

FMT_IN = "<HhhhhHHHHhHHH6s"
in_buf = bytearray(32)

CMD_START = 0x01
CMD_STOP  = 0x02

def clamp_sr(hz: int) -> int:
    if hz == 0:
        return 500
    if hz < 1:
        return 1
    if hz > 1000:
        return 1000
    return int(hz)

seq = 0
running = False
report_hz = 100  # start slow, easier to observe
next_tick_ns = time.monotonic_ns()

# placeholders
qw, qx, qy, qz = 32767, 0, 0, 0
knob0 = knob1 = slider0 = slider1 = 0
load16 = 0
roll16 = 0
buttons = 0

# Debug fields in last 6 bytes:
# [magic, errcount_lo, errcount_hi, out0, out1, out2]
magic = 0xA5
errcount = 0
out3 = b"\x00\x00\x00"

last_out = None

def make_status(running_flag: bool) -> int:
    # signature for now
    return 0x5678 if running_flag else 0x1234

def poll_out_and_apply():
    global running, report_hz, last_out, out3
    r = dev.get_last_received_report()
    if not r:
        return
    rb = bytes(r)
    if rb == last_out:
        return
    last_out = rb

    # capture first 3 payload bytes (best-effort)
    # if report-id 0 present, skip it
    if len(rb) >= 4 and rb[0] == 0:
        out3 = bytes(rb[1:4])
        cmd = rb[1]
        sr = rb[2] | (rb[3] << 8)
    elif len(rb) >= 3:
        out3 = bytes(rb[0:3])
        cmd = rb[0]
        sr = rb[1] | (rb[2] << 8)
    else:
        out3 = (rb + b"\x00\x00\x00")[:3]
        return

    if cmd == CMD_START:
        report_hz = clamp_sr(sr)
        running = True
    elif cmd == CMD_STOP:
        running = False

while True:
    try:
        poll_out_and_apply()

        now_ns = time.monotonic_ns()
        period_ns = int(1_000_000_000 // report_hz)

        if now_ns >= next_tick_ns:
            next_tick_ns = now_ns + period_ns
            seq = (seq + 1) & 0xFFFF

            status = make_status(running)

            tail = bytes((
                magic,
                errcount & 0xFF,
                (errcount >> 8) & 0xFF,
                out3[0], out3[1], out3[2],
            ))

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

            dev.send_report(in_buf)

        diag(f"alive t={time.monotonic():.1f} usb={supervisor.runtime.usb_connected}")

        time.sleep(0)

    except Exception as e:
        # don't die; just count errors and keep going
        errcount = (errcount + 1) & 0xFFFF
        time.sleep(0.01)
