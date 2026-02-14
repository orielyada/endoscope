import usb_hid
import usb_cdc
import supervisor

supervisor.set_usb_identification(
    manufacturer="OriElyada",
    product="Endoscope",
)

# Ensure CDC serial is enabled
usb_cdc.enable(console=True, data=True)

REPORT_DESC = bytes((
    0x06, 0x00, 0xFF,
    0x09, 0x01,
    0xA1, 0x01,

    0x09, 0x02,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x20,
    0x81, 0x02,

    0x09, 0x03,
    0x75, 0x08,
    0x95, 0x03,
    0x91, 0x02,

    0xC0
))

endoscope_hid = usb_hid.Device(
    report_descriptor=REPORT_DESC,
    usage_page=0xFF00,
    usage=0x0001,
    report_ids=(0,),
    in_report_lengths=(32,),
    out_report_lengths=(3,),
)

usb_hid.enable((endoscope_hid,), boot_device=False)
