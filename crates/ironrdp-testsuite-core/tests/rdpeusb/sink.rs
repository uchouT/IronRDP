use ironrdp_core::{decode, encode_vec};
use ironrdp_rdpeusb::pdu::sink::{
    DeviceSpeed, NoAckIsochWriteJitterBufSizeInMs, SupportedUsbVer, UsbBusIfaceVer, UsbDeviceCaps, UsbdiVer,
};
use rstest::rstest;

fn caps(supported_usb_ver: SupportedUsbVer) -> UsbDeviceCaps {
    UsbDeviceCaps {
        usb_bus_iface_ver: UsbBusIfaceVer::V2,
        usbdi_ver: UsbdiVer::V0X600,
        supported_usb_ver,
        device_speed: DeviceSpeed::HIGH_SPEED,
        no_ack_isoch_write_jitter_buf_size: NoAckIsochWriteJitterBufSizeInMs::try_from(0).unwrap(),
    }
}

/// Every `Supported_USB_Version` value round-trips through encode/decode — the
/// named ones (USB 1.0–3.2) and an unnamed device-reported value alike. A real
/// USB 3.2 device reporting `0x320` used to be rejected during decode (tearing
/// down the URBDRC channel); it must now decode and round-trip like the rest.
#[rstest]
#[case(SupportedUsbVer::USB_10)]
#[case(SupportedUsbVer::USB_11)]
#[case(SupportedUsbVer::USB_20)]
#[case(SupportedUsbVer::USB_30)]
#[case(SupportedUsbVer::USB_31)]
#[case(SupportedUsbVer::USB_32)]
#[case(SupportedUsbVer::from_u32(0x9999))]
fn capabilities_round_trip(#[case] supported_usb_ver: SupportedUsbVer) {
    let original = caps(supported_usb_ver);
    let encoded = encode_vec(&original).expect("encode should succeed");
    let decoded: UsbDeviceCaps = decode(&encoded).expect("capabilities should decode");
    assert_eq!(decoded.supported_usb_ver, supported_usb_ver);
    assert_eq!(original, decoded);
}

/// Decode straight from the raw 28-byte USB_DEVICE_CAPABILITIES a real USB 3.2
/// device sends, with `Supported_USB_Version = 0x320` at its wire offset (12).
/// This reproduces the original decode failure independently of the crate's own
/// encode — a compensating encode bug would slip past a round-trip test — and
/// pins the field offset against a hand-authored buffer.
#[test]
fn usb3_capabilities_decode_from_raw_bytes() {
    #[rustfmt::skip]
    let raw: [u8; 28] = [
        0x1c, 0x00, 0x00, 0x00, // CbSize = 28
        0x02, 0x00, 0x00, 0x00, // UsbBusInterfaceVersion = 2
        0x00, 0x06, 0x00, 0x00, // USBDI_Version = 0x600
        0x20, 0x03, 0x00, 0x00, // Supported_USB_Version = 0x320  (offset 12)
        0x00, 0x00, 0x00, 0x00, // HcdCapabilities = 0
        0x01, 0x00, 0x00, 0x00, // DeviceIsHighSpeed = 1
        0x00, 0x00, 0x00, 0x00, // NoAckIsochWriteJitterBufferSizeInMs = 0
    ];

    let decoded: UsbDeviceCaps = decode(&raw).expect("USB 3.2 capabilities should decode from raw bytes");
    assert_eq!(decoded.supported_usb_ver, SupportedUsbVer::USB_32);
    assert_eq!(decoded.usb_bus_iface_ver, UsbBusIfaceVer::V2);
    assert_eq!(decoded.usbdi_ver, UsbdiVer::V0X600);
    assert_eq!(decoded.device_speed, DeviceSpeed::HIGH_SPEED);
}
