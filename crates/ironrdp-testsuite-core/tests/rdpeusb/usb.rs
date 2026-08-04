use ironrdp_rdpeusb::{
    io::{TsUrbInKind, TsUrbOutKind, UrbFunction},
    usb::{
        ConversionError, TransferRequest, bulk_or_interrupt_in, bulk_or_interrupt_out, control_transfer,
        get_descriptor, select_configuration, select_interface,
    },
};
use ironrdp_usb::{
    control::{GetDescriptorRequest, Recipient, RequestKind, RequestType, SetupPacket, standard_request},
    descriptor::{ConfigurationDescriptorSet, descriptor_type},
    value::{Direction, InterfaceSelection, UsbSpeed},
};

const USBD_TRANSFER_DIRECTION_IN: u32 = 0x0000_0001;
const USBD_SHORT_TRANSFER_OK: u32 = 0x0000_0002;
const USBD_DEFAULT_PIPE_TRANSFER: u32 = 0x0000_0008;
const DEFAULT_CONTROL_PIPE_HANDLE: u32 = 0;

const CONFIGURATION: [u8; 57] = [
    9, 2, 57, 0, 2, 1, 0, 0x80, 50, // configuration
    9, 4, 0, 0, 2, 0xff, 0, 0, 0, // interface 0, alternate 0
    7, 5, 0x81, 3, 0x40, 0x10, 1, // high-bandwidth interrupt IN
    7, 5, 0x02, 2, 0x00, 0x02, 0, // bulk OUT
    9, 4, 1, 0, 0, 0xff, 0, 0, 0, // interface 1, alternate 0
    9, 4, 1, 1, 1, 0xff, 0, 0, 0, // interface 1, alternate 1
    7, 5, 0x83, 3, 32, 0, 4, // interrupt IN
];

fn setup(
    direction: Direction,
    kind: RequestKind,
    recipient: Recipient,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
) -> SetupPacket {
    SetupPacket {
        request_type: RequestType::new(direction, kind, recipient),
        request,
        value,
        index,
        length,
    }
}

fn selection(interface: u8, alternate_setting: u8) -> InterfaceSelection {
    InterfaceSelection {
        interface,
        alternate_setting,
    }
}

#[test]
fn feature_out_uses_rdpeusb_transfer_in() {
    let setup = setup(
        Direction::Out,
        RequestKind::STANDARD,
        Recipient::ENDPOINT,
        standard_request::SET_FEATURE,
        0,
        0x81,
        0,
    );

    let TransferRequest::In(request) = control_transfer(setup, Vec::new()).unwrap() else {
        panic!("feature request used TRANSFER_OUT_REQUEST");
    };
    assert_eq!(request.output_buffer_size, 0);
    assert_eq!(request.ts_urb.func, UrbFunction::URB_FUNCTION_SET_FEATURE_TO_ENDPOINT);
    let TsUrbInKind::CtlFeatReq(urb) = request.ts_urb.kind else {
        panic!("feature request used the wrong TS_URB variant");
    };
    assert_eq!(urb.feat_selector, 0);
    assert_eq!(urb.index, 0x81);
}

#[test]
fn descriptor_request_preserves_typed_fields() {
    let setup = setup(
        Direction::In,
        RequestKind::STANDARD,
        Recipient::INTERFACE,
        standard_request::GET_DESCRIPTOR,
        0x2203,
        0x0409,
        64,
    );

    let TransferRequest::In(request) = control_transfer(setup, Vec::new()).unwrap() else {
        panic!("GET_DESCRIPTOR used TRANSFER_OUT_REQUEST");
    };
    assert_eq!(request.output_buffer_size, 64);
    assert_eq!(
        request.ts_urb.func,
        UrbFunction::URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE
    );
    let TsUrbInKind::CtlDescReq(urb) = request.ts_urb.kind else {
        panic!("GET_DESCRIPTOR used the wrong TS_URB variant");
    };
    assert_eq!(urb.index, 3);
    assert_eq!(urb.desc_type, 0x22);
    assert_eq!(urb.lang_id, 0x0409);
}

#[test]
fn typed_get_descriptor_rejects_an_unrepresentable_recipient() {
    let error = get_descriptor(GetDescriptorRequest {
        recipient: Recipient::VENDOR_SPECIFIC,
        descriptor_type: descriptor_type::DEVICE,
        descriptor_index: 0,
        index: 0,
        requested_length: 18,
    })
    .unwrap_err();

    assert_eq!(
        error,
        ConversionError::UnsupportedDescriptorRecipient {
            recipient: Recipient::VENDOR_SPECIFIC
        }
    );
}

#[test]
fn class_in_accepts_short_control_responses() {
    let setup = SetupPacket {
        request_type: RequestType::new(Direction::In, RequestKind::CLASS, Recipient::INTERFACE),
        request: 0x81,
        value: 0x0200,
        index: 3,
        length: 8,
    };

    let TransferRequest::In(request) = control_transfer(setup, Vec::new()).unwrap() else {
        panic!("class IN request used TRANSFER_OUT_REQUEST");
    };
    let TsUrbInKind::VendorClassReq(urb) = request.ts_urb.kind else {
        panic!("class request used the wrong TS_URB variant");
    };
    assert_eq!(urb.transfer_flags, USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK);
}

#[test]
fn unknown_recipient_falls_back_without_losing_setup_fields() {
    let setup = SetupPacket {
        request_type: RequestType::new(Direction::In, RequestKind::VENDOR, Recipient::VENDOR_SPECIFIC),
        request: 0xa5,
        value: 0x1234,
        index: 0x5678,
        length: 17,
    };

    let TransferRequest::In(request) = control_transfer(setup, Vec::new()).unwrap() else {
        panic!("vendor IN request used TRANSFER_OUT_REQUEST");
    };
    assert_eq!(request.output_buffer_size, 17);
    assert_eq!(request.ts_urb.func, UrbFunction::URB_FUNCTION_CONTROL_TRANSFER);
    let TsUrbInKind::CtlTransfer(urb) = request.ts_urb.kind else {
        panic!("unknown recipient did not use generic control transfer");
    };
    assert_eq!(urb.pipe, DEFAULT_CONTROL_PIPE_HANDLE);
    assert_eq!(
        urb.transfer_flags,
        USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK | USBD_DEFAULT_PIPE_TRANSFER
    );
    assert_eq!(urb.setup_packet.request_type, setup.request_type.raw());
    assert_eq!(urb.setup_packet.request, setup.request);
    assert_eq!(urb.setup_packet.value, setup.value);
    assert_eq!(urb.setup_packet.index, setup.index);
    assert_eq!(urb.setup_packet.length, setup.length);
}

#[test]
fn noncanonical_typed_requests_fall_back_without_losing_fields() {
    let requests = [
        setup(
            Direction::In,
            RequestKind::STANDARD,
            Recipient::DEVICE,
            standard_request::GET_STATUS,
            1,
            0,
            2,
        ),
        setup(
            Direction::In,
            RequestKind::STANDARD,
            Recipient::DEVICE,
            standard_request::GET_CONFIGURATION,
            0,
            1,
            1,
        ),
        setup(
            Direction::In,
            RequestKind::STANDARD,
            Recipient::INTERFACE,
            standard_request::GET_INTERFACE,
            1,
            3,
            1,
        ),
        setup(
            Direction::In,
            RequestKind::STANDARD,
            Recipient::ENDPOINT,
            standard_request::CLEAR_FEATURE,
            0,
            0x81,
            0,
        ),
    ];

    for setup in requests {
        let TransferRequest::In(request) = control_transfer(setup, Vec::new()).unwrap() else {
            panic!("noncanonical USB IN request used TRANSFER_OUT_REQUEST");
        };
        let TsUrbInKind::CtlTransfer(urb) = request.ts_urb.kind else {
            panic!("noncanonical request lost fields in a typed TS_URB");
        };
        assert_eq!(urb.setup_packet.request_type, setup.request_type.raw());
        assert_eq!(urb.setup_packet.request, setup.request);
        assert_eq!(urb.setup_packet.value, setup.value);
        assert_eq!(urb.setup_packet.index, setup.index);
        assert_eq!(urb.setup_packet.length, setup.length);
    }
}

#[test]
fn stateful_standard_requests_do_not_use_generic_control() {
    let requests = [
        setup(
            Direction::Out,
            RequestKind::STANDARD,
            Recipient::DEVICE,
            standard_request::SET_ADDRESS,
            5,
            0,
            0,
        ),
        setup(
            Direction::Out,
            RequestKind::STANDARD,
            Recipient::DEVICE,
            standard_request::SET_CONFIGURATION,
            1,
            0,
            0,
        ),
        setup(
            Direction::Out,
            RequestKind::STANDARD,
            Recipient::INTERFACE,
            standard_request::SET_INTERFACE,
            1,
            0,
            0,
        ),
    ];

    for setup in requests {
        assert_eq!(
            control_transfer(setup, Vec::new()).unwrap_err(),
            ConversionError::StatefulStandardRequest { request: setup.request }
        );
    }
}

#[test]
fn control_data_shape_is_validated_before_translation() {
    let input = setup(
        Direction::In,
        RequestKind::STANDARD,
        Recipient::DEVICE,
        standard_request::GET_STATUS,
        0,
        0,
        2,
    );
    assert_eq!(
        control_transfer(input, vec![0]).unwrap_err(),
        ConversionError::InTransferHasData { actual: 1 }
    );

    let output = SetupPacket {
        request_type: RequestType::new(Direction::Out, RequestKind::VENDOR, Recipient::DEVICE),
        request: 1,
        value: 0,
        index: 0,
        length: 2,
    };
    assert_eq!(
        control_transfer(output, vec![0]).unwrap_err(),
        ConversionError::OutTransferLengthMismatch { expected: 2, actual: 1 }
    );
}

#[test]
fn bulk_builders_own_direction_and_buffer_shape() {
    let input = bulk_or_interrupt_in(7, 4096);
    assert_eq!(input.output_buffer_size, 4096);
    let TsUrbInKind::BulkInterruptTransfer(urb) = input.ts_urb.kind else {
        panic!("bulk IN used the wrong TS_URB variant");
    };
    assert_eq!(urb.pipe_handle, 7);
    assert_eq!(urb.transfer_flags, USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK);

    let output = bulk_or_interrupt_out(9, vec![1, 2, 3]);
    assert_eq!(output.output_buffer, [1, 2, 3]);
    assert!(!output.ts_urb.no_ack);
    let TsUrbOutKind::BulkInterruptTransfer(urb) = output.ts_urb.kind else {
        panic!("bulk OUT used the wrong TS_URB variant");
    };
    assert_eq!(urb.pipe_handle, 9);
    assert_eq!(urb.transfer_flags, 0);
}

#[test]
fn select_configuration_carries_full_descriptor_and_selected_interfaces() {
    let descriptor = ConfigurationDescriptorSet::parse(&CONFIGURATION).unwrap();
    descriptor.validate().unwrap();
    let selections = [selection(0, 0), selection(1, 1)];

    let request = select_configuration(Some(descriptor), &selections, UsbSpeed::High).unwrap();
    assert_eq!(request.output_buffer_size, 0);
    assert_eq!(request.ts_urb.func, UrbFunction::URB_FUNCTION_SELECT_CONFIGURATION);
    let TsUrbInKind::SelectConfig(urb) = request.ts_urb.kind else {
        panic!("selection used the wrong TS_URB variant");
    };
    let config = urb.desc.expect("configuration descriptor is present");
    assert_eq!(config.total_length, CONFIGURATION.len() as u16);
    assert_eq!(config.trailing, CONFIGURATION[9..]);
    assert_eq!(urb.usbd_ifaces.len(), 2);
    assert_eq!(urb.usbd_ifaces[0].interface_number, 0);
    assert_eq!(urb.usbd_ifaces[0].alternate_setting, 0);
    assert_eq!(urb.usbd_ifaces[0].ts_usbd_pipe_info.len(), 2);
    assert_eq!(urb.usbd_ifaces[0].ts_usbd_pipe_info[0].max_packet_size, 192);
    assert_eq!(urb.usbd_ifaces[0].ts_usbd_pipe_info[1].max_packet_size, 512);
    assert_eq!(urb.usbd_ifaces[1].interface_number, 1);
    assert_eq!(urb.usbd_ifaces[1].alternate_setting, 1);
}

#[test]
fn select_interface_uses_the_resolved_interface_descriptor() {
    let descriptor = ConfigurationDescriptorSet::parse(&CONFIGURATION).unwrap();
    let interface = descriptor.interface(1, 1).unwrap();

    let request = select_interface(42, interface, UsbSpeed::High).unwrap();
    assert_eq!(request.output_buffer_size, 0);
    assert_eq!(request.ts_urb.func, UrbFunction::URB_FUNCTION_SELECT_INTERFACE);
    let TsUrbInKind::SelectIface(urb) = request.ts_urb.kind else {
        panic!("interface selection used the wrong TS_URB variant");
    };
    assert_eq!(urb.config_handle, 42);
    assert_eq!(urb.usbd_iface.interface_number, 1);
    assert_eq!(urb.usbd_iface.alternate_setting, 1);
    assert_eq!(urb.usbd_iface.ts_usbd_pipe_info[0].max_packet_size, 32);
}

#[test]
fn selection_rejects_inconsistent_interface_sets() {
    let descriptor = ConfigurationDescriptorSet::parse(&CONFIGURATION).unwrap();
    assert_eq!(
        select_configuration(None, &[selection(0, 0)], UsbSpeed::Full).unwrap_err(),
        ConversionError::UnconfiguredDeviceHasInterfaces
    );
    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(0, 0)], UsbSpeed::High).unwrap_err(),
        ConversionError::DuplicateInterface { interface: 0 }
    );
    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0)], UsbSpeed::High).unwrap_err(),
        ConversionError::MissingInterfaceSelection { interface: 1 }
    );
    let missing = selection(7, 0);
    assert_eq!(
        select_configuration(Some(descriptor), &[missing], UsbSpeed::Full).unwrap_err(),
        ConversionError::InterfaceNotFound { selection: missing }
    );
}

#[test]
fn unconfigure_has_an_empty_transfer_in_request() {
    let request = select_configuration(None, &[], UsbSpeed::Full).unwrap();
    assert_eq!(request.output_buffer_size, 0);
    let TsUrbInKind::SelectConfig(urb) = request.ts_urb.kind else {
        panic!("unconfigure used the wrong TS_URB variant");
    };
    assert!(urb.desc.is_none());
    assert!(urb.usbd_ifaces.is_empty());
}

#[test]
fn selection_rejects_reserved_high_bandwidth_encoding() {
    let mut bytes = CONFIGURATION;
    bytes[22] = 0x40;
    bytes[23] = 0x18;
    let descriptor = ConfigurationDescriptorSet::parse(&bytes).unwrap();

    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(1, 1)], UsbSpeed::High).unwrap_err(),
        ConversionError::InvalidMaximumPacketSize {
            selection: selection(0, 0),
            raw: 0x1840
        }
    );
}

#[test]
fn selection_rejects_high_bandwidth_bits_at_other_speeds() {
    let mut bytes = CONFIGURATION;
    bytes[22] = 0x40;
    bytes[23] = 0x08;
    let descriptor = ConfigurationDescriptorSet::parse(&bytes).unwrap();

    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(1, 1)], UsbSpeed::Full).unwrap_err(),
        ConversionError::InvalidMaximumPacketSize {
            selection: selection(0, 0),
            raw: 0x0840
        }
    );
}

#[test]
fn selection_rejects_descriptor_count_mismatches() {
    let mut interface_count = CONFIGURATION;
    interface_count[4] = 3;
    let descriptor = ConfigurationDescriptorSet::parse(&interface_count).unwrap();
    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(1, 1)], UsbSpeed::Full).unwrap_err(),
        ConversionError::InterfaceCountMismatch { declared: 3, actual: 2 }
    );

    let mut endpoint_count = CONFIGURATION;
    endpoint_count[13] = 1;
    let descriptor = ConfigurationDescriptorSet::parse(&endpoint_count).unwrap();
    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(1, 1)], UsbSpeed::Full).unwrap_err(),
        ConversionError::EndpointCountMismatch {
            selection: selection(0, 0),
            declared: 1,
            actual: 2
        }
    );
}

#[test]
fn selection_rejects_more_than_thirty_pipes() {
    let total_length = 9 + 9 + 31 * 7;
    let mut bytes = vec![
        9,
        2,
        total_length as u8,
        (total_length >> 8) as u8,
        1,
        1,
        0,
        0x80,
        50,
        9,
        4,
        0,
        0,
        31,
        0xff,
        0,
        0,
        0,
    ];
    for endpoint in 0..31 {
        bytes.extend_from_slice(&[7, 5, (endpoint % 15 + 1) as u8, 2, 64, 0, 0]);
    }
    let descriptor = ConfigurationDescriptorSet::parse(&bytes).unwrap();

    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0)], UsbSpeed::Full).unwrap_err(),
        ConversionError::TooManyPipes {
            selection: selection(0, 0),
            actual: 31
        }
    );
}

#[test]
fn selection_rejects_more_than_thirty_active_pipes() {
    let total_length = 9 + 2 * 9 + 32 * 7;
    let mut bytes = vec![9, 2, total_length as u8, (total_length >> 8) as u8, 2, 1, 0, 0x80, 50];
    for interface in 0..2 {
        bytes.extend_from_slice(&[9, 4, interface, 0, 16, 0xff, 0, 0, 0]);
        for endpoint in 0..16 {
            bytes.extend_from_slice(&[7, 5, endpoint % 15 + 1, 2, 64, 0, 0]);
        }
    }
    let descriptor = ConfigurationDescriptorSet::parse(&bytes).unwrap();

    assert_eq!(
        select_configuration(Some(descriptor), &[selection(0, 0), selection(1, 0)], UsbSpeed::Full).unwrap_err(),
        ConversionError::TooManyActivePipes { actual: 32 }
    );
}
