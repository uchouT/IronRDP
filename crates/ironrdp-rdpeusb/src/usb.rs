//! Translation from protocol-independent USB semantics to RDPEUSB requests.
//!
//! This module is deliberately sans-I/O. It does not know about usbredir
//! packets, RDPEUSB request IDs, device state, completion routing, or async
//! execution. Each function returns a complete backend-facing RDPEUSB transfer
//! packet so that the TS_URB payload, URB function, transfer envelope, flags,
//! and buffer shape cannot disagree.
//!
//! The adapter belongs to RDPEUSB because it selects TS_URB forms and
//! Windows USBD conventions; the input types remain protocol-independent.

use alloc::vec::Vec;
use core::fmt;

use crate::{
    io::{TransferInPacket, TransferOutPacket, TsUrbInKind, TsUrbInPacket, TsUrbOutKind, TsUrbOutPacket, UrbFunction},
    pdu::{
        usb_dev::ts_urb::{
            TsUrbBulkOrInterruptTransfer, TsUrbControlDescRequest, TsUrbControlFeatRequest,
            TsUrbControlGetConfigRequest, TsUrbControlGetInterfaceRequest, TsUrbControlGetStatusRequest,
            TsUrbControlTransfer, TsUrbControlVendorClassRequest, TsUrbSelectConfig, TsUrbSelectInterface,
            utils::{SetupPacket as RdpeusbSetupPacket, TsUsbdInterfaceInfo, TsUsbdPipeInfo, UsbConfigDesc},
        },
        utils::{ConfigHandle, MAX_NON_DEFAULT_EP_COUNT, PipeHandle},
    },
};
use ironrdp_usb::{
    control::{GetDescriptorRequest, Recipient, RequestKind, SetupPacket, standard_request},
    descriptor::{ConfigurationDescriptorSet, InterfaceDescriptor},
    value::{Direction, InterfaceSelection, TransferType, UsbSpeed},
};

// WDK USBD transfer flags. RDPEUSB carries these values unchanged in TS_URB.
const USBD_TRANSFER_DIRECTION_IN: u32 = 0x0000_0001;
const USBD_SHORT_TRANSFER_OK: u32 = 0x0000_0002;
const USBD_DEFAULT_PIPE_TRANSFER: u32 = 0x0000_0008;

// A Windows URB addresses the default control endpoint with a null pipe
// handle and USBD_DEFAULT_PIPE_TRANSFER. This is a Windows URB convention,
// rather than an RDPEUSB-assigned pipe handle.
const DEFAULT_CONTROL_PIPE_HANDLE: PipeHandle = 0;

/// A complete RDPEUSB transfer request, before request-ID allocation and I/O.
#[derive(Debug, Clone)]
pub enum TransferRequest {
    /// An RDPEUSB `TRANSFER_IN_REQUEST` envelope.
    In(TransferInPacket),
    /// An RDPEUSB `TRANSFER_OUT_REQUEST` envelope.
    Out(TransferOutPacket),
}

/// A USB operation that cannot be represented by the requested RDPEUSB form.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ConversionError {
    InTransferHasData {
        actual: usize,
    },
    OutTransferLengthMismatch {
        expected: usize,
        actual: usize,
    },
    UnsupportedDescriptorRecipient {
        recipient: Recipient,
    },
    StatefulStandardRequest {
        request: u8,
    },
    UnconfiguredDeviceHasInterfaces,
    InvalidConfigurationHeaderLength {
        actual: u8,
    },
    InterfaceCountMismatch {
        declared: u8,
        actual: usize,
    },
    DuplicateInterface {
        interface: u8,
    },
    InterfaceNotFound {
        selection: InterfaceSelection,
    },
    MissingInterfaceSelection {
        interface: u8,
    },
    TooManyPipes {
        selection: InterfaceSelection,
        actual: usize,
    },
    TooManyActivePipes {
        actual: usize,
    },
    EndpointCountMismatch {
        selection: InterfaceSelection,
        declared: u8,
        actual: usize,
    },
    InvalidMaximumPacketSize {
        selection: InterfaceSelection,
        raw: u16,
    },
}

impl fmt::Display for ConversionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InTransferHasData { actual } => {
                write!(f, "USB IN transfer carries {actual} bytes of output data")
            }
            Self::OutTransferLengthMismatch { expected, actual } => write!(
                f,
                "USB OUT transfer declares {expected} bytes but carries {actual} bytes"
            ),
            Self::UnsupportedDescriptorRecipient { recipient } => write!(
                f,
                "RDPEUSB has no typed descriptor request for USB recipient {}",
                recipient.raw()
            ),
            Self::StatefulStandardRequest { request } => write!(
                f,
                "USB standard request {:#04x} requires host-controller state and cannot use a generic RDPEUSB control transfer",
                request
            ),
            Self::UnconfiguredDeviceHasInterfaces => {
                f.write_str("an unconfigured USB device cannot have selected interfaces")
            }
            Self::InvalidConfigurationHeaderLength { actual } => write!(
                f,
                "RDPEUSB requires a 9-byte USB configuration header, got {actual} bytes"
            ),
            Self::InterfaceCountMismatch { declared, actual } => write!(
                f,
                "USB configuration declares {declared} interfaces but contains {actual}"
            ),
            Self::DuplicateInterface { interface } => {
                write!(f, "USB interface {interface} is selected more than once")
            }
            Self::InterfaceNotFound { selection } => write!(
                f,
                "USB interface {} alternate setting {} is absent from the configuration",
                selection.interface, selection.alternate_setting
            ),
            Self::MissingInterfaceSelection { interface } => write!(
                f,
                "USB configuration has no selected alternate setting for interface {}",
                interface
            ),
            Self::TooManyPipes { selection, actual } => write!(
                f,
                "USB interface {} alternate setting {} has {actual} pipes; RDPEUSB supports at most {MAX_NON_DEFAULT_EP_COUNT}",
                selection.interface, selection.alternate_setting
            ),
            Self::TooManyActivePipes { actual } => write!(
                f,
                "USB configuration selects {actual} pipes; a USB device supports at most {MAX_NON_DEFAULT_EP_COUNT} non-default endpoints"
            ),
            Self::EndpointCountMismatch {
                selection,
                declared,
                actual,
            } => write!(
                f,
                "USB interface {} alternate setting {} declares {declared} endpoints but contains {actual}",
                selection.interface, selection.alternate_setting
            ),
            Self::InvalidMaximumPacketSize { selection, raw } => write!(
                f,
                "USB interface {} alternate setting {} has invalid wMaxPacketSize {raw:#06x}",
                selection.interface, selection.alternate_setting
            ),
        }
    }
}

/// Build a typed RDPEUSB GET_DESCRIPTOR request.
///
/// RDPEUSB only defines typed descriptor URBs for device, interface, and
/// endpoint recipients. [`control_transfer`] falls back to a generic control
/// URB for other recipients, while this explicitly typed helper returns an
/// error.
pub fn get_descriptor(request: GetDescriptorRequest) -> Result<TransferInPacket, ConversionError> {
    let func = descriptor_function(request.recipient, DescriptorOperation::Get).ok_or(
        ConversionError::UnsupportedDescriptorRecipient {
            recipient: request.recipient,
        },
    )?;
    let urb = TsUrbControlDescRequest {
        index: request.descriptor_index,
        desc_type: request.descriptor_type,
        lang_id: request.index,
    };

    Ok(transfer_in(
        TsUrbInKind::CtlDescReq(urb),
        func,
        u32::from(request.requested_length),
    ))
}

/// Translate a default-control-pipe USB request into an RDPEUSB transfer.
///
/// Canonical standard, class, and vendor requests use the operation-specific
/// TS_URB forms from MS-RDPEUSB sections 2.2.9.9 through 2.2.9.14. Other setup
/// packets are preserved losslessly in `TS_URB_CONTROL_TRANSFER`. Requests
/// whose execution changes host-controller state must be handled by the
/// owning state layer instead of this function.
pub fn control_transfer(setup: SetupPacket, data: Vec<u8>) -> Result<TransferRequest, ConversionError> {
    validate_control_data(setup, &data)?;
    if let Some(request) = setup.standard_request() {
        if matches!(
            request,
            standard_request::SET_ADDRESS | standard_request::SET_CONFIGURATION | standard_request::SET_INTERFACE
        ) {
            return Err(ConversionError::StatefulStandardRequest { request });
        }
    }

    let data = if setup.request_type.kind() == RequestKind::STANDARD {
        match standard_control_transfer(setup, data) {
            Ok(request) => return Ok(request),
            Err(data) => data,
        }
    } else if matches!(setup.request_type.kind(), RequestKind::CLASS | RequestKind::VENDOR) {
        if let Some(func) = vendor_or_class_function(setup.request_type.kind(), setup.request_type.recipient()) {
            let urb = TsUrbControlVendorClassRequest {
                transfer_flags: in_transfer_flags(setup.request_type.direction()),
                request: setup.request,
                value: setup.value,
                index: setup.index,
            };
            return Ok(match setup.request_type.direction() {
                Direction::In => TransferRequest::In(transfer_in(
                    TsUrbInKind::VendorClassReq(urb),
                    func,
                    u32::from(setup.length),
                )),
                Direction::Out => TransferRequest::Out(transfer_out(TsUrbOutKind::VendorClassReq(urb), func, data)),
            });
        }
        data
    } else {
        data
    };

    Ok(generic_control_transfer(setup, data))
}

/// Build an RDPEUSB bulk or interrupt IN transfer.
#[must_use]
pub fn bulk_or_interrupt_in(pipe_handle: PipeHandle, requested_length: u32) -> TransferInPacket {
    transfer_in(
        TsUrbInKind::BulkInterruptTransfer(TsUrbBulkOrInterruptTransfer {
            pipe_handle,
            transfer_flags: USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK,
        }),
        UrbFunction::URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER,
        requested_length,
    )
}

/// Build an RDPEUSB bulk or interrupt OUT transfer.
#[must_use]
pub fn bulk_or_interrupt_out(pipe_handle: PipeHandle, data: Vec<u8>) -> TransferOutPacket {
    transfer_out(
        TsUrbOutKind::BulkInterruptTransfer(TsUrbBulkOrInterruptTransfer {
            pipe_handle,
            transfer_flags: 0,
        }),
        UrbFunction::URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER,
        data,
    )
}

/// Build an RDPEUSB select-configuration request.
///
/// `None` represents configuration zero. Otherwise, the complete descriptor
/// set is included, and `active_interfaces` determines the alternate setting
/// requested for every interface number. The interface array preserves caller
/// order.
pub fn select_configuration(
    descriptor: Option<ConfigurationDescriptorSet<'_>>,
    active_interfaces: &[InterfaceSelection],
    speed: UsbSpeed,
) -> Result<TransferInPacket, ConversionError> {
    let urb = match descriptor {
        None => {
            if !active_interfaces.is_empty() {
                return Err(ConversionError::UnconfiguredDeviceHasInterfaces);
            }
            TsUrbSelectConfig {
                usbd_ifaces: Vec::new(),
                desc: None,
            }
        }
        Some(descriptor) => {
            let actual_interface_count = descriptor
                .interfaces()
                .filter(|interface| {
                    !descriptor.interfaces().any(|previous| {
                        previous.offset() < interface.offset() && previous.number() == interface.number()
                    })
                })
                .count();
            let declared_interface_count = descriptor.configuration().num_interfaces();
            if actual_interface_count != usize::from(declared_interface_count) {
                return Err(ConversionError::InterfaceCountMismatch {
                    declared: declared_interface_count,
                    actual: actual_interface_count,
                });
            }

            let mut usbd_ifaces = Vec::with_capacity(active_interfaces.len());
            for (index, selection) in active_interfaces.iter().copied().enumerate() {
                if active_interfaces[..index]
                    .iter()
                    .any(|previous| previous.interface == selection.interface)
                {
                    return Err(ConversionError::DuplicateInterface {
                        interface: selection.interface,
                    });
                }
                let interface = descriptor
                    .interface(selection.interface, selection.alternate_setting)
                    .ok_or(ConversionError::InterfaceNotFound { selection })?;
                usbd_ifaces.push(interface_information(interface, speed)?);
            }
            for interface in descriptor.interfaces() {
                if !active_interfaces
                    .iter()
                    .any(|selection| selection.interface == interface.number())
                {
                    return Err(ConversionError::MissingInterfaceSelection {
                        interface: interface.number(),
                    });
                }
            }
            let active_pipe_count = usbd_ifaces
                .iter()
                .map(|interface| interface.ts_usbd_pipe_info.len())
                .sum::<usize>();
            if active_pipe_count > MAX_NON_DEFAULT_EP_COUNT {
                return Err(ConversionError::TooManyActivePipes {
                    actual: active_pipe_count,
                });
            }

            TsUrbSelectConfig {
                usbd_ifaces,
                desc: Some(configuration_descriptor(descriptor)?),
            }
        }
    };

    Ok(transfer_in(
        TsUrbInKind::SelectConfig(urb),
        UrbFunction::URB_FUNCTION_SELECT_CONFIGURATION,
        0,
    ))
}

/// Build an RDPEUSB select-interface request for one resolved descriptor.
pub fn select_interface(
    config_handle: ConfigHandle,
    interface: InterfaceDescriptor<'_>,
    speed: UsbSpeed,
) -> Result<TransferInPacket, ConversionError> {
    let urb = TsUrbSelectInterface {
        config_handle,
        usbd_iface: interface_information(interface, speed)?,
    };
    Ok(transfer_in(
        TsUrbInKind::SelectIface(urb),
        UrbFunction::URB_FUNCTION_SELECT_INTERFACE,
        0,
    ))
}

fn standard_control_transfer(setup: SetupPacket, data: Vec<u8>) -> Result<TransferRequest, Vec<u8>> {
    let Some(standard_request) = setup.standard_request() else {
        return Err(data);
    };
    let recipient = setup.request_type.recipient();
    let direction = setup.request_type.direction();

    match standard_request {
        standard_request::GET_DESCRIPTOR if setup.length == 0 || direction == Direction::In => {
            let Some(func) = descriptor_function(recipient, DescriptorOperation::Get) else {
                return Err(data);
            };
            let [index, desc_type] = setup.value.to_le_bytes();
            Ok(TransferRequest::In(transfer_in(
                TsUrbInKind::CtlDescReq(TsUrbControlDescRequest {
                    index,
                    desc_type,
                    lang_id: setup.index,
                }),
                func,
                u32::from(setup.length),
            )))
        }
        standard_request::SET_DESCRIPTOR if setup.length == 0 || direction == Direction::Out => {
            let Some(func) = descriptor_function(recipient, DescriptorOperation::Set) else {
                return Err(data);
            };
            let [index, desc_type] = setup.value.to_le_bytes();
            Ok(TransferRequest::Out(transfer_out(
                TsUrbOutKind::CtlDescReq(TsUrbControlDescRequest {
                    index,
                    desc_type,
                    lang_id: setup.index,
                }),
                func,
                data,
            )))
        }
        standard_request::GET_STATUS if direction == Direction::In && setup.value == 0 && setup.length == 2 => {
            let Some(func) = get_status_function(recipient) else {
                return Err(data);
            };
            Ok(TransferRequest::In(transfer_in(
                TsUrbInKind::CtlGetStatus(TsUrbControlGetStatusRequest { index: setup.index }),
                func,
                2,
            )))
        }
        standard_request::CLEAR_FEATURE | standard_request::SET_FEATURE
            if direction == Direction::Out && setup.length == 0 =>
        {
            let Some(func) = feature_function(standard_request, recipient) else {
                return Err(data);
            };
            // MS-RDPEUSB 2.2.9.10 requires this USB host-to-device operation
            // in TRANSFER_IN_REQUEST with an empty output buffer.
            Ok(TransferRequest::In(transfer_in(
                TsUrbInKind::CtlFeatReq(TsUrbControlFeatRequest {
                    feat_selector: setup.value,
                    index: setup.index,
                }),
                func,
                0,
            )))
        }
        standard_request::GET_CONFIGURATION
            if direction == Direction::In
                && recipient == Recipient::DEVICE
                && setup.value == 0
                && setup.index == 0
                && setup.length == 1 =>
        {
            Ok(TransferRequest::In(transfer_in(
                TsUrbInKind::CtlGetConfig(TsUrbControlGetConfigRequest),
                UrbFunction::URB_FUNCTION_GET_CONFIGURATION,
                1,
            )))
        }
        standard_request::GET_INTERFACE
            if direction == Direction::In
                && recipient == Recipient::INTERFACE
                && setup.value == 0
                && setup.length == 1 =>
        {
            Ok(TransferRequest::In(transfer_in(
                TsUrbInKind::CtlGetIface(TsUrbControlGetInterfaceRequest { interface: setup.index }),
                UrbFunction::URB_FUNCTION_GET_INTERFACE,
                1,
            )))
        }
        _ => Err(data),
    }
}

fn generic_control_transfer(setup: SetupPacket, data: Vec<u8>) -> TransferRequest {
    let direction = setup.request_type.direction();
    let urb = TsUrbControlTransfer {
        pipe: DEFAULT_CONTROL_PIPE_HANDLE,
        transfer_flags: in_transfer_flags(direction) | USBD_DEFAULT_PIPE_TRANSFER,
        setup_packet: rdpeusb_setup_packet(setup),
    };

    match direction {
        Direction::In => TransferRequest::In(transfer_in(
            TsUrbInKind::CtlTransfer(urb),
            UrbFunction::URB_FUNCTION_CONTROL_TRANSFER,
            u32::from(setup.length),
        )),
        Direction::Out => TransferRequest::Out(transfer_out(
            TsUrbOutKind::CtlTransfer(urb),
            UrbFunction::URB_FUNCTION_CONTROL_TRANSFER,
            data,
        )),
    }
}

fn validate_control_data(setup: SetupPacket, data: &[u8]) -> Result<(), ConversionError> {
    match setup.request_type.direction() {
        Direction::In if !data.is_empty() => Err(ConversionError::InTransferHasData { actual: data.len() }),
        Direction::Out if data.len() != usize::from(setup.length) => Err(ConversionError::OutTransferLengthMismatch {
            expected: usize::from(setup.length),
            actual: data.len(),
        }),
        Direction::In | Direction::Out => Ok(()),
    }
}

#[derive(Debug, Clone, Copy)]
enum DescriptorOperation {
    Get,
    Set,
}

fn descriptor_function(recipient: Recipient, operation: DescriptorOperation) -> Option<UrbFunction> {
    match (operation, recipient) {
        (DescriptorOperation::Get, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE),
        (DescriptorOperation::Get, Recipient::INTERFACE) => {
            Some(UrbFunction::URB_FUNCTION_GET_DESCRIPTOR_FROM_INTERFACE)
        }
        (DescriptorOperation::Get, Recipient::ENDPOINT) => Some(UrbFunction::URB_FUNCTION_GET_DESCRIPTOR_FROM_ENDPOINT),
        (DescriptorOperation::Set, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_SET_DESCRIPTOR_TO_DEVICE),
        (DescriptorOperation::Set, Recipient::INTERFACE) => Some(UrbFunction::URB_FUNCTION_SET_DESCRIPTOR_TO_INTERFACE),
        (DescriptorOperation::Set, Recipient::ENDPOINT) => Some(UrbFunction::URB_FUNCTION_SET_DESCRIPTOR_TO_ENDPOINT),
        _ => None,
    }
}

fn feature_function(request: u8, recipient: Recipient) -> Option<UrbFunction> {
    match (request, recipient) {
        (standard_request::CLEAR_FEATURE, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_CLEAR_FEATURE_TO_DEVICE),
        (standard_request::CLEAR_FEATURE, Recipient::INTERFACE) => {
            Some(UrbFunction::URB_FUNCTION_CLEAR_FEATURE_TO_INTERFACE)
        }
        (standard_request::CLEAR_FEATURE, Recipient::ENDPOINT) => {
            Some(UrbFunction::URB_FUNCTION_CLEAR_FEATURE_TO_ENDPOINT)
        }
        (standard_request::CLEAR_FEATURE, Recipient::OTHER) => Some(UrbFunction::URB_FUNCTION_CLEAR_FEATURE_TO_OTHER),
        (standard_request::SET_FEATURE, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_SET_FEATURE_TO_DEVICE),
        (standard_request::SET_FEATURE, Recipient::INTERFACE) => {
            Some(UrbFunction::URB_FUNCTION_SET_FEATURE_TO_INTERFACE)
        }
        (standard_request::SET_FEATURE, Recipient::ENDPOINT) => Some(UrbFunction::URB_FUNCTION_SET_FEATURE_TO_ENDPOINT),
        (standard_request::SET_FEATURE, Recipient::OTHER) => Some(UrbFunction::URB_FUNCTION_SET_FEATURE_TO_OTHER),
        _ => None,
    }
}

fn get_status_function(recipient: Recipient) -> Option<UrbFunction> {
    match recipient {
        Recipient::DEVICE => Some(UrbFunction::URB_FUNCTION_GET_STATUS_FROM_DEVICE),
        Recipient::INTERFACE => Some(UrbFunction::URB_FUNCTION_GET_STATUS_FROM_INTERFACE),
        Recipient::ENDPOINT => Some(UrbFunction::URB_FUNCTION_GET_STATUS_FROM_ENDPOINT),
        Recipient::OTHER => Some(UrbFunction::URB_FUNCTION_GET_STATUS_FROM_OTHER),
        _ => None,
    }
}

fn vendor_or_class_function(kind: RequestKind, recipient: Recipient) -> Option<UrbFunction> {
    match (kind, recipient) {
        (RequestKind::VENDOR, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_VENDOR_DEVICE),
        (RequestKind::VENDOR, Recipient::INTERFACE) => Some(UrbFunction::URB_FUNCTION_VENDOR_INTERFACE),
        (RequestKind::VENDOR, Recipient::ENDPOINT) => Some(UrbFunction::URB_FUNCTION_VENDOR_ENDPOINT),
        (RequestKind::VENDOR, Recipient::OTHER) => Some(UrbFunction::URB_FUNCTION_VENDOR_OTHER),
        (RequestKind::CLASS, Recipient::DEVICE) => Some(UrbFunction::URB_FUNCTION_CLASS_DEVICE),
        (RequestKind::CLASS, Recipient::INTERFACE) => Some(UrbFunction::URB_FUNCTION_CLASS_INTERFACE),
        (RequestKind::CLASS, Recipient::ENDPOINT) => Some(UrbFunction::URB_FUNCTION_CLASS_ENDPOINT),
        (RequestKind::CLASS, Recipient::OTHER) => Some(UrbFunction::URB_FUNCTION_CLASS_OTHER),
        _ => None,
    }
}

fn configuration_descriptor(descriptor: ConfigurationDescriptorSet<'_>) -> Result<UsbConfigDesc, ConversionError> {
    let configuration = descriptor.configuration();
    if usize::from(configuration.length()) != UsbConfigDesc::FIXED_PART_SIZE {
        return Err(ConversionError::InvalidConfigurationHeaderLength {
            actual: configuration.length(),
        });
    }

    Ok(UsbConfigDesc {
        length: configuration.length(),
        descriptor_type: configuration.raw_descriptor().descriptor_type(),
        total_length: configuration.total_length(),
        num_interfaces: configuration.num_interfaces(),
        configuration_value: configuration.configuration_value(),
        configuration: configuration.configuration_string(),
        attributes: configuration.attributes().raw(),
        max_power: configuration.max_power_raw(),
        trailing: descriptor.as_bytes()[UsbConfigDesc::FIXED_PART_SIZE..].to_vec(),
    })
}

fn interface_information(
    interface: InterfaceDescriptor<'_>,
    speed: UsbSpeed,
) -> Result<TsUsbdInterfaceInfo, ConversionError> {
    let selection = InterfaceSelection {
        interface: interface.number(),
        alternate_setting: interface.alternate_setting(),
    };
    let pipe_count = interface.endpoints().count();
    if pipe_count != usize::from(interface.num_endpoints()) {
        return Err(ConversionError::EndpointCountMismatch {
            selection,
            declared: interface.num_endpoints(),
            actual: pipe_count,
        });
    }
    if pipe_count > MAX_NON_DEFAULT_EP_COUNT {
        return Err(ConversionError::TooManyPipes {
            selection,
            actual: pipe_count,
        });
    }

    let mut pipes = Vec::with_capacity(pipe_count);
    for endpoint in interface.endpoints() {
        let max_packet_size = maximum_packet_size(endpoint.max_packet_size(), endpoint.transfer_type(), speed)
            .map_err(|raw| ConversionError::InvalidMaximumPacketSize { selection, raw })?;
        pipes.push(TsUsbdPipeInfo {
            max_packet_size,
            // MS-RDPEUSB 2.2.9.1.3 notes that the client ignores this
            // obsolete USBD field. Zero requests the client default.
            max_transfer_size: 0,
            // No general USB semantic requests a Windows pipe override.
            pipe_flags: 0,
        });
    }

    Ok(TsUsbdInterfaceInfo {
        interface_number: selection.interface,
        alternate_setting: selection.alternate_setting,
        ts_usbd_pipe_info: pipes,
    })
}

/// Convert USB `wMaxPacketSize` semantics into RDPEUSB's effective pipe size.
///
/// High-speed isochronous and interrupt endpoints include the additional
/// transactions per microframe. Other endpoint kinds and speeds reject those
/// high-bandwidth bits instead of silently changing their meaning.
pub fn maximum_packet_size(
    max_packet_size: ironrdp_usb::endpoint::MaxPacketSize,
    transfer_type: TransferType,
    speed: UsbSpeed,
) -> Result<u16, u16> {
    let raw = max_packet_size.raw();
    if raw & 0xe000 != 0 {
        return Err(raw);
    }

    let high_speed_periodic =
        speed == UsbSpeed::High && matches!(transfer_type, TransferType::Isochronous | TransferType::Interrupt);
    if high_speed_periodic {
        let bytes = max_packet_size
            .high_speed_payload_per_microframe()
            .map_err(|error| error.raw())?;
        // At most 3 * 0x7ff, which is representable as u16.
        Ok(bytes as u16)
    } else if raw & 0x1800 != 0 {
        Err(raw)
    } else {
        Ok(max_packet_size.packet_size())
    }
}

fn in_transfer_flags(direction: Direction) -> u32 {
    match direction {
        Direction::In => USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK,
        Direction::Out => 0,
    }
}

fn rdpeusb_setup_packet(setup: SetupPacket) -> RdpeusbSetupPacket {
    RdpeusbSetupPacket {
        request_type: setup.request_type.raw(),
        request: setup.request,
        value: setup.value,
        index: setup.index,
        length: setup.length,
    }
}

fn transfer_in(kind: TsUrbInKind, func: UrbFunction, output_buffer_size: u32) -> TransferInPacket {
    TransferInPacket {
        ts_urb: TsUrbInPacket { kind, func },
        output_buffer_size,
    }
}

fn transfer_out(kind: TsUrbOutKind, func: UrbFunction, output_buffer: Vec<u8>) -> TransferOutPacket {
    TransferOutPacket {
        ts_urb: TsUrbOutPacket {
            kind,
            no_ack: false,
            func,
        },
        output_buffer,
    }
}
