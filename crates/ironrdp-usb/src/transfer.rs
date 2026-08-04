//! Protocol-independent USB transfer requests and completions.
//!
//! These are deliberately plain data structures. They describe USB operations
//! without parsing transport packets, allocating buffers, resolving endpoint
//! state, or managing request lifetimes.

use core::fmt;

use super::{control::SetupPacket, endpoint::EndpointAddress};

/// Result of a submitted USB operation.
///
/// This is unrelated to the payload returned by the USB standard `GET_STATUS`
/// request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum UsbStatus {
    Success,
    Cancelled,
    Stall,
    Timeout,
    Overflow,
    NoDevice,
    Error,
}

impl UsbStatus {
    #[must_use]
    pub const fn is_success(self) -> bool {
        matches!(self, Self::Success)
    }
}

impl fmt::Display for UsbStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Success => "success",
            Self::Cancelled => "cancelled",
            Self::Stall => "stall",
            Self::Timeout => "timeout",
            Self::Overflow => "overflow",
            Self::NoDevice => "no-device",
            Self::Error => "error",
        })
    }
}

/// Completion of a typed USB operation.
///
/// `output` is present when the operation produced a usable value. Transfer
/// methods use [`TransferCompletion`] or [`IsoCompletion`] instead because they
/// also need actual lengths and data buffers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UsbCompletion<T> {
    pub status: UsbStatus,
    pub output: Option<T>,
}

impl<T> UsbCompletion<T> {
    #[must_use]
    pub const fn success(output: T) -> Self {
        Self {
            status: UsbStatus::Success,
            output: Some(output),
        }
    }

    #[must_use]
    pub const fn failure(status: UsbStatus) -> Self {
        Self { status, output: None }
    }
}

/// One default-control-pipe transfer.
///
/// Direction and requested length are carried by `setup`. For an IN transfer,
/// `data` is empty. For an OUT transfer, it contains the data stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ControlTransferRequest<B> {
    pub setup: SetupPacket,
    pub data: B,
}

/// One bulk or interrupt transfer on a non-control endpoint.
///
/// Direction is carried by `endpoint`. For an IN endpoint, `length` is the
/// maximum requested response and `data` is empty. For an OUT endpoint,
/// `length` describes `data`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DataTransferRequest<B> {
    pub endpoint: EndpointAddress,
    pub length: u32,
    pub data: B,
}

/// A bulk transfer request.
///
/// Whether the active endpoint is actually bulk is device state checked by the
/// handle, not a property duplicated in this transport-independent type.
pub type BulkTransferRequest<B> = DataTransferRequest<B>;

/// An interrupt transfer request.
///
/// Whether the active endpoint is actually interrupt is device state checked
/// by the handle.
pub type InterruptTransferRequest<B> = DataTransferRequest<B>;

/// Completion of a control, bulk, or interrupt transfer.
///
/// For an IN transfer, `data` contains the received bytes. For an OUT transfer,
/// it is empty. `actual_length` is valid in both directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TransferCompletion<B> {
    pub status: UsbStatus,
    pub actual_length: u32,
    pub data: B,
}

pub type ControlCompletion<B> = TransferCompletion<B>;
pub type DataCompletion<B> = TransferCompletion<B>;

/// Host-controller frame number used for isochronous scheduling.
pub type FrameNumber = u32;

/// Result of one packet in an isochronous transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IsochronousPacketCompletion {
    pub status: UsbStatus,
    pub actual_length: u32,
}

/// One isochronous transfer containing one or more packets.
///
/// `start_frame == None` asks the host controller to schedule the transfer as
/// soon as possible. Packet payload slots are packed in request order. For an
/// IN endpoint `data` is empty; for an OUT endpoint it contains the packet
/// payloads in the same order. Each item in `packets` is a `u32` requested
/// packet length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IsochronousTransferRequest<B, P> {
    pub endpoint: EndpointAddress,
    pub start_frame: Option<FrameNumber>,
    pub data: B,
    pub packets: P,
}

/// Direction-independent output of an isochronous transfer.
///
/// For IN, successful packet payloads are concatenated in packet order in
/// `data`; failed packets contribute no bytes. The packet `actual_length`
/// values split the buffer. For OUT, `data` is empty.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IsochronousTransferOutput<B, P> {
    pub start_frame: FrameNumber,
    pub actual_length: u32,
    pub data: B,
    pub packets: P,
}

pub type IsoCompletion<B, P> = UsbCompletion<IsochronousTransferOutput<B, P>>;
