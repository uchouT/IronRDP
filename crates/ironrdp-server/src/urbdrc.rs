use ironrdp_dvc::DynamicChannelId;
use ironrdp_pdu::PduResult;
use ironrdp_rdpeusb::server::{UrbdrcControlServerBackend, UrbdrcDeviceServerBackend};
use tokio::sync::mpsc::UnboundedSender;

use crate::ServerEvent;

#[derive(Debug)]
pub enum UrbdrcServerMessage {
    AddChan,
}

pub trait DeviceFactory {
    fn create_device(&mut self, handle: UsbDeviceHandle) -> Option<Box<dyn UrbdrcDeviceServerBackend>>;
}

pub(super) struct UsbControlHandle {
    event_sender: UnboundedSender<ServerEvent>,
}

impl UsbControlHandle {
    pub(super) fn new(event_sender: UnboundedSender<ServerEvent>) -> Self {
        Self { event_sender }
    }
}

impl UrbdrcControlServerBackend for UsbControlHandle {
    fn create_device_chan(&mut self) -> PduResult<()> {
        let _ = self
            .event_sender
            .send(ServerEvent::Usb(UrbdrcServerMessage::AddChan));
        Ok(())
    }
}

/// Server device request sending handle.
#[derive(Debug, Clone)]
pub struct UsbDeviceHandle {
    sender: UnboundedSender<ServerEvent>,
    channel_id: DynamicChannelId,
}

impl UsbDeviceHandle {
    pub(super) fn new(sender: UnboundedSender<ServerEvent>, channel_id: DynamicChannelId) -> Self {
        Self { sender, channel_id }
    }

    pub fn transfer_in_request() {
        todo!()
    }

    pub fn transfer_out_request() {
        todo!()
    }

    pub fn ioctl_request() {
        todo!()
    }

    pub fn internal_ioctl_request() {
        todo!()
    }

    pub fn query_device_text_request() {
        todo!()
    }

    pub fn retract_request() {
        todo!()
    }
}
