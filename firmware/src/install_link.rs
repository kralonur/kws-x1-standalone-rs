//! Unpacker only: polled USB HID device for the USB install (`firmware_image::link`).

use cortex_m::peripheral::{SYST, syst::SystClkSource};
use embassy_hc32::gpio::{AnalogPin, Port};
use firmware_image::link::{self, REPORT_BYTES, REPORT_DESCRIPTOR, Receiver};
use usb_device::{
    bus::{InterfaceNumber, UsbBus, UsbBusAllocator},
    class::{ControlIn, ControlOut, UsbClass},
    control::{Recipient, RequestType},
    descriptor::DescriptorWriter,
    device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
    endpoint::{EndpointIn, EndpointOut},
};

use crate::usb_bus::{self, Bus};

/// SRAM wait control and its protection registers (RM 8.2).
const SRAM_WTCR: *mut u16 = 0x4005_0800 as *mut u16;
const SRAM_WTPR: *mut u16 = 0x4005_0804 as *mut u16;
const SRAM_CKPR: *mut u16 = 0x4005_080c as *mut u16;
const SRAM_KEY: u16 = 0x3b << 1;
/// One read and write wait for SRAM12 and SRAM3 (RM Table 8-1).
const SRAM_ONE_WAIT: u16 = 1 << 12 | 1 << 8 | 1 << 4 | 1;

const HID_CLASS: u8 = 3;
const HID_DESCRIPTOR: u8 = 0x21;
const REPORT_DESCRIPTOR_TYPE: u8 = 0x22;
const GET_DESCRIPTOR: u8 = 6;
const SET_IDLE: u8 = 0x0a;
/// bcdHID 1.11, no country, one report descriptor.
const HID_FIELDS: [u8; 7] = [
    0x11,
    0x01,
    0,
    1,
    REPORT_DESCRIPTOR_TYPE,
    REPORT_DESCRIPTOR.len() as u8,
    0,
];

/// Timer ticks per second.
const TICK_HZ: u32 = 100;
/// With the region already installed, wait this long for a flasher.
const IDLE_TICKS: u32 = 10 * TICK_HZ;
/// Keep polling after the last reply so the host reads it.
const LINGER_TICKS: u32 = TICK_HZ / 4;

struct Hid<'a, B: UsbBus> {
    interface: InterfaceNumber,
    output: EndpointOut<'a, B>,
    input: EndpointIn<'a, B>,
}

impl<B: UsbBus> UsbClass<B> for Hid<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface(self.interface, HID_CLASS, 0, 0)?;
        writer.write(HID_DESCRIPTOR, &HID_FIELDS)?;
        writer.endpoint(&self.output)?;
        writer.endpoint(&self.input)
    }

    fn control_in(&mut self, transfer: ControlIn<B>) {
        let request = *transfer.request();
        if request.request_type == RequestType::Standard
            && request.recipient == Recipient::Interface
            && request.index == u16::from(u8::from(self.interface))
            && request.request == GET_DESCRIPTOR
        {
            match (request.value >> 8) as u8 {
                REPORT_DESCRIPTOR_TYPE => transfer.accept_with_static(&REPORT_DESCRIPTOR).ok(),
                HID_DESCRIPTOR => transfer
                    .accept(|out| {
                        out[0] = 2 + HID_FIELDS.len() as u8;
                        out[1] = HID_DESCRIPTOR;
                        out[2..2 + HID_FIELDS.len()].copy_from_slice(&HID_FIELDS);
                        Ok(2 + HID_FIELDS.len())
                    })
                    .ok(),
                _ => None,
            };
        }
    }

    fn control_out(&mut self, transfer: ControlOut<B>) {
        let request = *transfer.request();
        if request.request_type == RequestType::Class
            && request.recipient == Recipient::Interface
            && request.index == u16::from(u8::from(self.interface))
            && request.request == SET_IDLE
        {
            transfer.accept().ok();
        }
    }
}

/// Unsafe: call before raising the clock, with interrupts off.
pub(crate) unsafe fn sram_waits() {
    unsafe {
        SRAM_WTPR.write_volatile(SRAM_KEY | 1);
        SRAM_CKPR.write_volatile(SRAM_KEY | 1);
        SRAM_WTCR.write_volatile(SRAM_ONE_WAIT);
        SRAM_WTPR.write_volatile(SRAM_KEY);
        SRAM_CKPR.write_volatile(SRAM_KEY);
    }
}

/// Serve the flasher; true once the upper region is installed. Unsafe: call once, interrupts off.
pub(crate) unsafe fn serve(
    hclk: u32,
    receiver: &mut Receiver,
    sector: &mut [u8],
    mut program: impl FnMut(usize, &[u8]) -> bool,
    mut verify: impl FnMut() -> bool,
) -> bool {
    // SAFETY: the USB PHY owns PA11/PA12.
    let _data_pins = unsafe { [AnalogPin::new(Port::A, 11), AnalogPin::new(Port::A, 12)] };
    // SAFETY: the caller gives this image the USB controller.
    let Some(bus) = (unsafe { Bus::start(hclk) }) else {
        return receiver.present();
    };
    let allocator = UsbBusAllocator::new(bus);
    let mut hid = Hid {
        interface: allocator.interface(),
        output: allocator.interrupt(REPORT_BYTES as u16, 1),
        input: allocator.interrupt(REPORT_BYTES as u16, 1),
    };
    let mut device = UsbDeviceBuilder::new(&allocator, UsbVidPid(link::VID, link::PID))
        .usb_rev(UsbRev::Usb200)
        .max_packet_size_0(REPORT_BYTES as u8)
        .unwrap()
        .strings(&[StringDescriptors::default()
            .manufacturer(link::MANUFACTURER)
            .product(link::PRODUCT)])
        .unwrap()
        .max_power(100)
        .unwrap()
        .build();

    // SAFETY: nothing else here uses SysTick.
    let mut timer: SYST = unsafe { cortex_m::Peripherals::steal().SYST };
    timer.set_clock_source(SystClkSource::Core);
    timer.set_reload(hclk / TICK_HZ - 1);
    timer.clear_current();
    timer.enable_counter();
    let mut now = 0u32;
    let mut reply: Option<link::Report> = None;
    let mut finished_at = None;
    loop {
        device.poll(&mut [&mut hid]);
        if timer.has_wrapped() {
            now += 1;
        }
        if let Some(report) = &reply {
            if hid.input.write(report).is_ok() {
                reply = None;
                finished_at = receiver.done().then_some(now);
            }
        } else if let Some(since) = finished_at {
            if now - since >= LINGER_TICKS {
                break;
            }
        } else {
            let mut report = [0; REPORT_BYTES];
            if let Ok(count) = hid.output.read(&mut report) {
                reply = receiver.handle(&report[..count], sector, &mut program, &mut verify);
            }
            if receiver.present() && !receiver.queried() && now >= IDLE_TICKS {
                break;
            }
        }
    }
    timer.disable_counter();
    usb_bus::detach();
    receiver.done() || receiver.present()
}
