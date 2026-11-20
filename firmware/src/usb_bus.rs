//! HC32F460 USBFS device in slave/FIFO mode, polled with its interrupt masked (RM chapter 32).

use core::cell::RefCell;

use cortex_m::interrupt::{self, Mutex};
use usb_device::{
    Result, UsbDirection, UsbError,
    bus::{PollResult, UsbBus},
    endpoint::{EndpointAddress, EndpointType},
};

const BASE: usize = 0x400c_0000;
const VBUS: usize = 0x000;
const AHB: usize = 0x008;
const USB: usize = 0x00c;
const RESET: usize = 0x010;
const EVENTS: usize = 0x014;
const MASK: usize = 0x018;
const RX_POP: usize = 0x020;
const RX_SIZE: usize = 0x024;
const EP0_FIFO_SIZE: usize = 0x028;
const EP1_FIFO_SIZE: usize = 0x104;
const CONFIG: usize = 0x800;
const CONTROL: usize = 0x804;
const IN_MASK: usize = 0x810;
const OUT_MASK: usize = 0x814;
const EP_MASK: usize = 0x81c;
const IN: usize = 0x900;
const OUT: usize = 0xb00;
const FIFO: usize = 0x1000;
const STOP_CLOCK: usize = 0xe00;

const SOFT_DISCONNECT: u32 = 1 << 1;
const CORE_RESET: u32 = 1;
const AHB_IDLE: u32 = 1 << 31;
const RX_FLUSH: u32 = 1 << 4;
const TX_FLUSH: u32 = 1 << 5;
const RESET_EVENT: u32 = 1 << 12;
const ENUM_EVENT: u32 = 1 << 13;
const RX_EVENT: u32 = 1 << 4;
const SUSPEND_EVENT: u32 = 1 << 11;
const RESUME_EVENT: u32 = 1 << 31;
const CLEAR_GLOBAL_NAK: u32 = (1 << 8) | (1 << 10);
const ENABLE: u32 = 1 << 31;
const DISABLE: u32 = 1 << 30;
const DATA0: u32 = 1 << 28;
const SET_NAK: u32 = 1 << 27;
const CLEAR_NAK: u32 = 1 << 26;
const STALL: u32 = 1 << 21;
const ACTIVE: u32 = 1 << 15;
// USBFS endpoint type field (EPTYP): interrupt = 3, bulk = 2.
const EP1_TYPE: u32 = if crate::USB_EP1_BULK {
    2 << 18
} else {
    3 << 18
};
const ONE_PACKET: u32 = 1 << 19;
const EP0_SETUP_COUNT: u32 = 3 << 29;
const TRANSFER_DONE: u32 = 1;
const SETUP_DONE: u32 = 1 << 3;
const MAX_PACKET: usize = 64;

fn read(offset: usize) -> u32 {
    // SAFETY: offsets are USBFS registers or FIFO apertures; the caller owns USBFS.
    unsafe { ((BASE + offset) as *const u32).read_volatile() }
}

fn write(offset: usize, value: u32) {
    // SAFETY: same ownership and address contract as read.
    unsafe { ((BASE + offset) as *mut u32).write_volatile(value) }
}

fn wait(offset: usize, mask: u32, value: u32) -> bool {
    (0..100_000).any(|_| read(offset) & mask == value)
}

fn reset_core() -> bool {
    if !wait(RESET, AHB_IDLE, AHB_IDLE) {
        return false;
    }
    write(RESET, CORE_RESET);
    wait(RESET, CORE_RESET, 0)
}

fn ep(base: usize, index: usize) -> usize {
    base + index * 0x20
}

fn fifo(index: usize) -> usize {
    FIFO * (index + 1)
}

fn arm_out(index: usize, size: u16) {
    let control = ep(OUT, index);
    let setup = if index == 0 { EP0_SETUP_COUNT } else { 0 };
    write(control + 0x10, setup | ONE_PACKET | u32::from(size));
    write(control, read(control) | ENABLE | CLEAR_NAK);
}

fn clock(hclk: u32) -> bool {
    let n = match hclk {
        120_000_000 => 30, // HRC is 16 MHz.
        150_000_000 => 24, // HRC is 20 MHz.
        _ => return false,
    };
    const FPRC: *mut u16 = 0x4005_43fe as *mut u16;
    const FCG0PC: *mut u32 = 0x4004_8010 as *mut u32;
    const FCG1: *mut u32 = 0x4004_8004 as *mut u32;
    const UPLLCR: *mut u8 = 0x4005_402e as *mut u8;
    const OSCSTBSR: *const u8 = 0x4005_403c as *const u8;
    const UPLLCFGR: *mut u32 = 0x4005_4104 as *mut u32;
    const UPLL_STABLE: u8 = 1 << 6;
    const USB_GATE: u32 = 1 << 8;

    // SAFETY: no other clock/USB owner; the PLL source is HRC. UPLL VCO = HRC / 2 * N = 240 MHz.
    unsafe {
        let cmu = &*hc32f460_pac::Cmu::ptr();
        let protection = FPRC.read_volatile() & 0x0f;
        FPRC.write_volatile(0xa500 | protection | 1);
        FCG0PC.write_volatile(0xa5a5_0001);
        FCG1.write_volatile(FCG1.read_volatile() | USB_GATE);
        UPLLCR.write_volatile(1);
        let stopped = (0..100_000).any(|_| OSCSTBSR.read_volatile() & UPLL_STABLE == 0);
        let ready = stopped && {
            UPLLCFGR.write_volatile(1 | ((n - 1) << 8) | (4 << 20) | (4 << 24) | (4 << 28));
            UPLLCR.write_volatile(0);
            (0..100_000).any(|_| OSCSTBSR.read_volatile() & UPLL_STABLE != 0)
        };
        if ready {
            // USBCKS 0b1100 selects UPLL/Q (240 MHz / 5).
            cmu.usbckcfgr().write(|w| w.usbcks().bits(0b1100));
            FCG1.write_volatile(FCG1.read_volatile() & !USB_GATE);
        }
        FCG0PC.write_volatile(0xa5a5_0000);
        FPRC.write_volatile(0xa500 | protection);
        ready
    }
}

struct Rx {
    setup: Option<[u8; 8]>,
    packets: [[u8; MAX_PACKET]; 2],
    lengths: [Option<u8>; 2],
}

impl Rx {
    const fn new() -> Self {
        Self {
            setup: None,
            packets: [[0; MAX_PACKET]; 2],
            lengths: [None; 2],
        }
    }

    fn drain(&mut self) {
        for _ in 0..8 {
            if read(EVENTS) & RX_EVENT == 0 {
                break;
            }
            let status = read(RX_POP);
            let index = (status & 15) as usize;
            let count = ((status >> 4) & 0x7ff) as usize;
            let kind = (status >> 17) & 15;
            let mut data = [0; MAX_PACKET];
            for word in 0..count.div_ceil(4) {
                let bytes = read(fifo(0)).to_le_bytes();
                if word < MAX_PACKET / 4 {
                    data[word * 4..word * 4 + 4].copy_from_slice(&bytes);
                }
            }
            if kind == 6 && index == 0 && count == 8 {
                self.setup = data[..8].try_into().ok();
                self.lengths[0] = None;
                write(ep(IN, 0) + 8, TRANSFER_DONE);
                write(IN, read(IN) & !(ENABLE | STALL));
                write(OUT, read(OUT) & !(ENABLE | STALL));
            } else if kind == 2 && index < 2 && count <= MAX_PACKET {
                self.packets[index] = data;
                self.lengths[index] = Some(count as u8);
            }
        }
    }
}

pub struct Bus {
    sizes: [[u16; 2]; 2],
    rx: Mutex<RefCell<Rx>>,
}

impl Bus {
    /// Unsafe: call once, with the USB pins and controller owned by the caller.
    pub(crate) unsafe fn start(hclk: u32) -> Option<Self> {
        if !clock(hclk) {
            return None;
        }
        write(CONTROL, SOFT_DISCONNECT);
        write(MASK, 0);
        write(AHB, 0); // FIFO slave mode, interrupts polled.
        write(STOP_CLOCK, 0);
        if !reset_core() {
            return None;
        }
        // The HC32 DDL resets once before and once after selecting the PHY.
        write(USB, 1 << 6);
        if !reset_core() {
            return None;
        }
        write(CONTROL, SOFT_DISCONNECT);
        cortex_m::asm::delay(hclk / 50);
        write(USB, (1 << 30) | (1 << 6) | (15 << 10)); // Device mode, FS PHY, turnaround.
        cortex_m::asm::delay(hclk / 20);
        if !wait(EVENTS, 1, 0) {
            return None;
        }
        write(VBUS, (1 << 6) | (1 << 7)); // Bus powered from the PC.
        write(CONFIG, 3); // Full speed with the embedded FS PHY.
        Some(Self {
            sizes: [[0; 2]; 2],
            rx: Mutex::new(RefCell::new(Rx::new())),
        })
    }
}

/// Drop off the bus.
pub fn detach() {
    write(CONTROL, SOFT_DISCONNECT);
}

impl UsbBus for Bus {
    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = true;

    fn alloc_ep(
        &mut self,
        direction: UsbDirection,
        address: Option<EndpointAddress>,
        kind: EndpointType,
        size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress> {
        let index = match kind {
            EndpointType::Control if size == 64 => 0,
            EndpointType::Interrupt if !crate::USB_EP1_BULK && size == 64 => 1,
            EndpointType::Bulk if crate::USB_EP1_BULK && size == 64 => 1,
            _ => return Err(UsbError::EndpointOverflow),
        };
        let endpoint = EndpointAddress::from_parts(index, direction);
        let sizes = &mut self.sizes[usize::from(direction == UsbDirection::In)];
        if address.is_some_and(|a| a != endpoint) || sizes[index] != 0 {
            return Err(UsbError::InvalidEndpoint);
        }
        sizes[index] = size;
        Ok(endpoint)
    }

    fn enable(&mut self) {
        write(EVENTS, u32::MAX);
        write(CONTROL, 0);
    }

    fn reset(&self) {
        write(CONFIG, 3);
        write(RX_SIZE, 128);
        write(EP0_FIFO_SIZE, (16 << 16) | 128);
        write(EP1_FIFO_SIZE, (32 << 16) | 144);
        write(RESET, TX_FLUSH | (0x10 << 6));
        wait(RESET, TX_FLUSH, 0);
        write(RESET, RX_FLUSH);
        wait(RESET, RX_FLUSH, 0);
        interrupt::free(|cs| *self.rx.borrow(cs).borrow_mut() = Rx::new());
        write(IN_MASK, TRANSFER_DONE);
        write(OUT_MASK, TRANSFER_DONE | SETUP_DONE);
        write(EP_MASK, 0x0003_0003);
        write(ep(IN, 0) + 8, u32::MAX);
        write(ep(OUT, 0) + 8, u32::MAX);
        write(ep(IN, 1) + 8, u32::MAX);
        write(ep(OUT, 1) + 8, u32::MAX);
        write(ep(IN, 0), ACTIVE | SET_NAK); // EP0 maximum packet field 0 = 64.
        write(ep(OUT, 0), ACTIVE);
        write(
            ep(IN, 1),
            ACTIVE | EP1_TYPE | DATA0 | SET_NAK | (1 << 22) | 64,
        );
        write(ep(OUT, 1), ACTIVE | EP1_TYPE | DATA0 | 64);
        arm_out(0, 64);
        arm_out(1, 64);
    }

    fn set_device_address(&self, address: u8) {
        write(
            CONFIG,
            (read(CONFIG) & !(0x7f << 4)) | (u32::from(address) << 4),
        );
    }

    fn write(&self, address: EndpointAddress, bytes: &[u8]) -> Result<usize> {
        let index = address.index();
        let size = *self.sizes[1].get(index).ok_or(UsbError::InvalidEndpoint)?;
        if size == 0 || address.direction() != UsbDirection::In {
            return Err(UsbError::InvalidEndpoint);
        }
        if bytes.len() > usize::from(size) {
            return Err(UsbError::BufferOverflow);
        }
        let control = ep(IN, index);
        if read(control) & (ENABLE | STALL) != 0
            || (read(control + 0x18) & 0xffff) < bytes.len().div_ceil(4) as u32
        {
            return Err(UsbError::WouldBlock);
        }
        write(control + 0x10, ONE_PACKET | bytes.len() as u32);
        write(control, read(control) | ENABLE | CLEAR_NAK);
        for chunk in bytes.chunks(4) {
            let mut word = [0; 4];
            word[..chunk.len()].copy_from_slice(chunk);
            write(fifo(index), u32::from_le_bytes(word));
        }
        Ok(bytes.len())
    }

    fn read(&self, address: EndpointAddress, out: &mut [u8]) -> Result<usize> {
        let index = address.index();
        if index >= 2 || self.sizes[0][index] == 0 || address.direction() != UsbDirection::Out {
            return Err(UsbError::InvalidEndpoint);
        }
        interrupt::free(|cs| {
            let mut rx = self.rx.borrow(cs).borrow_mut();
            if index == 0
                && let Some(setup) = rx.setup.take()
            {
                out.get_mut(..8)
                    .ok_or(UsbError::BufferOverflow)?
                    .copy_from_slice(&setup);
                return Ok(8);
            }
            let count = usize::from(rx.lengths[index].take().ok_or(UsbError::WouldBlock)?);
            out.get_mut(..count)
                .ok_or(UsbError::BufferOverflow)?
                .copy_from_slice(&rx.packets[index][..count]);
            Ok(count)
        })
    }

    fn set_stalled(&self, address: EndpointAddress, stalled: bool) {
        let index = address.index();
        if index >= 2 {
            return;
        }
        let base = if address.direction() == UsbDirection::In {
            IN
        } else {
            OUT
        };
        let control = ep(base, index);
        let value = read(control);
        if stalled {
            write(
                control,
                (value & !ENABLE)
                    | STALL
                    | if value & ENABLE != 0 && base == IN {
                        DISABLE
                    } else {
                        0
                    },
            );
        } else if index != 0 || value & STALL != 0 {
            if base == IN && index != 0 && value & ENABLE != 0 {
                write(control, (value & !ENABLE) | DISABLE | SET_NAK);
                wait(control, ENABLE, 0);
                write(RESET, TX_FLUSH | ((index as u32) << 6));
                wait(RESET, TX_FLUSH, 0);
                write(control + 8, u32::MAX);
            }
            write(
                control,
                (value & !(STALL | ENABLE)) | if index == 0 { 0 } else { DATA0 },
            );
            if base == OUT {
                interrupt::free(|cs| self.rx.borrow(cs).borrow_mut().lengths[index] = None);
            }
        }
    }

    fn is_stalled(&self, address: EndpointAddress) -> bool {
        let base = if address.direction() == UsbDirection::In {
            IN
        } else {
            OUT
        };
        address.index() < 2 && read(ep(base, address.index())) & STALL != 0
    }

    fn suspend(&self) {}

    fn resume(&self) {}

    fn poll(&self) -> PollResult {
        let events = read(EVENTS);
        if events & RESET_EVENT != 0 {
            write(
                EVENTS,
                RESET_EVENT | ENUM_EVENT | SUSPEND_EVENT | RESUME_EVENT,
            );
            return PollResult::Reset;
        }
        if events & ENUM_EVENT != 0 {
            write(CONTROL, read(CONTROL) | CLEAR_GLOBAL_NAK);
            write(EVENTS, ENUM_EVENT);
        }
        if events & RESUME_EVENT != 0 {
            write(EVENTS, RESUME_EVENT);
            return PollResult::Resume;
        }
        interrupt::free(|cs| {
            let mut rx = self.rx.borrow(cs).borrow_mut();
            rx.drain();
            let mut ep_out = 0;
            let mut ep_in_complete = 0;
            let mut ep_setup = 0;
            for index in 0..2 {
                let bit = 1 << index;
                let out = ep(OUT, index);
                let flags = read(out + 8);
                let setup = index == 0 && rx.setup.is_some();
                if setup && flags & SETUP_DONE != 0 {
                    ep_setup |= bit;
                }
                write(out + 8, flags);
                if rx.lengths[index].is_some() {
                    ep_out |= bit;
                } else if !setup && read(out) & (ENABLE | STALL) == 0 {
                    arm_out(index, self.sizes[0][index]);
                }
                let input = ep(IN, index) + 8;
                let flags = read(input);
                if flags & TRANSFER_DONE != 0 && !setup {
                    ep_in_complete |= bit;
                }
                write(input, flags);
            }
            if ep_out | ep_in_complete | ep_setup != 0 {
                PollResult::Data {
                    ep_out,
                    ep_in_complete,
                    ep_setup,
                }
            } else if events & SUSPEND_EVENT != 0 {
                write(EVENTS, SUSPEND_EVENT);
                PollResult::Suspend
            } else {
                PollResult::None
            }
        })
    }
}
