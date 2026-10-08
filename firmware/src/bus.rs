//! The shared I2C3 bus: meter and tilt sensor access, the failure counters
//! shown on the test page, and the latch that closes the bus after a fault.

use core::sync::atomic::{AtomicU32, Ordering};

use firmware_core::latch::Latch;

use crate::{ina226, lis3dh};

// Board-owned shared bus: platform drivers never depend on application task ownership.
pub static I2C3: embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Option<embassy_hc32::i2c::I2c3>,
> = embassy_sync::mutex::Mutex::new(None);

/// How many times the four I2C3 interrupt lines fired. Zero means the wake path
/// (routing or `CR2` arming) never worked; a climbing count means the waits are
/// woken and any failure is in the transaction logic instead.
pub static I2C_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);

/// Completed async I2C3 reads and failures, shown on the test page. Timeouts
/// (no wake at all) and bus errors (NACK/arbitration/timeout flag) are counted
/// separately: they point at completely different faults.
pub static ASYNC_READS: AtomicU32 = AtomicU32::new(0);
pub static I2C_ERRORS: AtomicU32 = AtomicU32::new(0);
pub static I2C_TIMEOUTS: AtomicU32 = AtomicU32::new(0);
/// `SR` and `CR1` as they stood when the bus last failed, for diagnosis.
pub static I2C_LAST_SR: AtomicU32 = AtomicU32::new(0);
pub static I2C_LAST_CR1: AtomicU32 = AtomicU32::new(0);
/// Bring-up step the driver was in when it timed out (`0` = no timeout).
pub static I2C_FAULT_STEP: AtomicU32 = AtomicU32::new(0);
/// `CR4` as it stood when the bus last failed (`BUSWAIT` is bit 10).
pub static I2C_LAST_CR4: AtomicU32 = AtomicU32::new(0);
/// Latches the bus off after the first failure (`firmware_core::latch`).
pub static BUS_LATCH: Latch = Latch::new();

/// Whether the shared bus is latched off after a failure.
pub fn bus_failed() -> bool {
    BUS_LATCH.is_failed()
}

/// Record a bus failure: count its kind, keep `SR` for the test page, latch off.
pub fn note_bus_failure(bus: &mut embassy_hc32::i2c::I2c3, error: embassy_hc32::i2c::Error) {
    match error {
        embassy_hc32::i2c::Error::Timeout(step) => {
            I2C_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
            I2C_FAULT_STEP.store(step, Ordering::Relaxed);
        }
        _ => {
            I2C_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
    }
    I2C_LAST_SR.store(bus.status(), Ordering::Relaxed);
    I2C_LAST_CR1.store(bus.control(), Ordering::Relaxed);
    I2C_LAST_CR4.store(bus.control4(), Ordering::Relaxed);
    BUS_LATCH.fail();
}

/// Read one big-endian INA226 register through the async I2C3 master.
///
/// `tilt_task` shares the bus, so the lock is what keeps a single owner at a
/// time. The first failure latches the bus off: a wedged transfer must not be
/// retried for ever, and the screen stays alive either way.
pub async fn read_meter(reg: u8) -> Option<u16> {
    if bus_failed() {
        return None;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return None;
    }
    let bus = guard.as_mut()?;
    match bus.read_register(ina226::ADDRESS, reg).await {
        Ok(value) => {
            ASYNC_READS.fetch_add(1, Ordering::Relaxed);
            Some(value)
        }
        Err(error) => {
            note_bus_failure(bus, error);
            None
        }
    }
}

/// Write one big-endian INA226 register (startup configuration).
pub async fn write_meter(reg: u8, value: u16) -> bool {
    if bus_failed() {
        return false;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return false;
    }
    let Some(bus) = guard.as_mut() else {
        return false;
    };
    match bus.write_register(ina226::ADDRESS, reg, value).await {
        Ok(()) => true,
        Err(error) => {
            note_bus_failure(bus, error);
            false
        }
    }
}

/// Write one LIS3DH register; the tilt sensor sits on the same bus.
pub async fn write_sensor(reg: u8, value: u8) -> bool {
    if bus_failed() {
        return false;
    }
    let mut guard = I2C3.lock().await;
    if bus_failed() {
        return false;
    }
    let Some(bus) = guard.as_mut() else {
        return false;
    };
    match bus.write_register8(lis3dh::ADDRESS, reg, value).await {
        Ok(()) => true,
        Err(error) => {
            note_bus_failure(bus, error);
            false
        }
    }
}
