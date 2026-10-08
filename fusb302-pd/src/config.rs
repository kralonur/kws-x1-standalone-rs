//! Readback-checked FUSB302 configuration writes; no board baseline defaults.
use crate::register::{self, control1};
use crate::snapshot as pd;
use embedded_hal_async::i2c::I2c;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Baseline { register: u8, value: u8 },
    Bus { register: u8, source: E },
    Verify { register: u8, value: u8 },
}
#[derive(Clone, Copy)]
pub enum ConfigRegister {
    Power,
    Switches0,
    Switches1,
    Control3,
    Control1,
    Control1Flush,
}
impl ConfigRegister {
    pub fn address(self) -> u8 {
        match self {
            Self::Power => register::POWER,
            Self::Switches0 => register::SWITCHES0,
            Self::Switches1 => register::SWITCHES1,
            Self::Control3 => register::CONTROL3,
            Self::Control1 | Self::Control1Flush => register::CONTROL1,
        }
    }
}
pub async fn write_checked<B: I2c>(
    bus: &mut B,
    register: ConfigRegister,
    value: u8,
) -> Result<(), Error<B::Error>> {
    let address = register.address();
    // RX_FLUSH self-clears (DS Table 23), so a flush writes only that bit and
    // reads back zero: no SOP'/SOP'' reception is left enabled.
    let (written, expected) = match register {
        ConfigRegister::Control1Flush => (control1::RX_FLUSH, 0),
        _ => (value, value),
    };
    bus.write(pd::ADDRESS, &[address, written])
        .await
        .map_err(|source| Error::Bus {
            register: address,
            source,
        })?;
    let mut actual = [0];
    bus.write_read(pd::ADDRESS, &[address], &mut actual)
        .await
        .map_err(|source| Error::Bus {
            register: address,
            source,
        })?;
    if actual[0] != expected {
        return Err(Error::Verify {
            register: address,
            value: actual[0],
        });
    }
    Ok(())
}
