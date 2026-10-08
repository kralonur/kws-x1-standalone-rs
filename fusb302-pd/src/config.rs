//! Typed, readback-checked FUSB configuration writes; no board baseline defaults.
use crate::register;
use crate::snapshot as pd;
use device_driver::{AsyncRegisterInterface, Fieldset, FieldsetMetadata, RegisterInterfaceBase};
use embedded_hal_async::i2c::I2c;
use fusb302b::FusbLowLevel;

#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Baseline { register: u8, value: u8 },
    Bus { register: u8, source: E },
    Verify { register: u8, value: u8 },
}
struct RegisterIo<'a, B>(&'a mut B);
impl<B: I2c> RegisterInterfaceBase for RegisterIo<'_, B> {
    type AddressType = u8;
    type Error = B::Error;
}
impl<B: I2c> AsyncRegisterInterface for RegisterIo<'_, B> {
    async fn read_register(
        &mut self,
        address: u8,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.0.write_read(pd::ADDRESS, &[address], data).await
    }
    async fn write_register(
        &mut self,
        address: u8,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        // Both generated registers exposed below are exactly eight bits.
        assert_eq!(data.len(), 1);
        self.0.write(pd::ADDRESS, &[address, data[0]]).await
    }
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
    let mut phy = FusbLowLevel::new(RegisterIo(bus));
    let address = register.address();
    let result = match register {
        ConfigRegister::Power => {
            phy.power()
                .write_async(|r| r.as_slice_mut()[0] = value)
                .await
        }
        ConfigRegister::Switches0 => {
            phy.switches_0()
                .write_async(|r| r.as_slice_mut()[0] = value)
                .await
        }
        ConfigRegister::Switches1 => {
            phy.switches_1()
                .write_async(|r| r.as_slice_mut()[0] = value)
                .await
        }
        ConfigRegister::Control3 => {
            phy.control_3()
                .write_async(|r| r.as_slice_mut()[0] = value)
                .await
        }
        ConfigRegister::Control1 | ConfigRegister::Control1Flush => {
            phy.control_1()
                .write_async(|r| {
                    if matches!(register, ConfigRegister::Control1Flush) {
                        r.set_rx_flush(true);
                    } else {
                        r.as_slice_mut()[0] = value;
                    }
                })
                .await
        }
    };
    result.map_err(|source| Error::Bus {
        register: address,
        source,
    })?;
    let result = match register {
        ConfigRegister::Power => phy
            .power()
            .read_async()
            .await
            .map(|mut r| r.as_slice_mut()[0]),
        ConfigRegister::Switches0 => phy
            .switches_0()
            .read_async()
            .await
            .map(|mut r| r.as_slice_mut()[0]),
        ConfigRegister::Switches1 => phy
            .switches_1()
            .read_async()
            .await
            .map(|mut r| r.as_slice_mut()[0]),
        ConfigRegister::Control3 => phy
            .control_3()
            .read_async()
            .await
            .map(|mut r| r.as_slice_mut()[0]),
        ConfigRegister::Control1 | ConfigRegister::Control1Flush => phy
            .control_1()
            .read_async()
            .await
            .map(|mut r| r.as_slice_mut()[0]),
    };
    let actual = result.map_err(|source| Error::Bus {
        register: address,
        source,
    })?;
    let expected = if matches!(register, ConfigRegister::Control1Flush) {
        0
    } else {
        value
    };
    if actual != expected {
        return Err(Error::Verify {
            register: address,
            value: actual,
        });
    }
    Ok(())
}
