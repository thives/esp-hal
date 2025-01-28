//! # Inter-Integrated Circuit (I2C) - Slave mode
//!
//! ## Overview
//!
//! In this mode, the I2C acts as slave; listens for START condition and
//! responds to I2C commands from one or more masters.
//!
//! ## Configuration
//!
//! Each I2C Slave is individually configurable, and the usual
//! setting such as frequency, timeout, and SDA/SCL pins can easily be
//! configured.
//!
//! ## Usage
//!
//! The I2C driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community, including the [embedded-hal].
//!
//! [embedded-hal]: embedded_hal

use core::marker::PhantomData;

#[cfg(any(doc, feature = "unstable"))]
use embassy_embedded_hal::SetConfig;
use embedded_hal::i2c::Operation as EhalOperation;
use enumset::EnumSet;

use crate::{
    gpio::{
        interconnect::{OutputConnection, PeripheralOutput},
        InputSignal, OutputSignal, PinGuard, Pull,
    },
    i2c::driver::Driver,
    i2c::{AnyI2c, Config, ConfigError, Error, Event, I2cAddress, Instance, OpKind, Operation, i2c::I2c},
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    private,
    system::{PeripheralClockControl, PeripheralGuard},
    Async, Blocking, DriverMode,
};

/// I2C driver
///
/// ### I2C initialization
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::i2c::slave::{Config, I2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = I2c::new(
///     peripherals.I2C0,
///     Config::default(),
/// )
/// .unwrap()
/// .with_sda(peripherals.GPIO1)
/// .with_scl(peripherals.GPIO2);
///
/// # }
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2cSlave<'d, Dm: DriverMode> {
    i2c: I2c<'d>,
    phantom: PhantomData<Dm>,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm: DriverMode> SetConfig for I2cSlave<'_, Dm> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.i2c.apply_config(config)
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2cSlave<'_, Dm> {
    type Error = Error;
}
