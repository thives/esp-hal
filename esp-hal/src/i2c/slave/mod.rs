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
use core::ops::AsyncFn;

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
        self.apply_config(config)
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2cSlave<'_, Dm> {
    type Error = Error;
}

impl embedded_hal::i2c::I2cSlave for I2cSlave<'_, Blocking> {
    fn listen<W: Fn(u8, &[u8]), R: Fn(u8, &mut [u8])>(
        &mut self,
        address: u8,
        write: W,
        read: R,
    ) -> Result<(), Self::Error> {
        todo!()
    }
}

impl embedded_hal_async::i2c::I2cSlave for I2cSlave<'_, Async> {
    async fn listen<
        W: AsyncFn(u8, &[u8]) -> Result<(), Self::Error>,
        R: AsyncFn(u8, &mut [u8]) -> Result<(), Self::Error>,
    >(
        &mut self,
        address: u8,
        write: W,
        read: R,
    ) -> Result<(), Self::Error> {
        todo!()
    }
}

impl<'d, Dm: DriverMode> I2cSlave<'d, Dm> {
    /// Applies a new configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.i2c.apply_config(config)
    }

    /// Connect a pin to the I2C SDA signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_sda(mut self, sda: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        self.i2c.with_sda(sda);
        self
    }

    /// Connect a pin to the I2C SCL signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_scl(mut self, scl: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        self.i2c.with_scl(scl);
        self
    }
}

impl<'d> I2cSlave<'d, Blocking> {
    /// Create a new I2C instance.
    pub fn new(
        i2c: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        crate::into_mapped_ref!(i2c);

        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let sda_pin = PinGuard::new_unconnected(i2c.info().sda_output);
        let scl_pin = PinGuard::new_unconnected(i2c.info().scl_output);

        let slave = I2cSlave {
            i2c: I2c {
                i2c,
                config,
                guard,
                sda_pin,
                scl_pin,
            },
            phantom: PhantomData,
        };

        slave.i2c.driver().setup(&slave.i2c.config)?;

        Ok(slave)
    }

    #[cfg_attr(
        not(multi_core),
        doc = "Registers an interrupt handler for the peripheral."
    )]
    #[cfg_attr(
        multi_core,
        doc = "Registers an interrupt handler for the peripheral on the current core."
    )]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [crate::DEFAULT_INTERRUPT_HANDLER]
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2c.driver().info.set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.driver().info.enable_listen(interrupts.into(), true)
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.driver().info.enable_listen(interrupts.into(), false)
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<Event> {
        self.i2c.driver().info.interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<Event>) {
        self.i2c.driver().info.clear_interrupts(interrupts)
    }

    /// Configures the I2C peripheral to operate in asynchronous mode.
    pub fn into_async(mut self) -> I2cSlave<'d, Async> {
        self.set_interrupt_handler(self.i2c.driver().info.async_handler);

        I2cSlave {
            i2c: self.i2c,
            phantom: PhantomData,
        }
    }

    /// Writes bytes to slave with address `address`
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )
    /// # .unwrap();
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// i2c.write(DEVICE_ADDR, &[0xaa]).ok();
    /// # }
    /// ```
    pub fn write<A: Into<I2cAddress>>(&mut self, address: A, buffer: &[u8]) -> Result<(), Error> {
        self.i2c.driver()
            .write_blocking(address.into(), buffer, true, true)
            .inspect_err(|_| self.i2c.internal_recover())
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )
    /// # .unwrap();
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.read(DEVICE_ADDR, &mut data).ok();
    /// # }
    /// ```
    pub fn read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.i2c.driver()
            .read_blocking(address.into(), buffer, true, true, false)
            .inspect_err(|_| self.i2c.internal_recover())
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes
    /// to fill `buffer` *in a single transaction*
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )
    /// # .unwrap();
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.write_read(DEVICE_ADDR, &[0xaa], &mut data).ok();
    /// # }
    /// ```
    pub fn write_read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let address = address.into();

        self.i2c.driver()
            .write_blocking(address, write_buffer, true, read_buffer.is_empty())
            .inspect_err(|_| self.i2c.internal_recover())?;

        self.i2c.driver()
            .read_blocking(address, read_buffer, true, true, false)
            .inspect_err(|_| self.i2c.internal_recover())?;

        Ok(())
    }
}