//! # Inter-Integrated Circuit (I2C) - Master mode
//!
//! ## Overview
//!
//! In this mode, the I2C acts as master and initiates the I2C communication by
//! generating a START condition. Note that only one master is allowed to occupy
//! the bus to access one slave at the same time.
//!
//! ## Configuration
//!
//! Each I2C Master controller is individually configurable, and the usual
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
        InputSignal,
        OutputSignal,
        PinGuard,
        Pull,
    },
    i2c::{
        AnyI2c,
        Config,
        ConfigError,
        Error,
        Event,
        I2cAddress,
        Instance,
        Operation,
        OpKind,
    },
    i2c::driver::Driver,
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    private,
    system::{PeripheralClockControl, PeripheralGuard},
    Async,
    Blocking,
    DriverMode,
};

/// I2C driver
///
/// ### I2C initialization and communication with the device
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::i2c::master::{Config, I2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = I2c::new(
///     peripherals.I2C0,
///     Config::default(),
/// )
/// .unwrap()
/// .with_sda(peripherals.GPIO1)
/// .with_scl(peripherals.GPIO2);
///
/// let mut data = [0u8; 22];
/// i2c.write_read(DEVICE_ADDR, &[0xaa], &mut data).ok();
/// # }
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<'d, Dm: DriverMode> {
    i2c: PeripheralRef<'d, AnyI2c>,
    phantom: PhantomData<Dm>,
    config: Config,
    guard: PeripheralGuard,
    sda_pin: PinGuard,
    scl_pin: PinGuard,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm: DriverMode> SetConfig for I2c<'_, Dm> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2c<'_, Dm> {
    type Error = Error;
}

impl<Dm: DriverMode> embedded_hal::i2c::I2c for I2c<'_, Dm> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl(
            I2cAddress::SevenBit(address),
            operations.iter_mut().map(Operation::from),
        )
        .inspect_err(|_| self.internal_recover())
    }
}

impl<'d, Dm: DriverMode> I2c<'d, Dm> {
    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
        }
    }

    fn internal_recover(&self) {
        PeripheralClockControl::disable(self.driver().info.peripheral);
        PeripheralClockControl::enable(self.driver().info.peripheral);
        PeripheralClockControl::reset(self.driver().info.peripheral);

        // We know the configuration is valid, we can ignore the result.
        _ = self.driver().setup(&self.config);
    }

    /// Applies a new configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().setup(config)?;
        self.config = *config;
        Ok(())
    }

    fn transaction_impl<'a>(
        &mut self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.driver().write_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                    )?;
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.driver().read_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                    )?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }

    /// Connect a pin to the I2C SDA signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_sda(mut self, sda: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Self::connect_pin(sda, input, output, &mut self.sda_pin);

        self
    }

    /// Connect a pin to the I2C SCL signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_scl(mut self, scl: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Self::connect_pin(scl, input, output, &mut self.scl_pin);

        self
    }

    fn connect_pin(
        pin: impl Peripheral<P = impl PeripheralOutput> + 'd,
        input: InputSignal,
        output: OutputSignal,
        guard: &mut PinGuard,
    ) {
        crate::into_mapped_ref!(pin);
        // avoid the pin going low during configuration
        pin.set_output_high(true);

        pin.set_to_open_drain_output();
        pin.enable_input(true);
        pin.pull_direction(Pull::Up);

        input.connect_to(pin.reborrow());

        *guard = OutputConnection::connect_with_guard(pin, output);
    }
}

impl<'d> I2c<'d, Blocking> {
    /// Create a new I2C instance.
    pub fn new(
        i2c: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        crate::into_mapped_ref!(i2c);

        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let sda_pin = PinGuard::new_unconnected(i2c.info().sda_output);
        let scl_pin = PinGuard::new_unconnected(i2c.info().scl_output);

        let i2c = I2c {
            i2c,
            phantom: PhantomData,
            config,
            guard,
            sda_pin,
            scl_pin,
        };

        i2c.driver().setup(&i2c.config)?;

        Ok(i2c)
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
        self.i2c.info().set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.info().enable_listen(interrupts.into(), true)
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.info().enable_listen(interrupts.into(), false)
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<Event> {
        self.i2c.info().interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<Event>) {
        self.i2c.info().clear_interrupts(interrupts)
    }

    /// Configures the I2C peripheral to operate in asynchronous mode.
    pub fn into_async(mut self) -> I2c<'d, Async> {
        self.set_interrupt_handler(self.driver().info.async_handler);

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
            guard: self.guard,
            sda_pin: self.sda_pin,
            scl_pin: self.scl_pin,
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
        self.driver()
            .write_blocking(address.into(), buffer, true, true)
            .inspect_err(|_| self.internal_recover())
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
        self.driver()
            .read_blocking(address.into(), buffer, true, true, false)
            .inspect_err(|_| self.internal_recover())
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

        self.driver()
            .write_blocking(address, write_buffer, true, read_buffer.is_empty())
            .inspect_err(|_| self.internal_recover())?;

        self.driver()
            .read_blocking(address, read_buffer, true, true, false)
            .inspect_err(|_| self.internal_recover())?;

        Ok(())
    }

    /// Execute the provided operations on the I2C bus.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This
    ///   is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each
    ///   other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is
    ///   sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an
    ///   acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0
    ///   to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::i2c::master::{Config, I2c, Operation};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )
    /// # .unwrap();
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.transaction(
    ///     DEVICE_ADDR,
    ///     &mut [Operation::Write(&[0xaa]), Operation::Read(&mut data)]
    /// ).ok();
    /// # }
    /// ```
    pub fn transaction<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl(address.into(), operations.into_iter().map(Operation::from))
            .inspect_err(|_| self.internal_recover())
    }
}

impl private::Sealed for I2c<'_, Blocking> {}

impl InterruptConfigurable for I2c<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2c.info().set_interrupt_handler(handler);
    }
}

impl<'d> I2c<'d, Async> {
    /// Configure the I2C peripheral to operate in blocking mode.
    pub fn into_blocking(self) -> I2c<'d, Blocking> {
        self.i2c.info().disable_interrupts();

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
            guard: self.guard,
            sda_pin: self.sda_pin,
            scl_pin: self.scl_pin,
        }
    }

    /// Writes bytes to slave with address `address`
    pub async fn write<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &[u8],
    ) -> Result<(), Error> {
        self.driver()
            .write(address.into(), buffer, true, true)
            .await
            .inspect_err(|_| self.internal_recover())
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    pub async fn read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.driver()
            .read(address.into(), buffer, true, true, false)
            .await
            .inspect_err(|_| self.internal_recover())
    }

    /// Writes bytes to slave with address `address` and then reads enough
    /// bytes to fill `buffer` *in a single transaction*
    pub async fn write_read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let address = address.into();

        self.driver()
            .write(address, write_buffer, true, read_buffer.is_empty())
            .await
            .inspect_err(|_| self.internal_recover())?;

        self.driver()
            .read(address, read_buffer, true, true, false)
            .await
            .inspect_err(|_| self.internal_recover())?;

        Ok(())
    }

    /// Execute the provided operations on the I2C bus as a single
    /// transaction.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This
    ///   is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each
    ///   other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is
    ///   sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an
    ///   acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0
    ///   to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    pub async fn transaction<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl_async(address.into(), operations.into_iter().map(Operation::from))
            .await
            .inspect_err(|_| self.internal_recover())
    }

    async fn transaction_impl_async<'a>(
        &mut self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.driver()
                        .write(
                            address,
                            buffer,
                            !matches!(last_op, Some(OpKind::Write)),
                            next_op.is_none(),
                        )
                        .await?;
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.driver()
                        .read(
                            address,
                            buffer,
                            !matches!(last_op, Some(OpKind::Read)),
                            next_op.is_none(),
                            matches!(next_op, Some(OpKind::Read)),
                        )
                        .await?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }
}

impl embedded_hal_async::i2c::I2c for I2c<'_, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [EhalOperation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl_async(address.into(), operations.iter_mut().map(Operation::from))
            .await
            .inspect_err(|_| self.internal_recover())
    }
}
