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
pub struct I2c<'d> {
    pub(crate) i2c: PeripheralRef<'d, AnyI2c>,
    pub(crate) config: Config,
    pub(crate) guard: PeripheralGuard,
    pub(crate) sda_pin: PinGuard,
    pub(crate) scl_pin: PinGuard,
}

//pub(crate) trait I2cDevice<'d> {
//    fn driver(&self) -> Driver<'_>;
//    fn internal_recover(&self);
//    fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError>;
//    fn with_sda(self, sda: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self;
//    fn with_scl(self, scl: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self;
//}

impl<'d> I2c<'d> {
    pub(crate) fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
        }
    }

    pub(crate) fn connect_pin(
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

    pub(crate) fn internal_recover(&self) {
        PeripheralClockControl::disable(self.driver().info.peripheral);
        PeripheralClockControl::enable(self.driver().info.peripheral);
        PeripheralClockControl::reset(self.driver().info.peripheral);

        // We know the configuration is valid, we can ignore the result.
        _ = self.driver().setup(&self.config);
    }

    pub(crate) fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().setup(config)?;
        self.config = *config;
        Ok(())
    }

    pub(crate) fn with_sda(&mut self, sda: impl Peripheral<P = impl PeripheralOutput> + 'd) {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Self::connect_pin(sda, input, output, &mut self.sda_pin);
    }

    pub(crate) fn with_scl(&mut self, scl: impl Peripheral<P = impl PeripheralOutput> + 'd) {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Self::connect_pin(scl, input, output, &mut self.scl_pin);
    }
}
