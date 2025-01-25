//! # Inter-Integrated Circuit (I2C)
//!
//! I2C is a serial, synchronous, multi-device, half-duplex communication
//! protocol that allows co-existence of multiple masters and slaves on the
//! same bus. I2C uses two bidirectional open-drain lines: serial data line
//! (SDA) and serial clock line (SCL), pulled up by resistors.
//!
//! For more information, see
#![doc = crate::trm_markdown_link!("i2c")]

mod driver;
pub mod info;
pub mod master;
pub mod slave;

#[cfg(lp_i2c0)]
crate::unstable_module! {
    pub mod lp_i2c;
}

use enumset::EnumSetType;
use fugit::HertzU32;

#[cfg(not(esp32))]
use core::{
    pin::Pin,
    task::{Context, Poll},
};

use crate::{
    asynch::AtomicWaker,
    gpio::{InputSignal, OutputSignal},
    i2c::info::Info,
    pac::i2c0::RegisterBlock,
    peripheral::Peripheral,
    peripherals::Interrupt,
};

// Chunk writes/reads by this size
#[cfg(any(esp32, esp32s2))]
pub(crate) const I2C_CHUNK_SIZE: usize = 32;

#[cfg(not(any(esp32, esp32s2)))]
pub(crate) const I2C_CHUNK_SIZE: usize = 254;

// on ESP32 there is a chance to get trapped in `wait_for_completion` forever
pub(crate) const MAX_ITERATIONS: u32 = 1_000_000;

/// Representation of I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum I2cAddress {
    /// 7-bit address mode type.
    ///
    /// Note that 7-bit addresses defined by drivers should be specified in
    /// **right-aligned** form, e.g. in the range `0x00..=0x7F`.
    ///
    /// For example, a device that has the seven bit address of `0b011_0010`,
    /// and therefore is addressed on the wire using:
    ///
    /// * `0b0110010_0` or `0x64` for *writes*
    /// * `0b0110010_1` or `0x65` for *reads*
    SevenBit(u8),
}

impl From<u8> for I2cAddress {
    fn from(value: u8) -> Self {
        I2cAddress::SevenBit(value)
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32s2)] {
        const I2C_LL_INTR_MASK: u32 = 0x1ffff;
    } else {
        const I2C_LL_INTR_MASK: u32 = 0x3ffff;
    }
}

/// I2C SCL timeout period.
///
/// When the level of SCL remains unchanged for more than `timeout` bus
/// clock cycles, the bus goes to idle state.
///
/// Default value is `BusCycles(10)`.
#[doc = ""]
#[cfg_attr(
    not(esp32),
    doc = "Note that the effective timeout may be longer than the value configured here."
)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, strum::Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
// TODO: when supporting interrupts, document that SCL = high also triggers an
// interrupt.
pub enum BusTimeout {
    /// Use the maximum timeout value.
    Maximum,

    /// Disable timeout control.
    #[cfg(not(any(esp32, esp32s2)))]
    Disabled,

    /// Timeout in bus clock cycles.
    BusCycles(u32),
}

impl BusTimeout {
    fn cycles(&self) -> u32 {
        match self {
            #[cfg(esp32)]
            BusTimeout::Maximum => 0xF_FFFF,

            #[cfg(esp32s2)]
            BusTimeout::Maximum => 0xFF_FFFF,

            #[cfg(not(any(esp32, esp32s2)))]
            BusTimeout::Maximum => 0x1F,

            #[cfg(not(any(esp32, esp32s2)))]
            BusTimeout::Disabled => 1,

            BusTimeout::BusCycles(cycles) => *cycles,
        }
    }

    #[cfg(not(esp32))]
    fn is_set(&self) -> bool {
        matches!(self, BusTimeout::BusCycles(_) | BusTimeout::Maximum)
    }
}

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The transmission exceeded the FIFO size.
    FifoExceeded,
    /// The acknowledgment check failed.
    AcknowledgeCheckFailed(AcknowledgeCheckFailedReason),
    /// A timeout occurred during transmission.
    Timeout,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
    /// The execution of the I2C command was incomplete.
    ExecutionIncomplete,
    /// The number of commands issued exceeded the limit.
    CommandNumberExceeded,
    /// Zero length read or write operation.
    ZeroLengthInvalid,
}

/// I2C no acknowledge error reason.
///
/// Consider this as a hint and make sure to always handle all cases.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AcknowledgeCheckFailedReason {
    /// The device did not acknowledge its address. The device may be missing.
    Address,

    /// The device did not acknowledge the data. It may not be ready to process
    /// requests at the moment.
    Data,

    /// Either the device did not acknowledge its address or the data, but it is
    /// unknown which.
    Unknown,
}

impl core::fmt::Display for AcknowledgeCheckFailedReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AcknowledgeCheckFailedReason::Address => write!(f, "Address"),
            AcknowledgeCheckFailedReason::Data => write!(f, "Data"),
            AcknowledgeCheckFailedReason::Unknown => write!(f, "Unknown"),
        }
    }
}

impl From<&AcknowledgeCheckFailedReason> for embedded_hal::i2c::NoAcknowledgeSource {
    fn from(value: &AcknowledgeCheckFailedReason) -> Self {
        match value {
            AcknowledgeCheckFailedReason::Address => {
                embedded_hal::i2c::NoAcknowledgeSource::Address
            }
            AcknowledgeCheckFailedReason::Data => embedded_hal::i2c::NoAcknowledgeSource::Data,
            AcknowledgeCheckFailedReason::Unknown => {
                embedded_hal::i2c::NoAcknowledgeSource::Unknown
            }
        }
    }
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::FifoExceeded => write!(f, "The transmission exceeded the FIFO size"),
            Error::AcknowledgeCheckFailed(reason) => {
                write!(f, "The acknowledgment check failed. Reason: {}", reason)
            }
            Error::Timeout => write!(f, "A timeout occurred during transmission"),
            Error::ArbitrationLost => write!(f, "The arbitration for the bus was lost"),
            Error::ExecutionIncomplete => {
                write!(f, "The execution of the I2C command was incomplete")
            }
            Error::CommandNumberExceeded => {
                write!(f, "The number of commands issued exceeded the limit")
            }
            Error::ZeroLengthInvalid => write!(f, "Zero length read or write operation"),
        }
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Provided bus frequency is invalid for the current configuration.
    FrequencyInvalid,
    /// Provided timeout is invalid for the current configuration.
    TimeoutInvalid,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::FrequencyInvalid => write!(
                f,
                "Provided bus frequency is invalid for the current configuration"
            ),
            ConfigError::TimeoutInvalid => write!(
                f,
                "Provided timeout is invalid for the current configuration"
            ),
        }
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::ErrorKind;

        match self {
            Self::FifoExceeded => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::AcknowledgeCheckFailed(reason) => ErrorKind::NoAcknowledge(reason.into()),
            _ => ErrorKind::Other,
        }
    }
}

/// A generic I2C Command
#[cfg_attr(feature = "debug", derive(Debug))]
enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

pub(crate) enum OperationType {
    Write = 0,
    Read = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub(crate) enum Ack {
    Ack = 0,
    Nack = 1,
}

impl From<u32> for Ack {
    fn from(ack: u32) -> Self {
        match ack {
            0 => Ack::Ack,
            1 => Ack::Nack,
            _ => unreachable!(),
        }
    }
}

impl From<Ack> for u32 {
    fn from(ack: Ack) -> u32 {
        ack as u32
    }
}

// This enum is used to keep track of the last/next operation that was/will be
// performed in an embedded-hal(-async) I2c::transaction. It is used to
// determine whether a START condition should be issued at the start of the
// current operation and whether a read needs an ack or a nack for the final
// byte.
#[derive(PartialEq)]
pub(crate) enum OpKind {
    Write,
    Read,
}

/// I2C operation.
///
/// Several operations can be combined as part of a transaction.
#[derive(Debug, PartialEq, Eq, Hash, strum::Display)]
pub enum Operation<'a> {
    /// Write data from the provided buffer.
    Write(&'a [u8]),

    /// Read data into the provided buffer.
    Read(&'a mut [u8]),
}

impl<'a, 'b> From<&'a mut embedded_hal::i2c::Operation<'b>> for Operation<'a> {
    fn from(value: &'a mut embedded_hal::i2c::Operation<'b>) -> Self {
        match value {
            embedded_hal::i2c::Operation::Write(buffer) => Operation::Write(buffer),
            embedded_hal::i2c::Operation::Read(buffer) => Operation::Read(buffer),
        }
    }
}

impl<'a, 'b> From<&'a mut Operation<'b>> for Operation<'a> {
    fn from(value: &'a mut Operation<'b>) -> Self {
        match value {
            Operation::Write(buffer) => Operation::Write(buffer),
            Operation::Read(buffer) => Operation::Read(buffer),
        }
    }
}

impl Operation<'_> {
    pub(crate) fn is_write(&self) -> bool {
        matches!(self, Operation::Write(_))
    }

    pub(crate) fn kind(&self) -> OpKind {
        match self {
            Operation::Write(_) => OpKind::Write,
            Operation::Read(_) => OpKind::Read,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        match self {
            Operation::Write(buffer) => buffer.is_empty(),
            Operation::Read(buffer) => buffer.is_empty(),
        }
    }
}

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C clock frequency.
    pub frequency: HertzU32,

    /// I2C SCL timeout period.
    pub timeout: BusTimeout,
}

impl core::hash::Hash for Config {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.frequency.to_Hz().hash(state); // `HertzU32` doesn't implement `Hash`
        self.timeout.hash(state);
    }
}

impl Default for Config {
    fn default() -> Self {
        use fugit::RateExtU32;
        Config {
            frequency: 100.kHz(),
            timeout: BusTimeout::BusCycles(10),
        }
    }
}

#[cfg_attr(esp32, allow(dead_code))]
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Event {
    /// Triggered when op_code of the master indicates an END command and an END
    /// condition is detected.
    EndDetect,

    /// Triggered when the I2C controller detects a STOP bit.
    TxComplete,

    /// Triggered when the TX FIFO watermark check is enabled and the TX fifo
    /// falls below the configured watermark.
    #[cfg(not(any(esp32, esp32s2)))]
    TxFifoWatermark,
}

#[cfg(not(esp32))]
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct I2cFuture<'a> {
    event: Event,
    info: &'a Info,
    state: &'a State,
}

#[cfg(not(esp32))]
impl<'a> I2cFuture<'a> {
    pub fn new(event: Event, info: &'a Info, state: &'a State) -> Self {
        info.regs().int_ena().modify(|_, w| {
            let w = match event {
                Event::EndDetect => w.end_detect().set_bit(),
                Event::TxComplete => w.trans_complete().set_bit(),
                #[cfg(not(any(esp32, esp32s2)))]
                Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
            };

            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w.nack().set_bit();

            w
        });

        Self { event, state, info }
    }

    fn event_bit_is_clear(&self) -> bool {
        let r = self.info.regs().int_ena().read();

        match self.event {
            Event::EndDetect => r.end_detect().bit_is_clear(),
            Event::TxComplete => r.trans_complete().bit_is_clear(),
            #[cfg(not(any(esp32, esp32s2)))]
            Event::TxFifoWatermark => r.txfifo_wm().bit_is_clear(),
        }
    }

    fn check_error(&self) -> Result<(), Error> {
        let r = self.info.regs().int_raw().read();

        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }

        if r.time_out().bit_is_set() {
            return Err(Error::Timeout);
        }

        if r.nack().bit_is_set() {
            return Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(
                self.info.regs(),
            )));
        }

        #[cfg(not(esp32))]
        if r.trans_complete().bit_is_set() && self.info.regs().sr().read().resp_rec().bit_is_clear()
        {
            return Err(Error::AcknowledgeCheckFailed(
                AcknowledgeCheckFailedReason::Data,
            ));
        }

        Ok(())
    }
}

#[cfg(not(esp32))]
impl core::future::Future for I2cFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        self.state.waker.register(ctx.waker());

        let error = self.check_error();

        if error.is_err() {
            return Poll::Ready(error);
        }

        if self.event_bit_is_clear() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

// Estimate the reason for an acknowledge check failure on a best effort basis.
// When in doubt it's better to return `Unknown` than to return a wrong reason.
pub(crate) fn estimate_ack_failed_reason(
    _register_block: &RegisterBlock,
) -> AcknowledgeCheckFailedReason {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32c2, esp32c3))] {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            // this is based on observations rather than documented behavior
            if _register_block.fifo_st().read().txfifo_raddr().bits() <= 1 {
                AcknowledgeCheckFailedReason::Address
            } else {
                AcknowledgeCheckFailedReason::Data
            }
        }
    }
}

/// Peripheral state for an I2C instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous operations.
    pub waker: AtomicWaker,
}

pub(crate) fn async_handler(info: &Info, state: &State) {
    let regs = info.regs();
    regs.int_ena().modify(|_, w| {
        w.end_detect().clear_bit();
        w.trans_complete().clear_bit();
        w.arbitration_lost().clear_bit();
        w.time_out().clear_bit();

        #[cfg(not(any(esp32, esp32s2)))]
        w.txfifo_wm().clear_bit();

        w.nack().clear_bit()
    });

    state.waker.wake();
}

/// Sets the filter with a supplied threshold in clock cycles for which a
/// pulse must be present to pass the filter
pub(crate) fn set_filter(
    register_block: &RegisterBlock,
    sda_threshold: Option<u8>,
    scl_threshold: Option<u8>,
) {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            register_block.sda_filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some())
            });
            register_block.scl_filter_cfg().modify(|_, w| {
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        } else {
            register_block.filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some());
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        }
    }
}

#[allow(clippy::too_many_arguments, unused)]
/// Configures the clock and timing parameters for the I2C peripheral.
pub(crate) fn configure_clock(
    register_block: &RegisterBlock,
    sclk_div: u32,
    scl_low_period: u32,
    scl_high_period: u32,
    scl_wait_high_period: u32,
    sda_hold_time: u32,
    sda_sample_time: u32,
    scl_rstart_setup_time: u32,
    scl_stop_setup_time: u32,
    scl_start_hold_time: u32,
    scl_stop_hold_time: u32,
    timeout: BusTimeout,
) -> Result<(), ConfigError> {
    unsafe {
        // divider
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        register_block.clk_conf().modify(|_, w| {
            w.sclk_sel().clear_bit();
            w.sclk_div_num().bits((sclk_div - 1) as u8)
        });

        // scl period
        register_block
            .scl_low_period()
            .write(|w| w.scl_low_period().bits(scl_low_period as u16));

        #[cfg(not(esp32))]
        let scl_wait_high_period = scl_wait_high_period
            .try_into()
            .map_err(|_| ConfigError::FrequencyInvalid)?;

        register_block.scl_high_period().write(|w| {
            #[cfg(not(esp32))] // ESP32 does not have a wait_high field
            w.scl_wait_high_period().bits(scl_wait_high_period);
            w.scl_high_period().bits(scl_high_period as u16)
        });

        // sda sample
        register_block
            .sda_hold()
            .write(|w| w.time().bits(sda_hold_time as u16));
        register_block
            .sda_sample()
            .write(|w| w.time().bits(sda_sample_time as u16));

        // setup
        register_block
            .scl_rstart_setup()
            .write(|w| w.time().bits(scl_rstart_setup_time as u16));
        register_block
            .scl_stop_setup()
            .write(|w| w.time().bits(scl_stop_setup_time as u16));

        // hold
        register_block
            .scl_start_hold()
            .write(|w| w.time().bits(scl_start_hold_time as u16));
        register_block
            .scl_stop_hold()
            .write(|w| w.time().bits(scl_stop_hold_time as u16));

        // The ESP32 variant does not have an enable flag for the
        // timeout mechanism
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                register_block
                    .to()
                    .write(|w| w.time_out().bits(timeout.cycles()));
            } else {
                register_block
                    .to()
                    .write(|w| w.time_out_en().bit(timeout.is_set())
                    .time_out_value()
                    .bits(timeout.cycles() as _)
                );
            }
        }
    }
    Ok(())
}

/// I2C Peripheral Instance
#[doc(hidden)]
pub trait Instance: Peripheral<P = Self> + Into<AnyI2c> + 'static {
    /// Returns the peripheral data and state describing this instance.
    fn parts(&self) -> (&Info, &State);

    /// Returns the peripheral data describing this instance.
    #[inline(always)]
    fn info(&self) -> &Info {
        self.parts().0
    }

    /// Returns the peripheral state for this instance.
    #[inline(always)]
    fn state(&self) -> &State {
        self.parts().1
    }
}

macro_rules! instance {
    ($inst:ident, $peri:ident, $scl:ident, $sda:ident, $interrupt:ident) => {
        impl Instance for crate::peripherals::$inst {
            fn parts(&self) -> (&Info, &State) {
                #[crate::handler]
                pub(super) fn irq_handler() {
                    async_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State {
                    waker: AtomicWaker::new(),
                };

                static PERIPHERAL: Info = Info {
                    register_block: crate::peripherals::$inst::ptr(),
                    peripheral: crate::system::Peripheral::$peri,
                    async_handler: irq_handler,
                    interrupt: Interrupt::$interrupt,
                    scl_output: OutputSignal::$scl,
                    scl_input: InputSignal::$scl,
                    sda_output: OutputSignal::$sda,
                    sda_input: InputSignal::$sda,
                };
                (&PERIPHERAL, &STATE)
            }
        }
    };
}

#[cfg(i2c0)]
instance!(I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA, I2C_EXT0);
#[cfg(i2c1)]
instance!(I2C1, I2cExt1, I2CEXT1_SCL, I2CEXT1_SDA, I2C_EXT1);

crate::any_peripheral! {
    /// Represents any I2C peripheral.
    pub peripheral AnyI2c {
        #[cfg(i2c0)]
        I2c0(crate::peripherals::I2C0),
        #[cfg(i2c1)]
        I2c1(crate::peripherals::I2C1),
    }
}

impl Instance for AnyI2c {
    delegate::delegate! {
        to match &self.0 {
            AnyI2cInner::I2c0(i2c) => i2c,
            #[cfg(i2c1)]
            AnyI2cInner::I2c1(i2c) => i2c,
        } {
            fn parts(&self) -> (&Info, &State);
        }
    }
}
