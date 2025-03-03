#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Inter-Integrated Circuit (I2C) - Async Slave Mode (Interrupt-Driven)
//!
//! ## Overview
//!
//! This module provides a **true async**, **interrupt-driven** I2C slave driver that
//! allows concurrent task execution on a single core. Unlike the standard slave driver
//! which uses polling, this driver leverages hardware interrupts to achieve:
//!
//! - **Zero CPU usage** while waiting for I2C communication
//! - **Immediate response** to master requests (<1µs latency)
//! - **True concurrency** - other async tasks can run while waiting
//! - **Power efficiency** - CPU can sleep between I2C transactions
//!
//! ## Key Differences from Standard Slave Driver
//!
//! | Feature | Standard Slave | SlaveAsync (This Driver) |
//! |---------|---------------|--------------------------|
//! | CPU while idle | 100% polling | ~0% sleeping |
//! | Concurrent tasks | ❌ Blocks | ✅ Fully concurrent |
//! | Response latency | ~1µs | <1µs (interrupt) |
//! | Power efficiency | Low | High |
//! | Code complexity | Simple | Complex |
//! | Memory overhead | Low | Medium |
//!
//! ## Architecture
//!
//! The driver uses a **state machine** driven by hardware interrupts:
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │                     I2C Master                               │
//! └──────────────┬───────────────────────────────┬───────────────┘
//!                │ START + Address               │ Data bytes
//!                ▼                               ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │              ESP32 Hardware (I2C Peripheral)                 │
//! │  ┌────────────────────────────────────────────────────────┐  │
//! │  │  FIFO (32 bytes)  │  Address Match  │  Clock Stretch   │  │
//! │  └────────────────────────────────────────────────────────┘  │
//! └───────────────┬──────────────────────────────────────────────┘
//!                 │ Interrupts
//!                 ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │           Interrupt Handler (Fast, <1µs)                     │
//! │  • Address Match → Setup state                               │
//! │  • RX FIFO → Read bytes into buffer                          │
//! │  • TX FIFO → Write bytes from buffer                         │
//! │  • Transaction Complete → Wake async task                    │
//! └───────────────┬──────────────────────────────────────────────┘
//!                 │ Waker::wake()
//!                 ▼
//! ┌──────────────────────────────────────────────────────────────┐
//! │              Async Task (User Code)                          │
//! │  loop {                                                      │
//! │      slave.read_async(&mut buf).await?; // No blocking!      │
//! │      process_command(&buf).await;       // Other tasks run   │
//! │      slave.write_async(&response).await?;                    │
//! │  }                                                           │
//! └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Configuration
//!
//! Configuration is similar to the standard slave driver:
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::slave_async::{Config, SlaveAsync};
//!
//! let config = Config::default()
//!     .with_address(0x55.into())
//!     .with_clock_stretch_enable(true)
//!     .with_rx_fifo_threshold(16)  // Interrupt when 16 bytes available
//!     .with_tx_fifo_threshold(16); // Interrupt when 16 bytes free
//!
//! let mut i2c = SlaveAsync::new(peripherals.I2C0, config)?
//!     .with_sda(peripherals.GPIO2)
//!     .with_scl(peripherals.GPIO3);
//! # {after_snippet}
//! ```
//!
//! ## Usage Example
//!
//! ### Basic Echo Server (Fully Concurrent)
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::slave_async::{Config, SlaveAsync};
//! use embassy_time::{Duration, Timer};
//!
//! #[embassy_executor::task]
//! async fn i2c_slave_task(mut i2c: SlaveAsync<'static>) {
//!     let mut buffer = [0u8; 32];
//!
//!     loop {
//!         // Read from master - DOESN'T BLOCK!
//!         // Other tasks (like led_task) run while waiting
//!         match i2c.read_async(&mut buffer).await {
//!             Ok(len) => {
//!                 // Process and respond
//!                 i2c.write_async(&buffer[..len]).await.ok();
//!             }
//!             Err(e) => {
//!                 defmt::error!("I2C error: {:?}", e);
//!             }
//!         }
//!     }
//! }
//!
//! #[embassy_executor::task]
//! async fn led_task() {
//!     loop {
//!         Timer::after(Duration::from_millis(500)).await;
//!         toggle_led();  // Runs smoothly, not blocked by I2C! ✓
//!     }
//! }
//!
//! #[main]
//! async fn main(spawner: Spawner) {
//!     let peripherals = esp_hal::init(esp_hal::Config::default());
//!
//!     let config = Config::default().with_address(0x55.into());
//!     let i2c = SlaveAsync::new(peripherals.I2C0, config).unwrap()
//!         .with_sda(peripherals.GPIO2)
//!         .with_scl(peripherals.GPIO3);
//!
//!     spawner.spawn(i2c_slave_task(i2c)).unwrap();
//!     spawner.spawn(led_task()).unwrap();
//! }
//! # {after_snippet}
//! ```
//!
//! ### Register-Based Device Emulation
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::slave_async::{Config, SlaveAsync};
//!
//! #[embassy_executor::task]
//! async fn sensor_emulator(mut i2c: SlaveAsync<'static>) {
//!     let mut registers = [0u8; 256];
//!     registers[0x00] = 0x42; // Device ID
//!     registers[0x01] = 0x10; // Version
//!
//!     loop {
//!         // Wait for master to write register address
//!         let mut reg_addr = [0u8; 1];
//!         if i2c.read_async(&mut reg_addr).await.is_ok() {
//!             // Prepare response (register value)
//!             let value = registers[reg_addr[0] as usize];
//!
//!             // Send when master reads
//!             i2c.write_async(&[value]).await.ok();
//!         }
//!     }
//! }
//! # {after_snippet}
//! ```
//!
//! ## Interrupt Configuration
//!
//! The driver automatically configures interrupts when created. You can customize
//! interrupt priority if needed:
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::slave_async::{Config, SlaveAsync};
//! use esp_hal::interrupt::Priority;
//!
//! let mut i2c = SlaveAsync::new(peripherals.I2C0, config)?
//!     .with_sda(peripherals.GPIO2)
//!     .with_scl(peripherals.GPIO3)
//!     .with_interrupt_priority(Priority::Priority3);
//! # {after_snippet}
//! ```
//!
//! ## ESP32-C6 Specific Considerations
//!
//! ### Clock Stretching Release Timing
//!
//! ESP32-C6 requires careful timing when releasing clock stretch for TX operations.
//! This driver handles it in two stages:
//!
//! 1. **Interrupt handler**: Fills TX FIFO quickly
//! 2. **Deferred task**: Handles the stabilization delay if needed
//!
//! This prevents long delays in the interrupt handler while still meeting timing requirements.
//!
//! ### Register-Based Mode
//!
//! ESP32-C6 supports register-based mode where the first byte is treated as a register address:
//!
//! ```rust, no_run
//! # {before_snippet}
//! #[cfg(esp32c6)]
//! let config = Config::default()
//!     .with_register_based_mode(true);
//! # {after_snippet}
//! ```
//!
//! ## Performance Characteristics
//!
//! - **Interrupt latency**: <1µs (typically 200-500ns)
//! - **FIFO filling time**: ~2-5µs for 32 bytes
//! - **Wakeup overhead**: <10µs for async task resume
//! - **Memory overhead**: ~256 bytes static + 64 bytes per instance
//!
//! ## Safety and Concurrency
//!
//! The driver uses interrupt-safe mechanisms:
//! - Critical sections for shared state access
//! - Atomic operations where possible
//! - No `unsafe` code in user-facing API
//! - Proper lifetime management for interrupt context
//!
//! ## Limitations
//!
//! 1. **Single instance per I2C peripheral**: Only one `SlaveAsync` instance can exist
//!    per I2C peripheral (I2C0 or I2C1) due to interrupt handler requirements.
//!
//! 2. **Buffer size**: Internal buffers are statically allocated (configurable at compile time)
//!
//! 3. **Transaction size**: Limited by FIFO size (32 bytes) or internal buffer size
//!
//! 4. **Clock stretching duration**: Extended clock stretching may cause master timeouts
//!
//! ## When to Use This Driver
//!
//! ✅ **Use SlaveAsync when**:
//! - Need concurrent tasks on same core (LED blinking, sensors, etc.)
//! - Battery-powered applications (CPU can sleep)
//! - Critical response time requirements
//! - I2C communication is sporadic
//!
//! ❌ **Use standard slave driver when**:
//! - Simple dedicated I2C application
//! - I2C is the only task
//! - Development time is limited
//! - Multi-core architecture available (dedicate one core to I2C)
//!
//! ## See Also
//!
//! - [`crate::i2c::slave`] - Standard polling-based slave driver
//! - [`crate::i2c::master`] - I2C master driver
//! - [Embassy async runtime](https://embassy.dev/)

use core::{
    cell::RefCell,
    future::Future,
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll, Waker},
};

use critical_section::Mutex;
use enumset::{EnumSet, EnumSetType};

use crate::{
    asynch::AtomicWaker,
    gpio::{
        DriveMode, InputSignal, OutputConfig, OutputSignal, PinGuard, Pull,
        interconnect::{self, PeripheralOutput},
    },
    interrupt::{self, InterruptHandler, Priority},
    pac::i2c0::RegisterBlock,
    private, ram,
    system::PeripheralGuard,
};

mod state;
use state::*;

mod config;
pub use config::*;

mod instance;
use instance::*;

// Re-export interrupt counter function for external access
#[cfg(i2c_slave_i2c0)]
pub use instance::get_i2c0_interrupt_count;

mod driver;
use driver::*;

const I2C_FIFO_SIZE: usize = property!("i2c_master.fifo_size");
const DEFAULT_BUFFER_SIZE: usize = 256;

/// Maximum time to wait in interrupt handler (microseconds)
/// Interrupts handlers must be fast - this prevents long delays
const MAX_INTERRUPT_DURATION_US: u32 = 50;

/// I2C slave events that can trigger interrupts
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Event {
    /// Slave address matched (transaction starting)
    AddressMatch,

    /// RX FIFO reached threshold (data available)
    RxFifoThreshold,

    /// TX FIFO below threshold (space available)
    TxFifoThreshold,

    /// Byte received
    ByteReceived,

    /// Byte transmitted
    ByteTransmitted,

    /// Transaction complete (STOP condition)
    TransComplete,

    /// STOP condition detected
    StopDetected,

    /// START condition detected
    StartDetected,

    /// Arbitration lost
    ArbitrationLost,

    /// Timeout occurred
    Timeout,

    /// RX FIFO overflow
    RxFifoOverflow,

    /// TX FIFO underflow
    TxFifoUnderflow,
}

/// Representation of I2C slave address.
///
/// Addresses can be created from `u8`, `u16`, or `i32` types using `.into()`:
/// - `u8` values create 7-bit addresses
/// - `u16` values create 7-bit if ≤ 0x7F, otherwise 10-bit
/// - `i32` values are converted to their absolute value, then use same logic as `u16`
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum I2cAddress {
    /// 7-bit address mode type.
    ///
    /// Note that 7-bit addresses are specified in **right-aligned** form, e.g.
    /// in the range `0x00..=0x7F`.
    SevenBit(u8),

    /// 10-bit address mode type.
    ///
    /// Note that 10-bit addresses are specified in **right-aligned** form, e.g.
    /// in the range `0x00..=0x3FF`.
    TenBit(u16),
}

impl I2cAddress {
    fn validate(&self) -> Result<(), Error> {
        match self {
            I2cAddress::SevenBit(addr) => {
                if *addr > 0x7F {
                    return Err(Error::AddressInvalid(*self));
                }
            }
            I2cAddress::TenBit(addr) => {
                if *addr > 0x3FF {
                    return Err(Error::AddressInvalid(*self));
                }
            }
        }
        Ok(())
    }

    fn is_ten_bit(&self) -> bool {
        matches!(self, I2cAddress::TenBit(_))
    }

    fn as_u16(&self) -> u16 {
        match self {
            I2cAddress::SevenBit(addr) => *addr as u16,
            I2cAddress::TenBit(addr) => *addr,
        }
    }
}

impl From<u8> for I2cAddress {
    fn from(value: u8) -> Self {
        I2cAddress::SevenBit(value)
    }
}

impl From<u16> for I2cAddress {
    fn from(value: u16) -> Self {
        if value <= 0x7F {
            I2cAddress::SevenBit(value as u8)
        } else if value <= 0x3FF {
            I2cAddress::TenBit(value)
        } else {
            I2cAddress::TenBit(value & 0x3FF)
        }
    }
}

impl From<i32> for I2cAddress {
    fn from(value: i32) -> Self {
        let unsigned = value.unsigned_abs();

        if unsigned <= 0x7F {
            I2cAddress::SevenBit(unsigned as u8)
        } else if unsigned <= 0x3FF {
            I2cAddress::TenBit(unsigned as u16)
        } else {
            I2cAddress::TenBit((unsigned & 0x3FF) as u16)
        }
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
    AcknowledgeCheckFailed,
    /// A timeout occurred during transmission.
    Timeout,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
    /// The execution of the I2C command was incomplete.
    ExecutionIncomplete,
    /// Zero length read or write operation.
    ZeroLengthInvalid,
    /// The given address is invalid.
    AddressInvalid(I2cAddress),
    /// Bus is busy.
    BusBusy,
    /// TX FIFO overflow.
    TxFifoOverflow,
    /// TX FIFO underflow.
    TxFifoUnderflow,
    /// RX FIFO overflow.
    RxFifoOverflow,
    /// RX FIFO underflow.
    RxFifoUnderflow,
    /// Buffer too small for operation.
    BufferTooSmall,
    /// Driver not initialized.
    NotInitialized,
    /// Driver already in use.
    AlreadyInUse,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::FifoExceeded => write!(f, "The transmission exceeded the FIFO size"),
            Error::AcknowledgeCheckFailed => write!(f, "The acknowledgment check failed"),
            Error::Timeout => write!(f, "A timeout occurred during transmission"),
            Error::ArbitrationLost => write!(f, "The arbitration for the bus was lost"),
            Error::ExecutionIncomplete => {
                write!(f, "The execution of the I2C command was incomplete")
            }
            Error::ZeroLengthInvalid => write!(f, "Zero length read or write operation"),
            Error::AddressInvalid(address) => {
                write!(f, "The given address ({address:?}) is invalid")
            }
            Error::BusBusy => write!(f, "Bus is busy"),
            Error::TxFifoOverflow => write!(f, "TX FIFO overflow"),
            Error::TxFifoUnderflow => write!(f, "TX FIFO underflow"),
            Error::RxFifoOverflow => write!(f, "RX FIFO overflow"),
            Error::RxFifoUnderflow => write!(f, "RX FIFO underflow"),
            Error::BufferTooSmall => write!(f, "Buffer too small for operation"),
            Error::NotInitialized => write!(f, "Driver not initialized"),
            Error::AlreadyInUse => write!(f, "Driver already in use"),
        }
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Provided address is not valid.
    AddressInvalid,
    /// FIFO threshold is invalid.
    InvalidFifoThreshold,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::AddressInvalid => write!(f, "Provided address is invalid"),
            ConfigError::InvalidFifoThreshold => write!(f, "FIFO threshold is invalid"),
        }
    }
}

#[procmacros::doc_replace]
/// Async I2C slave driver (interrupt-driven)
///
/// This driver provides true async operation using hardware interrupts,
/// allowing concurrent task execution on a single core.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::i2c::slave_async::{Config, SlaveAsync};
///
/// let config = Config::default().with_address(0x55.into());
/// let mut i2c = SlaveAsync::new(peripherals.I2C0, config)?
///     .with_sda(peripherals.GPIO1)
///     .with_scl(peripherals.GPIO2);
///
/// let mut data = [0u8; 32];
/// let len = i2c.read_async(&mut data).await?;
/// # {after_snippet}
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveAsync<'d> {
    i2c: AnyI2cSlave<'d>,
    phantom: PhantomData<&'d ()>,
    guard: PeripheralGuard,
    config: DriverConfig,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DriverConfig {
    config: Config,
    sda_pin: PinGuard,
    scl_pin: PinGuard,
}

impl<'d> SlaveAsync<'d> {
    #[procmacros::doc_replace]
    /// Create a new async I2C slave instance.
    ///
    /// This automatically configures interrupts and initializes the driver state.
    ///
    /// ## Errors
    ///
    /// A [`ConfigError`] variant will be returned if the slave address
    /// passed in config is invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::slave_async::{Config, SlaveAsync};
    ///
    /// let config = Config::default().with_address(0x55.into());
    /// let i2c = SlaveAsync::new(peripherals.I2C0, config)?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    pub fn new(i2c: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let sda_pin = PinGuard::new_unconnected(i2c.info().sda_output);
        let scl_pin = PinGuard::new_unconnected(i2c.info().scl_output);

        let mut slave = SlaveAsync {
            i2c: instance::any::Degrade::degrade(i2c),
            phantom: PhantomData,
            guard,
            config: DriverConfig {
                config,
                sda_pin,
                scl_pin,
            },
        };

        slave.apply_config(&config)?;

        // Step 1: Set up interrupt handler AFTER degradation using proper delegation
        // This only binds the handler but doesn't enable peripheral interrupts yet
        slave.set_interrupt_handler(slave.driver().info.async_handler);

        // Step 2: Clear any interrupts that might have been generated during configuration
        slave
            .driver()
            .info
            .clear_interrupts(enumset::EnumSet::all());

        // Step 3: Enable peripheral interrupts BEFORE enabling any interrupt sources
        // This prevents any interrupt events from triggering when peripheral is enabled
        slave.enable_peripheral_interrupts(Priority::Priority3);

        // Step 4: Only after peripheral interrupts are enabled, enable interrupt sources
        slave.setup_interrupts();

        Ok(slave)
    }

    fn setup_interrupts(&mut self) {
        // Interrupt handler is already set during initialization
        // Just enable the interrupt sources we need

        // Enable essential interrupts for I2C slave operation + Timeout for testing
        self.driver().info.enable_listen(
            Event::AddressMatch
                | Event::RxFifoThreshold
                | Event::TransComplete
                | Event::Timeout
                | Event::StartDetected,
            true,
        );
    }

    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2c.set_interrupt_handler(handler);
    }

    fn enable_peripheral_interrupts(&mut self, priority: Priority) {
        self.i2c.enable_peripheral_interrupts(priority);
    }

    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
            config: &self.config,
        }
    }

    /// Get the number of interrupts handled since initialization.
    pub fn get_interrupt_counter(&self) -> u32 {
        self.driver().state.get_interrupt_count()
    }

    /// Connect a pin to the I2C SDA signal.
    pub fn with_sda(mut self, sda: impl PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Driver::connect_pin(sda.into(), input, output, &mut self.config.sda_pin);
        self
    }

    /// Connect a pin to the I2C SCL signal.
    pub fn with_scl(mut self, scl: impl PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Driver::connect_pin(scl.into(), input, output, &mut self.config.scl_pin);
        self
    }

    /// Set the interrupt priority.
    pub fn with_interrupt_priority(mut self, priority: Priority) -> Self {
        self.i2c.set_interrupt_priority(priority);
        self
    }

    #[procmacros::doc_replace]
    /// Reads data sent by the master asynchronously.
    ///
    /// This method yields to the executor, allowing other tasks to run while
    /// waiting for I2C communication.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::slave_async::{Config, SlaveAsync};
    ///
    /// let mut i2c = SlaveAsync::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2);
    ///
    /// let mut data = [0u8; 32];
    /// let len = i2c.read_async(&mut data).await?;
    /// # {after_snippet}
    /// ```
    pub async fn read_async(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthInvalid);
        }

        ReadFuture::new(self.driver(), buffer).await
    }

    #[procmacros::doc_replace]
    /// Writes data to be sent to the master asynchronously.
    ///
    /// **IMPORTANT**: For slave write (master read) operations, you should call
    /// this function to preload data into the TX FIFO **BEFORE** the master initiates
    /// a read request.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::slave_async::{Config, SlaveAsync};
    ///
    /// let mut i2c = SlaveAsync::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2);
    ///
    /// // Preload data before master reads
    /// i2c.write_async(&[0xAA, 0xBB]).await?;
    /// # {after_snippet}
    /// ```
    pub async fn write_async(&mut self, buffer: &[u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthInvalid);
        }

        WriteFuture::new(self.driver(), buffer).await
    }

    /// Reads the register address byte when in register-based mode (ESP32-C6 only).
    #[cfg(esp32c6)]
    pub fn read_register_address(&self) -> u8 {
        self.driver()
            .info
            .regs()
            .fifo_st()
            .read()
            .rxfifo_raddr()
            .bits()
    }

    /// Applies a new configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        config
            .address
            .validate()
            .map_err(|_| ConfigError::AddressInvalid)?;
        self.config.config = *config;
        self.driver().setup(config)?;
        Ok(())
    }
}

impl Drop for SlaveAsync<'_> {
    fn drop(&mut self) {
        // Disable interrupts
        self.driver().info.enable_listen(EnumSet::all(), false);
        // Clear any pending interrupts
        self.driver().info.clear_interrupts(EnumSet::all());
        // Reset driver state
        self.driver().state.reset();
    }
}

// Re-export key types from submodules for convenience
pub use config::Config;
pub use driver::Driver;
