//! Configuration for async I2C slave driver

use super::{ConfigError, I2C_FIFO_SIZE, I2cAddress};

/// I2C slave driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C slave address.
    ///
    /// Supports both 7-bit (0x00..=0x7F) and 10-bit (0x000..=0x3FF) addresses.
    ///
    /// Default value: 7-bit address 0x55.
    pub address: I2cAddress,

    /// Enable clock stretching.
    ///
    /// When enabled, the slave can hold SCL low to pause the master while
    /// processing data or preparing a response.
    ///
    /// **Warning**: Some masters (notably ESP32 original) have poor clock
    /// stretching support and may timeout or hang when slave stretches SCL.
    ///
    /// Default value: true.
    pub clock_stretch_enable: bool,

    /// Enable SDA filtering.
    ///
    /// Default value: true.
    pub sda_filter_enable: bool,

    /// SDA filter threshold (in APB clock cycles).
    ///
    /// Pulses shorter than this threshold are filtered out.
    ///
    /// Default value: 7.
    pub sda_filter_threshold: u8,

    /// Enable SCL filtering.
    ///
    /// Default value: true.
    pub scl_filter_enable: bool,

    /// SCL filter threshold (in APB clock cycles).
    ///
    /// Pulses shorter than this threshold are filtered out.
    ///
    /// Default value: 7.
    pub scl_filter_threshold: u8,

    /// RX FIFO threshold for interrupt generation.
    ///
    /// Interrupt fires when RX FIFO contains at least this many bytes.
    /// Must be <= FIFO size (typically 32 bytes).
    ///
    /// Lower values = more frequent interrupts but faster response.
    /// Higher values = fewer interrupts but risk of FIFO overflow.
    ///
    /// Recommended: 16 (half of FIFO size) for balanced performance.
    ///
    /// Default value: 16.
    pub rx_fifo_threshold: u8,

    /// TX FIFO threshold for interrupt generation.
    ///
    /// Interrupt fires when TX FIFO has at least this much free space.
    /// Must be <= FIFO size (typically 32 bytes).
    ///
    /// Lower values = more frequent interrupts but prevents underflow.
    /// Higher values = fewer interrupts but may cause clock stretching.
    ///
    /// Recommended: 16 (half of FIFO size) for balanced performance.
    ///
    /// Default value: 16.
    pub tx_fifo_threshold: u8,

    /// Enable register-based mode (FIFO address configuration).
    ///
    /// When enabled, the first byte received after the slave address is treated
    /// as a "register address" and stored in a separate internal register (not
    /// the main RX FIFO). This is useful for emulating register-based I2C devices
    /// like sensors that use register addressing.
    ///
    /// When disabled (default), all received bytes are stored sequentially in
    /// the RX FIFO (raw data stream mode).
    ///
    /// **Supported devices**: Currently only ESP32-C6 is confirmed to support
    /// this feature. The setting is only available when compiling for ESP32-C6.
    ///
    /// Default value: false (raw data stream mode).
    #[cfg(esp32c6)]
    pub register_based_mode: bool,

    /// Timeout duration for operations in milliseconds.
    ///
    /// This is used as a safeguard to prevent infinite waiting in error conditions.
    ///
    /// Default value: 1000 (1 second).
    pub timeout_ms: u32,

    /// Interrupt priority.
    ///
    /// Higher priority interrupts preempt lower priority ones.
    /// Be careful not to set too high as this may affect system responsiveness.
    ///
    /// Default value: Priority::Priority1 (medium).
    pub interrupt_priority: crate::interrupt::Priority,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            address: I2cAddress::SevenBit(0x55),
            clock_stretch_enable: true,
            sda_filter_enable: true,
            sda_filter_threshold: 7,
            scl_filter_enable: true,
            scl_filter_threshold: 7,
            rx_fifo_threshold: 16,
            tx_fifo_threshold: 16,
            #[cfg(esp32c6)]
            register_based_mode: false,
            timeout_ms: 1000,
            interrupt_priority: crate::interrupt::Priority::Priority1,
        }
    }
}

impl Config {
    /// Validate the configuration
    pub fn validate(&self) -> Result<(), ConfigError> {
        // Validate address
        self.address
            .validate()
            .map_err(|_| ConfigError::AddressInvalid)?;

        // Validate FIFO thresholds
        if self.rx_fifo_threshold as usize > I2C_FIFO_SIZE {
            return Err(ConfigError::InvalidFifoThreshold);
        }
        if self.tx_fifo_threshold as usize > I2C_FIFO_SIZE {
            return Err(ConfigError::InvalidFifoThreshold);
        }

        Ok(())
    }
}
