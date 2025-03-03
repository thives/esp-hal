//! Core driver implementation with interrupt handling

use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};

use crate::{
    gpio::{DriveMode, InputSignal, OutputConfig, OutputSignal, PinGuard, Pull, interconnect},
    pac::i2c0::RegisterBlock,
    ram,
};

use super::{Config, ConfigError, DriverConfig, Error, Event, I2C_FIFO_SIZE, state::State};

/// Driver helper for accessing hardware and state
#[derive(Clone, Copy)]
pub struct Driver<'a> {
    pub(crate) info: &'a super::instance::Info,
    pub(crate) state: &'a State,
    pub(crate) config: &'a DriverConfig,
}

impl Driver<'_> {
    pub(crate) fn regs(&self) -> &RegisterBlock {
        self.info.regs()
    }

    pub(crate) fn connect_pin(
        pin: crate::gpio::interconnect::OutputSignal<'_>,
        input: InputSignal,
        output: OutputSignal,
        guard: &mut PinGuard,
    ) {
        pin.set_output_high(true);

        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::None),
        );
        pin.set_output_enable(true);
        pin.set_input_enable(true);

        input.connect_to(&pin);

        *guard = interconnect::OutputSignal::connect_with_guard(pin, output);
    }

    /// Initialize the peripheral in slave mode
    pub(crate) fn init_slave(&self) {
        #[cfg(esp32c6)]
        {
            // Reset the I2C peripheral
            self.regs().ctr().write(|w| unsafe { w.bits(0) });

            // Delay for reset
            for _ in 0..100 {
                unsafe { core::arch::asm!("nop") };
            }
        }

        self.regs().ctr().write(|w| {
            w.ms_mode().clear_bit(); // Slave mode
            w.conf_upgate().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();

            #[cfg(esp32s2)]
            w.ref_always_on().set_bit();

            w.slv_tx_auto_start_en().set_bit();

            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });

        #[cfg(not(any(esp32, esp32c6)))]
        self.regs().ctr().modify(|_, w| w.slave_addr_en().set_bit());

        #[cfg(esp32c6)]
        {
            // Configure ESP32-C6 specific slave settings (ms_mode already set above)
            self.regs()
                .ctr()
                .modify(|_, w| w.addr_broadcasting_en().clear_bit());
        }

        // Configure FIFO thresholds
        self.configure_fifo_thresholds();

        // Reset FIFO
        self.reset_fifo();
    }

    /// Configure FIFO thresholds based on config
    fn configure_fifo_thresholds(&self) {
        let rx_threshold = self.config.config.rx_fifo_threshold;
        let tx_threshold = self.config.config.tx_fifo_threshold;

        #[cfg(esp32c6)]
        {
            self.regs().fifo_conf().modify(|_, w| {
                unsafe {
                    w.rxfifo_wm_thrhd().bits(rx_threshold);
                    w.txfifo_wm_thrhd().bits(tx_threshold);
                }
                if self.config.config.register_based_mode {
                    w.fifo_addr_cfg_en().set_bit()
                } else {
                    w.fifo_addr_cfg_en().clear_bit()
                };
                w
            });
        }

        #[cfg(not(any(esp32, esp32c6)))]
        self.regs().fifo_conf().modify(|_, w| {
            unsafe {
                w.rxfifo_wm_thrhd().bits(rx_threshold);
                w.txfifo_wm_thrhd().bits(tx_threshold);
            }
            w
        });

        #[cfg(esp32)]
        self.regs().fifo_conf().modify(|_, w| {
            unsafe {
                w.rxfifo_full_thrhd().bits(rx_threshold);
                w.txfifo_empty_thrhd().bits(tx_threshold);
            }
            w
        });
    }

    /// Setup the driver with the given configuration
    pub(crate) fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate()?;

        self.init_slave();

        // Set slave address
        let is_10bit = config.address.is_ten_bit();
        let addr_value = config.address.as_u16();

        self.regs().slave_addr().write(|w| unsafe {
            w.slave_addr().bits(addr_value);
            w.addr_10bit_en().bit(is_10bit)
        });

        #[cfg(esp32c6)]
        {
            self.regs().ctr().modify(|_, w| {
                w.addr_10bit_rw_check_en().bit(is_10bit);
                w.slv_tx_auto_start_en().set_bit()
            });
        }

        #[cfg(not(any(esp32, esp32c6)))]
        self.regs().ctr().modify(|_, w| {
            w.addr_10bit_en().bit(is_10bit);
            w.slave_addr_en().set_bit()
        });

        // Configure clock stretching
        #[cfg(esp32c6)]
        self.regs().scl_stretch_conf().modify(|_, w| {
            w.slave_scl_stretch_en().bit(config.clock_stretch_enable);
            unsafe {
                w.stretch_protect_num().bits(1000);
            }
            w.slave_byte_ack_ctl_en().clear_bit();
            w.slave_byte_ack_lvl().clear_bit()
        });

        #[cfg(not(any(esp32, esp32c6)))]
        self.regs().scl_stretch_conf().modify(|_, w| {
            w.slave_scl_stretch_en().bit(config.clock_stretch_enable);
            unsafe { w.stretch_protect_num().bits(1000) }
        });

        // Configure timeout
        #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s2, esp32s3))]
        self.regs().to().modify(|_, w| {
            unsafe {
                w.time_out_en().set_bit();
                w.time_out_value().bits(0x10);
            }
            w
        });

        // Configure filters
        self.configure_filters(config);

        self.update_registers();

        Ok(())
    }

    /// Configure SDA/SCL filters
    fn configure_filters(&self, config: &Config) {
        let sda_threshold = if config.sda_filter_enable {
            Some(config.sda_filter_threshold)
        } else {
            None
        };

        let scl_threshold = if config.scl_filter_enable {
            Some(config.scl_filter_threshold)
        } else {
            None
        };

        cfg_if::cfg_if! {
            if #[cfg(i2c_master_separate_filter_config_registers)] {
                self.regs().sda_filter_cfg().modify(|_, w| {
                    if let Some(threshold) = sda_threshold {
                        unsafe { w.sda_filter_thres().bits(threshold) };
                    }
                    w.sda_filter_en().bit(sda_threshold.is_some())
                });
                self.regs().scl_filter_cfg().modify(|_, w| {
                    if let Some(threshold) = scl_threshold {
                        unsafe { w.scl_filter_thres().bits(threshold) };
                    }
                    w.scl_filter_en().bit(scl_threshold.is_some())
                });
            } else {
                self.regs().filter_cfg().modify(|_, w| {
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

    /// Reset TX and RX FIFOs
    pub(crate) fn reset_fifo(&self) {
        self.clear_tx_fifo();
        self.clear_rx_fifo();

        self.clear_all_interrupts();
    }

    /// Explicitly clear TX FIFO (for stale data prevention)
    pub(crate) fn clear_tx_fifo(&self) {
        #[cfg(not(esp32))]
        {
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().set_bit());
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().clear_bit());
        }
        #[cfg(esp32)]
        {
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().set_bit());
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.tx_fifo_rst().clear_bit());
        }
    }

    pub(crate) fn clear_rx_fifo(&self) {
        #[cfg(not(esp32))]
        {
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().set_bit());
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().clear_bit());
        }
        #[cfg(esp32)]
        {
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().set_bit());
            self.regs()
                .fifo_conf()
                .modify(|_, w| w.rx_fifo_rst().clear_bit());
        }
    }

    /// Clear all pending interrupts
    fn clear_all_interrupts(&self) {
        self.regs().int_clr().write(|w| unsafe { w.bits(0x1FFF) });
    }

    /// Update hardware registers
    fn update_registers(&self) {
        #[cfg(esp32)]
        self.regs().ctr().modify(|_, w| w.trans_start().set_bit());

        #[cfg(not(any(esp32)))]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Read a byte from the FIFO
    pub(crate) fn read_fifo_byte(&self) -> u8 {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                let peri_offset = self.regs() as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
                let fifo_ptr = (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
                unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
            } else {
                self.regs().data().read().fifo_rdata().bits()
            }
        }
    }

    /// Read a byte from the RX FIFO address register (ESP32-C6 specific)
    /// On ESP32-C6, the first data byte after address matching is stored in rxfifo_raddr
    #[cfg(esp32c6)]
    pub(crate) fn read_fifo_byte_from_address_register(&self) -> u8 {
        self.regs().fifo_st().read().rxfifo_raddr().bits()
    }

    /// Write a byte to the FIFO
    pub(crate) fn write_fifo_byte(&self, data: u8) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let peri_offset = self.regs() as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
                let fifo_ptr = (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
                unsafe { fifo_ptr.write_volatile(data as u32); }
            } else {
                self.regs().data().write(|w| unsafe { w.fifo_rdata().bits(data) });
            }
        }
    }

    /// Get number of bytes available in RX FIFO
    pub(crate) fn rx_fifo_count(&self) -> usize {
        self.regs().sr().read().rxfifo_cnt().bits() as usize
    }

    /// Get number of free bytes in TX FIFO
    pub(crate) fn tx_fifo_free(&self) -> usize {
        I2C_FIFO_SIZE - self.regs().sr().read().txfifo_cnt().bits() as usize
    }

    /// Check for errors in interrupt status
    pub(crate) fn check_interrupt_errors(&self) -> Option<Error> {
        let status = self.regs().int_raw().read();

        if status.arbitration_lost().bit_is_set() {
            return Some(Error::ArbitrationLost);
        }

        if status.time_out().bit_is_set() {
            return Some(Error::Timeout);
        }

        #[cfg(not(esp32))]
        {
            if status.rxfifo_ovf().bit_is_set() {
                return Some(Error::RxFifoOverflow);
            }
            if status.rxfifo_udf().bit_is_set() {
                return Some(Error::TxFifoUnderflow);
            }
        }

        None
    }

    /// ESP32-C6 specific: Manually release clock stretching after write()
    ///
    /// This function must be called after loading TX FIFO data to release clock stretching
    /// and allow the master to read the response.
    #[cfg(esp32c6)]
    pub(crate) fn release_scl_stretch(&self) {
        if !self.config.config.clock_stretch_enable {
            return; // No stretch to release if disabled
        }

        // Use the proper ESP-IDF mechanism to clear clock stretching
        // The slave_scl_stretch_clr bit tells hardware to release the SCL line
        self.regs()
            .scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());

        // Delay to let hardware process the clear
        for _ in 0..50 {
            unsafe { core::arch::asm!("nop") };
        }

        // Update configuration to ensure state machine is synchronized
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());

        // Final delay to ensure all hardware changes are applied
        for _ in 0..30 {
            unsafe { core::arch::asm!("nop") };
        }
    }

    /// ESP32-C6 specific: Force release of clock stretching (emergency release)
    #[cfg(esp32c6)]
    pub(crate) fn force_release_scl_stretch(&self) {
        if !self.config.config.clock_stretch_enable {
            return;
        }

        // Multi-step aggressive release for stuck transactions
        // Step 1: Force clear using the clear bit multiple times if needed
        for _ in 0..3 {
            self.regs()
                .scl_stretch_conf()
                .modify(|_, w| w.slave_scl_stretch_clr().set_bit());

            // Delay between attempts
            for _ in 0..10 {
                unsafe { core::arch::asm!("nop") };
            }
        }

        // Step 2: Temporarily disable and re-enable stretching to reset state
        self.regs()
            .scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_en().clear_bit());

        // Longer delay for state reset
        for _ in 0..20 {
            unsafe { core::arch::asm!("nop") };
        }

        // Re-enable if originally enabled
        self.regs()
            .scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_en().set_bit());

        // Step 3: Update configuration to ensure changes take effect
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());

        // Final delay
        for _ in 0..10 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}

/// Interrupt handler for I2C slave
///
/// This is called by the hardware interrupt and must be FAST (<1µs if possible).
/// It performs minimal work and wakes async tasks to handle processing.
#[ram]
pub(crate) fn async_handler(info: &super::instance::Info, state: &State) {
    let regs = info.regs();
    let int_status = regs.int_raw().read();

    state.increment_interrupt_counter();

    // Check for errors first
    if int_status.arbitration_lost().bit_is_set() {
        state.set_error(Error::ArbitrationLost);
        regs.int_clr()
            .write(|w| w.arbitration_lost().clear_bit_by_one());
        state.wake_rx();
        state.wake_tx();
        return;
    }

    if int_status.time_out().bit_is_set() {
        state.set_error(Error::Timeout);
        regs.int_clr().write(|w| w.time_out().clear_bit_by_one());
        state.wake_rx();
        state.wake_tx();
        return;
    }

    // Handle RX FIFO threshold (data available to read)
    #[cfg(esp32)]
    let rx_ready = int_status.rxfifo_full().bit_is_set();
    #[cfg(not(esp32))]
    let rx_ready = int_status.rxfifo_wm().bit_is_set();

    if rx_ready {
        // Wake RX task to process data
        state.wake_rx();

        #[cfg(esp32)]
        regs.int_clr().write(|w| w.rxfifo_full().clear_bit_by_one());
        #[cfg(not(esp32))]
        regs.int_clr().write(|w| w.rxfifo_wm().clear_bit_by_one());
    }

    // Handle TX FIFO threshold (space available to write)
    #[cfg(esp32)]
    let tx_ready = int_status.txfifo_empty().bit_is_set();
    #[cfg(not(esp32))]
    let tx_ready = int_status.txfifo_wm().bit_is_set();

    if tx_ready {
        // Wake TX task to send more data
        state.wake_tx();

        #[cfg(esp32)]
        regs.int_clr()
            .write(|w| w.txfifo_empty().clear_bit_by_one());
        #[cfg(not(esp32))]
        regs.int_clr().write(|w| w.txfifo_wm().clear_bit_by_one());
    }

    // Handle transaction complete
    // CRITICAL TIMING NOTE:
    // trans_complete fires AFTER address+ACK but BEFORE data bytes arrive in RX FIFO.
    // If we mark the transaction as Complete when bytes_transferred=0, the future would
    // loop indefinitely waiting for data that never arrives (the rxfifo_wm interrupt won't
    // fire again because the state is already Complete). By staying in Receiving state
    // when bytes_received=0, we allow the rxfifo_wm interrupt to properly wake us when
    // the first data byte actually arrives in the FIFO.
    if int_status.trans_complete().bit_is_set() {
        use super::state::TransactionState;

        critical_section::with(|cs| {
            match state.get_state() {
                TransactionState::Receiving { bytes_received } => {
                    // Only mark complete if we actually received data
                    // trans_complete fires after address+ACK but BEFORE data arrives in write operations
                    if bytes_received > 0 {
                        state.set_state(TransactionState::Complete {
                            bytes_transferred: bytes_received,
                        });
                    }
                    // If bytes_received == 0, stay in Receiving state to wait for actual data
                }
                TransactionState::Transmitting { bytes_sent } => {
                    state.set_state(TransactionState::Complete {
                        bytes_transferred: bytes_sent,
                    });
                }
                _ => {
                    // Transaction completed but we might not have been tracking it properly
                    // Check if there's data available and estimate bytes transferred
                    let rx_fifo_count = info.regs().sr().read().rxfifo_cnt().bits() as usize;
                    let rx_index = *state.rx_index.borrow_ref(cs);
                    let total_bytes = rx_index + rx_fifo_count;

                    // Only mark complete if there's actual data
                    if total_bytes > 0 {
                        state.set_state(TransactionState::Complete {
                            bytes_transferred: total_bytes,
                        });
                    }
                    // Otherwise stay in Idle state to wait for data
                }
            }
        });

        state.wake_rx();
        state.wake_tx();

        regs.int_clr()
            .write(|w| w.trans_complete().clear_bit_by_one());
    }

    // Handle STOP detection
    if int_status.end_detect().bit_is_set() {
        // STOP condition indicates transaction end
        regs.int_clr().write(|w| w.end_detect().clear_bit_by_one());
    }

    // Handle TX FIFO underflow
    #[cfg(esp32)]
    let tx_underflow = int_status.txfifo_empty().bit_is_set(); // Note: ESP32 doesn't have separate underflow
    #[cfg(not(esp32))]
    let tx_underflow = int_status.rxfifo_udf().bit_is_set(); // Note: confusing naming - TxFifoUnderflow maps to rxfifo_udf

    if tx_underflow {
        // TX FIFO underflow - master tried to read but we don't have data ready
        // This can happen if the master reads faster than we can supply data
        // Set error state and wake TX task to handle it
        state.set_error(Error::TxFifoUnderflow);
        state.wake_tx();

        #[cfg(not(esp32))]
        regs.int_clr().write(|w| w.rxfifo_udf().clear_bit_by_one()); // Note: confusing naming
    }

    // Handle RX FIFO overflow
    #[cfg(esp32)]
    let rx_overflow = int_status.rxfifo_full().bit_is_set(); // Note: ESP32 doesn't have separate overflow
    #[cfg(not(esp32))]
    let rx_overflow = int_status.rxfifo_ovf().bit_is_set();

    if rx_overflow {
        // RX FIFO overflow - master sent data faster than we can process
        state.set_error(Error::RxFifoOverflow);
        state.wake_rx();

        #[cfg(not(esp32))]
        regs.int_clr().write(|w| w.rxfifo_ovf().clear_bit_by_one());
    }
}

/// Future for async read operation
pub struct ReadFuture<'a> {
    driver: Driver<'a>,
    buffer: &'a mut [u8],
    bytes_read: usize,
}

impl<'a> ReadFuture<'a> {
    pub fn new(driver: Driver<'a>, buffer: &'a mut [u8]) -> Self {
        // Reset state for new transaction
        // Start in Receiving state with 0 bytes so that when trans_complete fires
        // after address+ACK (before data), we properly wait for actual data instead
        // of incorrectly transitioning to Complete state
        driver.state.set_rx_index(0);
        driver
            .state
            .set_state(super::state::TransactionState::Receiving { bytes_received: 0 });

        Self {
            driver,
            buffer,
            bytes_read: 0,
        }
    }
}

impl Future for ReadFuture<'_> {
    type Output = Result<usize, Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register waker
        self.driver.state.rx_waker.register(cx.waker());

        // Check for errors
        if let Some(error) = self.driver.state.take_error() {
            return Poll::Ready(Err(error));
        }

        // Read any available data from FIFO
        let available = self.driver.rx_fifo_count();
        if available > 0 {
            let to_read = available.min(self.buffer.len() - self.bytes_read);
            let bytes_read = self.bytes_read;
            for i in 0..to_read {
                self.buffer[bytes_read + i] = self.driver.read_fifo_byte();
            }
            self.bytes_read += to_read;
            self.driver.state.set_rx_index(self.bytes_read);
        }

        // Check if transaction is complete
        use super::state::TransactionState;
        match self.driver.state.get_state() {
            TransactionState::Complete { bytes_transferred } => {
                // CRITICAL: Only return if we actually have some data!
                // This prevents early completion when trans_complete fires after address but before data
                if self.bytes_read > 0 {
                    Poll::Ready(Ok(self.bytes_read.min(self.buffer.len())))
                } else {
                    // No data yet, keep waiting for actual data bytes
                    Poll::Pending
                }
            }
            TransactionState::Error(e) => Poll::Ready(Err(e)),
            _ => {
                // Update state to receiving if we have data
                if self.bytes_read > 0 {
                    self.driver.state.set_state(TransactionState::Receiving {
                        bytes_received: self.bytes_read,
                    });
                }
                Poll::Pending
            }
        }
    }
}

/// Future for async write operation
pub struct WriteFuture<'a> {
    driver: Driver<'a>,
    buffer: &'a [u8],
    bytes_written: usize,
}

impl<'a> WriteFuture<'a> {
    pub fn new(driver: Driver<'a>, buffer: &'a [u8]) -> Self {
        // ESP32-C6: Force release any stuck stretch from previous RX phase
        // This is critical for write_read() where repeated START doesn't reset stretch state
        #[cfg(esp32c6)]
        {
            driver.force_release_scl_stretch();
        }

        // Reset state for new transaction
        driver.state.set_tx_index(0);
        driver.state.set_state(super::state::TransactionState::Idle);

        // Preload TX FIFO with initial data
        let mut bytes_written = 0;
        let free_space = driver.tx_fifo_free();
        let to_write = buffer.len().min(free_space);

        for i in 0..to_write {
            driver.write_fifo_byte(buffer[i]);
            bytes_written += 1;
        }

        // ESP32-C6 specific: Release clock stretch after loading TX FIFO
        #[cfg(esp32c6)]
        {
            // Critical: Release clock stretch to allow master to read the data
            driver.release_scl_stretch();
        }

        driver.state.set_tx_index(bytes_written);
        driver
            .state
            .set_state(super::state::TransactionState::Transmitting {
                bytes_sent: bytes_written,
            });

        Self {
            driver,
            buffer,
            bytes_written,
        }
    }
}

impl Future for WriteFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register waker
        self.driver.state.tx_waker.register(cx.waker());

        // Check for errors
        if let Some(error) = self.driver.state.take_error() {
            return Poll::Ready(Err(error));
        }

        // ESP32-C6 specific: Continuously release clock stretch during TX
        // This is critical for write_read() protocol
        #[cfg(esp32c6)]
        {
            self.driver.release_scl_stretch();
        }

        // Write more data if FIFO has space
        let free_space = self.driver.tx_fifo_free();
        if free_space > 0 && self.bytes_written < self.buffer.len() {
            let to_write = (self.buffer.len() - self.bytes_written).min(free_space);
            for i in 0..to_write {
                self.driver
                    .write_fifo_byte(self.buffer[self.bytes_written + i]);
            }
            self.bytes_written += to_write;
            self.driver.state.set_tx_index(self.bytes_written);
            self.driver
                .state
                .set_state(super::state::TransactionState::Transmitting {
                    bytes_sent: self.bytes_written,
                });

            // ESP32-C6 specific: Release clock stretch after writing more data
            #[cfg(esp32c6)]
            {
                self.driver.release_scl_stretch();
            }
        }

        // Check if all data has been written
        if self.bytes_written >= self.buffer.len() {
            use super::state::TransactionState;
            match self.driver.state.get_state() {
                TransactionState::Complete { .. } => {
                    // Clear TX FIFO only after transmission is fully complete
                    self.driver.clear_tx_fifo();
                    Poll::Ready(Ok(()))
                }
                TransactionState::Error(e) => Poll::Ready(Err(e)),
                _ => Poll::Pending,
            }
        } else {
            Poll::Pending
        }
    }
}
