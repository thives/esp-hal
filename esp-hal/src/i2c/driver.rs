/// I2c driver
/// This implements the lower level parts of the driver.

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    i2c::{
        Ack,
        AcknowledgeCheckFailedReason,
        BusTimeout,
        Command,
        Config,
        ConfigError,
        Error,
        Event,
        I2cAddress,
        I2cFuture,
        OperationType,
        State,
        I2C_LL_INTR_MASK,
        I2C_CHUNK_SIZE,
        MAX_ITERATIONS,
        configure_clock,
        estimate_ack_failed_reason,
        set_filter
    },
    i2c::info::Info,
    pac::i2c0::{RegisterBlock, COMD},
};

#[allow(dead_code)] // Some versions don't need `state`
pub(crate) struct Driver<'a> {
    pub(crate) info: &'a Info,
    pub(crate) state: &'a State,
}

impl Driver<'_> {
    pub(crate) fn regs(&self) -> &RegisterBlock {
        self.info.regs()
    }

    /// Configures the I2C peripheral with the specified frequency, clocks, and
    /// optional timeout.
    pub(crate) fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.regs().ctr().write(|w| {
            // Set I2C controller to master mode
            w.ms_mode().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();
            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });

        #[cfg(esp32s2)]
        self.regs().ctr().modify(|_, w| w.ref_always_on().set_bit());

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        set_filter(self.regs(), Some(7), Some(7));

        // Configure frequency
        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let clock = clocks.i2c_clock.convert();
            } else if #[cfg(esp32s2)] {
                let clock = clocks.apb_clock.convert();
            } else {
                let clock = clocks.xtal_clock.convert();
            }
        }
        self.set_frequency(clock, config.frequency, config.timeout)?;

        self.update_config();

        // Reset entire peripheral (also resets fifo)
        self.reset();

        Ok(())
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    pub(crate) fn reset(&self) {
        // Reset the FSM
        // (the option to reset the FSM is not available
        // for the ESP32)
        #[cfg(not(esp32))]
        self.regs().ctr().modify(|_, w| w.fsm_rst().set_bit());

        // Clear all I2C interrupts
        self.regs()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Resets the I2C peripheral's command registers
    pub(crate) fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.regs().comd_iter() {
            cmd.reset();
        }
    }

    #[cfg(esp32)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        let timeout = BusTimeout::BusCycles(match timeout {
            BusTimeout::Maximum => 0xF_FFFF,
            BusTimeout::BusCycles(cycles) => check_timeout(cycles * 2 * half_cycle, 0xF_FFFF)?,
        });

        // SCL period. According to the TRM, we should always subtract 1 to SCL low
        // period
        let scl_low = scl_low - 1;
        // Still according to the TRM, if filter is not enbled, we have to subtract 7,
        // if SCL filter is enabled, we have to subtract:
        //   8 if SCL filter is between 0 and 2 (included)
        //   6 + SCL threshold if SCL filter is between 3 and 7 (included)
        // to SCL high period
        let mut scl_high = scl_high;
        // In the "worst" case, we will subtract 13, make sure the result will still be
        // correct

        // FIXME since we always set the filter threshold to 7 we don't need conditional
        // code here once that changes we need the conditional code here
        scl_high -= 7 + 6;

        // if (filter_cfg_en) {
        //     if (thres <= 2) {
        //         scl_high -= 8;
        //     } else {
        //         assert(hw->scl_filter_cfg.thres <= 7);
        //         scl_high -= thres + 6;
        //     }
        // } else {
        //    scl_high -= 7;
        //}

        let scl_high_period = scl_high;
        let scl_low_period = scl_low;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold;
        let scl_stop_hold_time = hold;

        configure_clock(
            self.regs(),
            0,
            scl_low_period,
            scl_high_period,
            0,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(esp32s2)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        let scl_high = half_cycle / 2 + 2;
        let scl_wait_high = half_cycle - scl_high;
        let sda_hold = half_cycle / 2;
        // scl_wait_high < sda_sample <= scl_high
        let sda_sample = half_cycle / 2 - 1;
        let setup = half_cycle;
        let hold = half_cycle;

        // scl period
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold;

        let timeout = BusTimeout::BusCycles(match timeout {
            BusTimeout::Maximum => 0xFF_FFFF,
            BusTimeout::BusCycles(cycles) => check_timeout(cycles * 2 * half_cycle, 0xFF_FFFF)?,
        });

        configure_clock(
            self.regs(),
            0,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    pub(crate) fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
        let sclk_freq: u32 = source_clk / clkm_div;
        let half_cycle: u32 = sclk_freq / bus_freq / 2;
        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        // Make 80KHz as a boundary here, because when working at lower frequency, too
        // much scl_wait_high will faster the frequency according to some
        // hardware behaviors.
        let scl_wait_high = if bus_freq >= 80 * 1000 {
            half_cycle / 2 - 2
        } else {
            half_cycle / 4
        };
        let scl_high = half_cycle - scl_wait_high;
        let sda_hold = half_cycle / 4;
        let sda_sample = half_cycle / 2 + scl_wait_high;
        let setup = half_cycle;
        let hold = half_cycle;

        // According to the Technical Reference Manual, the following timings must be
        // subtracted by 1. However, according to the practical measurement and
        // some hardware behaviour, if wait_high_period and scl_high minus one.
        // The SCL frequency would be a little higher than expected. Therefore, the
        // solution here is not to minus scl_high as well as scl_wait high, and
        // the frequency will be absolutely accurate to all frequency
        // to some extent.
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold - 1;
        let sda_sample_time = sda_sample - 1;
        // setup
        let scl_rstart_setup_time = setup - 1;
        let scl_stop_setup_time = setup - 1;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold - 1;

        let timeout = match timeout {
            BusTimeout::Maximum => BusTimeout::BusCycles(0x1F),
            BusTimeout::Disabled => BusTimeout::Disabled,
            BusTimeout::BusCycles(cycles) => {
                let to_peri = (cycles * 2 * half_cycle).max(1);
                let log2 = to_peri.ilog2();
                // Round up so that we don't shorten timeouts.
                let raw = if to_peri != 1 << log2 { log2 + 1 } else { log2 };
                BusTimeout::BusCycles(check_timeout(raw, 0x1F)?)
            }
        };

        configure_clock(
            self.regs(),
            clkm_div,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(any(esp32, esp32s2))]
    pub(crate) async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.len() > 32 {
            return Err(Error::FifoExceeded);
        }

        self.wait_for_completion(false).await?;

        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.regs());
        }

        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    pub(crate) async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        self.read_all_from_fifo_blocking(buffer)
    }

    /// Configures the I2C peripheral for a write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) fn setup_write<'a, I>(
        &self,
        addr: I2cAddress,
        bytes: &[u8],
        start: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        // if start is true we can only send 254 additional bytes with the address as
        // the first
        let max_len = if start { 254usize } else { 255usize };
        if bytes.len() > max_len {
            // we could support more by adding multiple write operations
            return Err(Error::FifoExceeded);
        }

        let write_len = if start { bytes.len() + 1 } else { bytes.len() };
        // don't issue write if there is no data to write
        if write_len > 0 {
            // WRITE command
            add_cmd(
                cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: write_len as u8,
                },
            )?;
        }

        self.update_config();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(self.regs(), (addr << 1) | OperationType::Write as u8);
                }
            }
        }
        Ok(())
    }

    /// Configures the I2C peripheral for a read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) fn setup_read<'a, I>(
        &self,
        addr: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        will_continue: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthInvalid);
        }
        let (max_len, initial_len) = if will_continue {
            (255usize, buffer.len())
        } else {
            (254usize, buffer.len() - 1)
        };
        if buffer.len() > max_len {
            // we could support more by adding multiple read operations
            return Err(Error::FifoExceeded);
        }

        if start {
            // WRITE command
            add_cmd(
                cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: 1,
                },
            )?;
        }

        if initial_len > 0 {
            // READ command
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Ack,
                    length: initial_len as u8,
                },
            )?;
        }

        if !will_continue {
            // this is the last read so we need to nack the last byte
            // READ w/o ACK
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Nack,
                    length: 1,
                },
            )?;
        }

        self.update_config();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(self.regs(), (addr << 1) | OperationType::Read as u8);
                }
            }
        }
        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Reads all bytes from the RX FIFO.
    pub(crate) fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            loop {
                self.check_errors()?;

                let reg = self.regs().fifo_st().read();
                if reg.rxfifo_raddr().bits() != reg.rxfifo_waddr().bits() {
                    break;
                }
            }

            *byte = read_fifo(self.regs());
        }

        Ok(())
    }

    #[cfg(any(esp32, esp32s2))]
    /// Reads all bytes from the RX FIFO.
    pub(crate) fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if buffer.len() > 32 {
            return Err(Error::FifoExceeded);
        }

        // wait for completion - then we can just read the data from FIFO
        // once we change to non-fifo mode to support larger transfers that
        // won't work anymore
        self.wait_for_completion_blocking(false)?;

        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.regs());
        }

        Ok(())
    }

    /// Clears all pending interrupts for the I2C peripheral.
    pub(crate) fn clear_all_interrupts(&self) {
        self.regs()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    #[cfg(any(esp32, esp32s2))]
    pub(crate) async fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        if start_index >= bytes.len() {
            return Ok(());
        }

        for b in bytes {
            write_fifo(self.regs(), *b);
            self.check_errors()?;
        }

        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    pub(crate) async fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            self.regs()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            if index >= bytes.len() {
                break Ok(());
            }

            write_fifo(self.regs(), bytes[index]);
            index += 1;
        }
    }

    #[cfg(not(esp32))]
    pub(crate) async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        self.check_errors()?;

        if end_only {
            I2cFuture::new(Event::EndDetect, self.info, self.state).await?;
        } else {
            let res = embassy_futures::select::select(
                I2cFuture::new(Event::TxComplete, self.info, self.state),
                I2cFuture::new(Event::EndDetect, self.info, self.state),
            )
            .await;

            match res {
                embassy_futures::select::Either::First(res) => res?,
                embassy_futures::select::Either::Second(res) => res?,
            }
        }
        self.check_all_commands_done()?;

        Ok(())
    }

    #[cfg(esp32)]
    pub(crate) async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        // for ESP32 we need a timeout here but wasting a timer seems unnecessary
        // given the short time we spend here

        let mut tout = MAX_ITERATIONS / 10; // adjust the timeout because we are yielding in the loop
        loop {
            let interrupts = self.regs().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed (either a STOP condition or END was
            // processed)
            if (!end_only && interrupts.trans_complete().bit_is_set())
                || interrupts.end_detect().bit_is_set()
            {
                break;
            }

            tout -= 1;
            if tout == 0 {
                return Err(Error::Timeout);
            }

            embassy_futures::yield_now().await;
        }
        self.check_all_commands_done()?;
        Ok(())
    }

    /// Waits for the completion of an I2C transaction.
    pub(crate) fn wait_for_completion_blocking(&self, end_only: bool) -> Result<(), Error> {
        let mut tout = MAX_ITERATIONS;
        loop {
            let interrupts = self.regs().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed (either a STOP condition or END was
            // processed)
            if (!end_only && interrupts.trans_complete().bit_is_set())
                || interrupts.end_detect().bit_is_set()
            {
                break;
            }

            tout -= 1;
            if tout == 0 {
                return Err(Error::Timeout);
            }
        }
        self.check_all_commands_done()?;
        Ok(())
    }

    /// Checks whether all I2C commands have completed execution.
    pub(crate) fn check_all_commands_done(&self) -> Result<(), Error> {
        // NOTE: on esp32 executing the end command generates the end_detect interrupt
        //       but does not seem to clear the done bit! So we don't check the done
        //       status of an end command
        for cmd_reg in self.regs().comd_iter() {
            let cmd = cmd_reg.read();

            if cmd.bits() != 0x0 && !cmd.opcode().is_end() && !cmd.command_done().bit_is_set() {
                return Err(Error::ExecutionIncomplete);
            }
        }

        Ok(())
    }

    /// Checks for I2C transmission errors and handles them.
    ///
    /// This function inspects specific I2C-related interrupts to detect errors
    /// during communication, such as timeouts, failed acknowledgments, or
    /// arbitration loss. If an error is detected, the function handles it
    /// by resetting the I2C peripheral to clear the error condition and then
    /// returns an appropriate error.
    pub(crate) fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.regs().int_raw().read();

        // The ESP32 variant has a slightly different interrupt naming
        // scheme!
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // Handle error cases
                let retval = if interrupts.time_out().bit_is_set() {
                    Err(Error::Timeout)
                } else if interrupts.nack().bit_is_set() {
                    Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(self.regs())))
                } else if interrupts.arbitration_lost().bit_is_set() {
                    Err(Error::ArbitrationLost)
                } else {
                    Ok(())
                };
            } else {
                // Handle error cases
                let retval = if interrupts.time_out().bit_is_set() {
                    Err(Error::Timeout)
                } else if interrupts.nack().bit_is_set() {
                    Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(self.regs())))
                } else if interrupts.arbitration_lost().bit_is_set() {
                    Err(Error::ArbitrationLost)
                } else if interrupts.trans_complete().bit_is_set() && self.regs().sr().read().resp_rec().bit_is_clear() {
                    Err(Error::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Data))
                } else {
                    Ok(())
                };
            }
        }

        if retval.is_err() {
            self.reset();
        }

        retval
    }

    /// Updates the configuration of the I2C peripheral.
    ///
    /// This function ensures that the configuration values, such as clock
    /// settings, SDA/SCL filtering, timeouts, and other operational
    /// parameters, which are configured in other functions, are properly
    /// propagated to the I2C hardware. This step is necessary to synchronize
    /// the software-configured settings with the peripheral's internal
    /// registers, ensuring that the hardware behaves according to the
    /// current configuration.
    pub(crate) fn update_config(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C2, C3, C6, H2 and S3 variant)
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Starts an I2C transmission.
    pub(crate) fn start_transmission(&self) {
        // Start transmission
        self.regs().ctr().modify(|_, w| w.trans_start().set_bit());
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Fills the TX FIFO with data from the provided slice.
    pub(crate) fn fill_tx_fifo(&self, bytes: &[u8]) -> Result<usize, Error> {
        let mut index = 0;
        while index < bytes.len() && !self.regs().int_raw().read().txfifo_ovf().bit_is_set() {
            write_fifo(self.regs(), bytes[index]);
            index += 1;
        }
        if self.regs().int_raw().read().txfifo_ovf().bit_is_set() {
            index -= 1;
            self.regs()
                .int_clr()
                .write(|w| w.txfifo_ovf().clear_bit_by_one());
        }
        Ok(index)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    pub(crate) fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            while !self.regs().int_raw().read().txfifo_wm().bit_is_set() {
                self.check_errors()?;
            }

            self.regs()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            while !self.regs().int_raw().read().txfifo_wm().bit_is_set() {
                self.check_errors()?;
            }

            if index >= bytes.len() {
                break Ok(());
            }

            write_fifo(self.regs(), bytes[index]);
            index += 1;
        }
    }

    #[cfg(any(esp32, esp32s2))]
    /// Fills the TX FIFO with data from the provided slice.
    pub(crate) fn fill_tx_fifo(&self, bytes: &[u8]) -> Result<usize, Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if bytes.len() > 31 {
            return Err(Error::FifoExceeded);
        }

        for b in bytes {
            write_fifo(self.regs(), *b);
        }

        Ok(bytes.len())
    }

    #[cfg(any(esp32, esp32s2))]
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    pub(crate) fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if start_index >= bytes.len() {
            return Ok(());
        }

        // this is only possible when writing the I2C address in release mode
        // from [perform_write_read]
        for b in bytes {
            write_fifo(self.regs(), *b);
            self.check_errors()?;
        }

        Ok(())
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(not(esp32))]
    pub(crate) fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.regs().fifo_conf().modify(|_, w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.fifo_prt_en().set_bit();
            w.rxfifo_wm_thrhd().bits(1);
            w.txfifo_wm_thrhd().bits(8)
        });

        self.regs().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.regs().int_clr().write(|w| {
            w.rxfifo_wm().clear_bit_by_one();
            w.txfifo_wm().clear_bit_by_one()
        });

        self.update_config();
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(esp32)]
    pub(crate) fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.regs().fifo_conf().modify(|_, w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.nonfifo_rx_thres().bits(1);
            w.nonfifo_tx_thres().bits(32)
        });

        self.regs().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.regs()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    pub(crate) fn start_write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<usize, Error> {
        self.reset_fifo();
        self.reset_command_list();
        let cmd_iterator = &mut self.regs().comd_iter();

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        self.setup_write(address, bytes, start, cmd_iterator)?;

        add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        let index = self.fill_tx_fifo(bytes)?;
        self.start_transmission();

        Ok(index)
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) fn start_read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.reset_fifo();
        self.reset_command_list();

        let cmd_iterator = &mut self.regs().comd_iter();

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        self.setup_read(address, buffer, start, will_continue, cmd_iterator)?;

        add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        self.start_transmission();
        Ok(())
    }

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) fn write_operation_blocking(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let index = self.start_write_operation(address, bytes, start, stop)?;
        // Fill the FIFO with the remaining bytes:
        self.write_remaining_tx_fifo_blocking(index, bytes)?;
        self.wait_for_completion_blocking(!stop)?;
        Ok(())
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) fn read_operation_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        self.start_read_operation(address, buffer, start, stop, will_continue)?;
        self.read_all_from_fifo_blocking(buffer)?;
        self.wait_for_completion_blocking(!stop)?;
        Ok(())
    }

    /// Executes an async I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) async fn write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let index = self.start_write_operation(address, bytes, start, stop)?;
        // Fill the FIFO with the remaining bytes:
        self.write_remaining_tx_fifo(index, bytes).await?;
        self.wait_for_completion(!stop).await?;
        Ok(())
    }

    /// Executes an async I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    pub(crate) async fn read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        self.start_read_operation(address, buffer, start, stop, will_continue)?;
        self.read_all_from_fifo(buffer).await?;
        self.wait_for_completion(!stop).await?;
        Ok(())
    }

    pub(crate) fn read_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.read_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
            )?;
        }

        Ok(())
    }

    pub(crate) fn write_blocking(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation_blocking(address, &[], start, stop);
        }
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.write_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
            )?;
        }

        Ok(())
    }

    pub(crate) async fn read(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.read_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
            )
            .await?;
        }

        Ok(())
    }

    pub(crate) async fn write(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation(address, &[], start, stop).await;
        }
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.write_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
            )
            .await?;
        }

        Ok(())
    }
}

fn check_timeout(v: u32, max: u32) -> Result<u32, ConfigError> {
    if v <= max {
        Ok(v)
    } else {
        Err(ConfigError::TimeoutInvalid)
    }
}

/// Adds a command to the I2C command sequence.
fn add_cmd<'a, I>(cmd_iterator: &mut I, command: Command) -> Result<(), Error>
where
    I: Iterator<Item = &'a COMD>,
{
    let cmd = cmd_iterator.next().ok_or(Error::CommandNumberExceeded)?;

    cmd.write(|w| match command {
        Command::Start => w.opcode().rstart(),
        Command::Stop => w.opcode().stop(),
        Command::End => w.opcode().end(),
        Command::Write {
            ack_exp,
            ack_check_en,
            length,
        } => unsafe {
            w.opcode().write();
            w.ack_exp().bit(ack_exp == Ack::Nack);
            w.ack_check_en().bit(ack_check_en);
            w.byte_num().bits(length);
            w
        },
        Command::Read { ack_value, length } => unsafe {
            w.opcode().read();
            w.ack_value().bit(ack_value == Ack::Nack);
            w.byte_num().bits(length);
            w
        },
    });

    Ok(())
}

#[cfg(not(esp32s2))]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data().read().fifo_rdata().bits()
}

#[cfg(not(esp32))]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    register_block
        .data()
        .write(|w| unsafe { w.fifo_rdata().bits(data) });
}

#[cfg(esp32s2)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
}

#[cfg(esp32)]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3FF53000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe {
        fifo_ptr.write_volatile(data as u32);
    }
}
