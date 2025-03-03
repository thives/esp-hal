//! Peripheral instance definitions for I2C slave

use crate::{
    gpio::{InputSignal, OutputSignal},
    interrupt::{InterruptHandler, Priority},
    pac::i2c0::RegisterBlock,
    system::{self, Peripheral},
};

use super::{Event, driver::async_handler, state::State};

/// Peripheral instance information
#[doc(hidden)]
#[derive(Debug)]
pub struct Info {
    /// Pointer to the register block
    pub register_block: *const RegisterBlock,

    /// System peripheral marker
    pub peripheral: system::Peripheral,

    /// Async interrupt handler
    pub async_handler: InterruptHandler,

    /// SCL output signal
    pub scl_output: OutputSignal,

    /// SCL input signal
    pub scl_input: InputSignal,

    /// SDA output signal
    pub sda_output: OutputSignal,

    /// SDA input signal
    pub sda_input: InputSignal,
}

impl Info {
    /// Get register block
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Enable or disable interrupt listening for specific events
    pub fn enable_listen(&self, interrupts: enumset::EnumSet<Event>, enable: bool) {
        let regs = self.regs();

        regs.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    #[cfg(esp32)]
                    Event::RxFifoThreshold => w.rxfifo_full().bit(enable),
                    #[cfg(not(esp32))]
                    Event::RxFifoThreshold => w.rxfifo_wm().bit(enable),

                    #[cfg(esp32)]
                    Event::TxFifoThreshold => w.txfifo_empty().bit(enable),
                    #[cfg(not(esp32))]
                    Event::TxFifoThreshold => w.txfifo_wm().bit(enable),

                    #[cfg(esp32)]
                    Event::ByteReceived => w.rx_rec_full().bit(enable),
                    #[cfg(not(esp32))]
                    Event::ByteReceived => w.rxfifo_wm().bit(enable),

                    #[cfg(esp32)]
                    Event::ByteTransmitted => w.tx_send_empty().bit(enable),
                    #[cfg(not(esp32))]
                    Event::ByteTransmitted => w.txfifo_wm().bit(enable),

                    Event::TransComplete => w.trans_complete().bit(enable),
                    Event::AddressMatch | Event::StartDetected => w.trans_start().bit(enable),
                    Event::StopDetected => w.end_detect().bit(enable),
                    Event::ArbitrationLost => w.arbitration_lost().bit(enable),
                    Event::Timeout => w.time_out().bit(enable),

                    #[cfg(not(esp32))]
                    Event::RxFifoOverflow => w.rxfifo_ovf().bit(enable),
                    #[cfg(esp32)]
                    Event::RxFifoOverflow => w,

                    #[cfg(not(esp32))]
                    Event::TxFifoUnderflow => w.rxfifo_udf().bit(enable),
                    #[cfg(esp32)]
                    Event::TxFifoUnderflow => w,
                };
            }
            w
        });
    }

    /// Clear specific interrupts
    pub fn clear_interrupts(&self, interrupts: enumset::EnumSet<Event>) {
        let regs = self.regs();

        regs.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    #[cfg(esp32)]
                    Event::RxFifoThreshold => w.rxfifo_full().clear_bit_by_one(),
                    #[cfg(not(esp32))]
                    Event::RxFifoThreshold => w.rxfifo_wm().clear_bit_by_one(),

                    #[cfg(esp32)]
                    Event::TxFifoThreshold => w.txfifo_empty().clear_bit_by_one(),
                    #[cfg(not(esp32))]
                    Event::TxFifoThreshold => w.txfifo_wm().clear_bit_by_one(),

                    #[cfg(esp32)]
                    Event::ByteReceived => w.rx_rec_full().clear_bit_by_one(),
                    #[cfg(not(esp32))]
                    Event::ByteReceived => w.rxfifo_wm().clear_bit_by_one(),

                    #[cfg(esp32)]
                    Event::ByteTransmitted => w.tx_send_empty().clear_bit_by_one(),
                    #[cfg(not(esp32))]
                    Event::ByteTransmitted => w.txfifo_wm().clear_bit_by_one(),

                    Event::TransComplete => w.trans_complete().clear_bit_by_one(),
                    Event::AddressMatch | Event::StartDetected => {
                        w.trans_start().clear_bit_by_one()
                    }
                    Event::StopDetected => w.end_detect().clear_bit_by_one(),
                    Event::ArbitrationLost => w.arbitration_lost().clear_bit_by_one(),
                    Event::Timeout => w.time_out().clear_bit_by_one(),

                    #[cfg(not(esp32))]
                    Event::RxFifoOverflow => w.rxfifo_ovf().clear_bit_by_one(),
                    #[cfg(esp32)]
                    Event::RxFifoOverflow => w,

                    #[cfg(not(esp32))]
                    Event::TxFifoUnderflow => w.rxfifo_udf().clear_bit_by_one(),
                    #[cfg(esp32)]
                    Event::TxFifoUnderflow => w,
                };
            }
            w
        });
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

/// Trait implemented by I2C peripheral instances
/// A peripheral singleton compatible with the I2C slave driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    /// Get peripheral info and state
    fn parts(&self) -> (&Info, &State);

    /// Get peripheral info
    fn info(&self) -> &Info {
        self.parts().0
    }

    /// Get peripheral state
    fn state(&self) -> &State {
        self.parts().1
    }

    /// Set interrupt handler on the concrete peripheral
    fn set_interrupt_handler(&self, handler: InterruptHandler);

    /// Enable peripheral interrupts (call after handler is set and peripheral is configured)
    fn enable_peripheral_interrupts(&self, priority: Priority);
}

/// Macro to implement peripheral instances
macro_rules! impl_instance {
    ($inst:ident, $peri:ident, $irq:ident, $scl:ident, $sda:ident) => {
        impl Instance for crate::peripherals::$inst<'_> {
            fn parts(&self) -> (&Info, &State) {
                #[crate::handler]
                fn irq_handler() {
                    async_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State::new();

                static PERIPHERAL: Info = Info {
                    register_block: crate::peripherals::$inst::ptr(),
                    peripheral: crate::system::Peripheral::$peri,
                    async_handler: irq_handler,
                    scl_output: OutputSignal::$scl,
                    scl_input: InputSignal::$scl,
                    sda_output: OutputSignal::$sda,
                    sda_input: InputSignal::$sda,
                };

                (&PERIPHERAL, &STATE)
            }

            fn set_interrupt_handler(&self, handler: InterruptHandler) {
                // First, ensure peripheral interrupt is disabled
                self.disable_peri_interrupt();

                let (info, _) = self.parts();
                // Disable all interrupt listening
                info.enable_listen(enumset::EnumSet::all(), false);
                // Clear any pending interrupts
                info.clear_interrupts(enumset::EnumSet::all());

                // Bind the handler but don't enable peripheral interrupt yet
                // We'll enable it after the driver is fully configured
                self.bind_peri_interrupt(handler.handler());
                // Note: NOT calling enable_peri_interrupt here
            }

            fn enable_peripheral_interrupts(&self, priority: Priority) {
                // Enable peripheral interrupt only after everything is configured
                self.enable_peri_interrupt(priority);
            }
        }
    };
}

// Implement for I2C0
#[cfg(i2c_slave_i2c0)]
impl_instance!(I2C0, I2cExt0, I2C_EXT0, I2CEXT0_SCL, I2CEXT0_SDA);

crate::any_peripheral! {
    /// Any I2C slave peripheral.
    pub peripheral AnyI2cSlave<'d> {
        #[cfg(i2c_slave_i2c0)]
        I2c0(crate::peripherals::I2C0<'d>),
        #[cfg(i2c_slave_i2c1)]
        I2c1(crate::peripherals::I2C1<'d>),
    }
}

impl Instance for AnyI2cSlave<'_> {
    fn parts(&self) -> (&Info, &State) {
        any::delegate!(self, i2c => { i2c.parts() })
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        any::delegate!(self, i2c => { i2c.set_interrupt_handler(handler) })
    }

    fn enable_peripheral_interrupts(&self, priority: Priority) {
        any::delegate!(self, i2c => { i2c.enable_peripheral_interrupts(priority) })
    }
}

impl AnyI2cSlave<'_> {
    pub(crate) fn info(&self) -> &Info {
        self.parts().0
    }

    pub(crate) fn state(&self) -> &State {
        self.parts().1
    }

    pub(crate) fn set_interrupt_priority(&self, priority: Priority) {
        // Implementations should be added per-chip
        _ = priority;
    }
}

/// Get interrupt counter for I2C0 (if available)
/// This allows access to the interrupt counter from other tasks
#[cfg(i2c_slave_i2c0)]
pub fn get_i2c0_interrupt_count() -> u32 {
    use crate::peripherals::I2C0;
    let i2c = unsafe { I2C0::steal() };
    i2c.state().get_interrupt_count()
}

// Implement for I2C1
#[cfg(i2c_slave_i2c1)]
impl_instance!(I2C1, I2cExt1, I2C_EXT1, I2CEXT1_SCL, I2CEXT1_SDA);
