//! # I2C driver info struct
#![doc(hidden)]

use crate::{
    gpio::{InputSignal, OutputSignal},
    i2c::Event,
    interrupt::InterruptHandler,
    pac::i2c0::RegisterBlock,
    peripherals::Interrupt,
};
use enumset::EnumSet;

/// Peripheral data describing a particular I2C instance.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this I2C instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// System peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt handler for the asynchronous operations of this I2C instance.
    pub async_handler: InterruptHandler,

    /// Interrupt for this I2C instance.
    pub interrupt: Interrupt,

    /// SCL output signal.
    pub scl_output: OutputSignal,

    /// SCL input signal.
    pub scl_input: InputSignal,

    /// SDA output signal.
    pub sda_output: OutputSignal,

    /// SDA input signal.
    pub sda_input: InputSignal,
}

impl Info {
    /// Returns the register block for this I2C instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    pub(crate) fn enable_listen(&self, interrupts: EnumSet<Event>, enable: bool) {
        let reg_block = self.regs();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().bit(enable),
                    Event::TxComplete => w.trans_complete().bit(enable),
                    #[cfg(not(any(esp32, esp32s2)))]
                    Event::TxFifoWatermark => w.txfifo_wm().bit(enable),
                };
            }
            w
        });
    }

    pub(crate) fn interrupts(&self) -> EnumSet<Event> {
        let mut res = EnumSet::new();
        let reg_block = self.regs();

        let ints = reg_block.int_raw().read();

        if ints.end_detect().bit_is_set() {
            res.insert(Event::EndDetect);
        }
        if ints.trans_complete().bit_is_set() {
            res.insert(Event::TxComplete);
        }
        #[cfg(not(any(esp32, esp32s2)))]
        if ints.txfifo_wm().bit_is_set() {
            res.insert(Event::TxFifoWatermark);
        }

        res
    }

    pub(crate) fn clear_interrupts(&self, interrupts: EnumSet<Event>) {
        let reg_block = self.regs();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().clear_bit_by_one(),
                    Event::TxComplete => w.trans_complete().clear_bit_by_one(),
                    #[cfg(not(any(esp32, esp32s2)))]
                    Event::TxFifoWatermark => w.txfifo_wm().clear_bit_by_one(),
                };
            }
            w
        });
    }

    pub(crate) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, self.interrupt);
        }
        self.enable_listen(EnumSet::all(), false);
        self.clear_interrupts(EnumSet::all());
        unsafe { crate::interrupt::bind_interrupt(self.interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(self.interrupt, handler.priority()));
    }

    pub(crate) fn disable_interrupts(&self) {
        crate::interrupt::disable(crate::Cpu::current(), self.interrupt);
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}
