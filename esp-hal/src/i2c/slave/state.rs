//! State management for interrupt-driven I2C slave
//!
//! This module manages the shared state between interrupt handlers and async tasks.
//! Uses critical sections and atomic operations for thread-safe access.

use core::cell::RefCell;

use critical_section::Mutex;

use crate::asynch::AtomicWaker;

use super::{DEFAULT_BUFFER_SIZE, Error};

/// Transaction state for I2C slave
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransactionState {
    /// Idle, waiting for master communication
    Idle,

    /// Address matched, transaction starting
    AddressMatched {
        /// True if master is reading (slave transmitting)
        is_read: bool,
    },

    /// Receiving data from master
    Receiving {
        /// Number of bytes received so far
        bytes_received: usize,
    },

    /// Transmitting data to master
    Transmitting {
        /// Number of bytes sent so far
        bytes_sent: usize,
    },

    /// Clock stretching active (waiting for software action)
    ClockStretching,

    /// Transaction completed successfully
    Complete {
        /// Total bytes transferred
        bytes_transferred: usize,
    },

    /// Error occurred
    Error(Error),
}

/// Shared state between interrupt handler and async tasks
pub struct State {
    pub(crate) interrupt_counter: Mutex<RefCell<u32>>,

    /// Current transaction state
    pub(crate) transaction_state: Mutex<RefCell<TransactionState>>,

    /// Previous transaction state (before last interrupt)
    pub(crate) previous_state: Mutex<RefCell<TransactionState>>,

    /// RX buffer for receiving data from master
    pub(crate) rx_buffer: Mutex<RefCell<Option<&'static mut [u8]>>>,

    /// TX buffer for sending data to master
    pub(crate) tx_buffer: Mutex<RefCell<Option<&'static [u8]>>>,

    /// RX buffer index (how many bytes written)
    pub(crate) rx_index: Mutex<RefCell<usize>>,

    /// TX buffer index (how many bytes read)
    pub(crate) tx_index: Mutex<RefCell<usize>>,

    /// Waker for RX operations
    pub(crate) rx_waker: AtomicWaker,

    /// Waker for TX operations
    pub(crate) tx_waker: AtomicWaker,

    /// Last error that occurred
    pub(crate) last_error: Mutex<RefCell<Option<Error>>>,

    /// Internal buffer for RX (used when no user buffer provided)
    pub(crate) internal_rx_buffer: Mutex<RefCell<[u8; DEFAULT_BUFFER_SIZE]>>,

    /// Internal buffer for TX (used when no user buffer provided)
    pub(crate) internal_tx_buffer: Mutex<RefCell<[u8; DEFAULT_BUFFER_SIZE]>>,
}

impl State {
    /// Create a new state instance
    pub const fn new() -> Self {
        Self {
            interrupt_counter: Mutex::new(RefCell::new(0)),
            transaction_state: Mutex::new(RefCell::new(TransactionState::Idle)),
            previous_state: Mutex::new(RefCell::new(TransactionState::Idle)),
            rx_buffer: Mutex::new(RefCell::new(None)),
            tx_buffer: Mutex::new(RefCell::new(None)),
            rx_index: Mutex::new(RefCell::new(0)),
            tx_index: Mutex::new(RefCell::new(0)),
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            last_error: Mutex::new(RefCell::new(None)),
            internal_rx_buffer: Mutex::new(RefCell::new([0u8; DEFAULT_BUFFER_SIZE])),
            internal_tx_buffer: Mutex::new(RefCell::new([0u8; DEFAULT_BUFFER_SIZE])),
        }
    }

    /// Reset the state to idle
    pub fn reset(&self) {
        critical_section::with(|cs| {
            *self.transaction_state.borrow_ref_mut(cs) = TransactionState::Idle;
            *self.previous_state.borrow_ref_mut(cs) = TransactionState::Idle;
            *self.rx_buffer.borrow_ref_mut(cs) = None;
            *self.tx_buffer.borrow_ref_mut(cs) = None;
            *self.rx_index.borrow_ref_mut(cs) = 0;
            *self.tx_index.borrow_ref_mut(cs) = 0;
            *self.last_error.borrow_ref_mut(cs) = None;
        });
    }

    pub fn get_interrupt_count(&self) -> u32 {
        critical_section::with(|cs| *self.interrupt_counter.borrow_ref(cs))
    }

    pub fn set_interrupt_counter(&self, value: u32) {
        critical_section::with(|cs| {
            *self.interrupt_counter.borrow_ref_mut(cs) = value;
        });
    }

    /// Increment interrupt counter
    pub fn increment_interrupt_counter(&self) {
        critical_section::with(|cs| {
            let mut counter = self.interrupt_counter.borrow_ref_mut(cs);
            *counter += 1;
        });
    }

    /// Get the current transaction state
    pub fn get_state(&self) -> TransactionState {
        critical_section::with(|cs| *self.transaction_state.borrow_ref(cs))
    }

    /// Get the previous transaction state
    pub fn get_previous_state(&self) -> TransactionState {
        critical_section::with(|cs| *self.previous_state.borrow_ref(cs))
    }

    /// Set the transaction state
    pub fn set_state(&self, state: TransactionState) {
        critical_section::with(|cs| {
            *self.previous_state.borrow_ref_mut(cs) = *self.transaction_state.borrow_ref(cs);
            *self.transaction_state.borrow_ref_mut(cs) = state;
        });
    }

    /// Get the RX index
    pub fn get_rx_index(&self) -> usize {
        critical_section::with(|cs| *self.rx_index.borrow_ref(cs))
    }

    /// Set the RX index
    pub fn set_rx_index(&self, index: usize) {
        critical_section::with(|cs| {
            *self.rx_index.borrow_ref_mut(cs) = index;
        });
    }

    /// Increment the RX index and return the new value
    pub fn increment_rx_index(&self) -> usize {
        critical_section::with(|cs| {
            let mut idx = self.rx_index.borrow_ref_mut(cs);
            *idx += 1;
            *idx
        })
    }

    /// Get the TX index
    pub fn get_tx_index(&self) -> usize {
        critical_section::with(|cs| *self.tx_index.borrow_ref(cs))
    }

    /// Set the TX index
    pub fn set_tx_index(&self, index: usize) {
        critical_section::with(|cs| {
            *self.tx_index.borrow_ref_mut(cs) = index;
        });
    }

    /// Increment the TX index and return the new value
    pub fn increment_tx_index(&self) -> usize {
        critical_section::with(|cs| {
            let mut idx = self.tx_index.borrow_ref_mut(cs);
            *idx += 1;
            *idx
        })
    }

    /// Set the last error
    pub fn set_error(&self, error: Error) {
        critical_section::with(|cs| {
            *self.last_error.borrow_ref_mut(cs) = Some(error);
            *self.transaction_state.borrow_ref_mut(cs) = TransactionState::Error(error);
        });
    }

    /// Get and clear the last error
    pub fn take_error(&self) -> Option<Error> {
        critical_section::with(|cs| self.last_error.borrow_ref_mut(cs).take())
    }

    /// Check if a transaction is complete
    pub fn is_complete(&self) -> bool {
        matches!(
            self.get_state(),
            TransactionState::Complete { .. } | TransactionState::Error(_)
        )
    }

    /// Wake the RX task if one is waiting
    pub fn wake_rx(&self) {
        self.rx_waker.wake();
    }

    /// Wake the TX task if one is waiting
    pub fn wake_tx(&self) {
        self.tx_waker.wake();
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("State")
            .field("transaction_state", &"<mutex>")
            .field("rx_buffer", &"<mutex>")
            .field("tx_buffer", &"<mutex>")
            .field("rx_index", &"<mutex>")
            .field("tx_index", &"<mutex>")
            .field("last_error", &"<mutex>")
            .finish()
    }
}
