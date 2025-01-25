//! # Inter-Integrated Circuit (I2C) - Slave mode
//!
//! ## Overview
//!
//! In this mode, the I2C acts as slave; listens for START condition and
//! responds to I2C commands from one or more masters.
//!
//! ## Configuration
//!
//! Each I2C Slave is individually configurable, and the usual
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
