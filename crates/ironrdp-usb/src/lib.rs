//! Protocol-independent USB data types and descriptor semantics.
//!
//! This crate is sans-I/O and intentionally knows nothing about usbredir,
//! RDPEUSB, async runtimes, request routing, or server state.
//!
//! The crate is `no_std` and dependency-free. Descriptor views borrow their
//! input, while transfer types are generic over caller-owned buffer and packet
//! storage. The crate itself does not allocate.
//!
//! Parsing is limited to byte layouts defined by USB itself, such as setup
//! packets and descriptors. Parsing usbredir packets, RDPEUSB PDUs, or any
//! other transport framing belongs in the corresponding protocol crate.
//!
//! The typed descriptor model currently follows the USB 2.0 Chapter 9 device
//! framework. USB 3.x values and descriptor identities remain representable and
//! losslessly traversable, but detailed SuperSpeed descriptor semantics are
//! intentionally deferred until a non-RDPEUSB consumer requires them.
#![cfg_attr(doc, doc = include_str!("../README.md"))]
#![no_std]

pub mod control;
pub mod descriptor;
pub mod endpoint;
pub mod transfer;
pub mod value;
