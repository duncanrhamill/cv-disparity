//! # Error standards
//! 
//! This module provides a standardised error enum and result type for this crate.

// -----------------------------------------------------------------------------------------------
// TYPES
// -----------------------------------------------------------------------------------------------

/// Standard result type used in the disparity crate.
pub type Result<T> = std::result::Result<T, Error>;

// -----------------------------------------------------------------------------------------------
// ENUMERATIONS
// -----------------------------------------------------------------------------------------------

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Error was thrown during debugging operations")]
    Debug
}
