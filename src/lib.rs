//! # Disparity Computation
//!
//! This crate provides disparity map computation for stereo computer vision.

// -----------------------------------------------------------------------------------------------
// MODULES
// -----------------------------------------------------------------------------------------------

mod disparity;
mod error;
pub mod magdeburg;
pub mod mcmanamon;

// -----------------------------------------------------------------------------------------------
// EXPORTS
// -----------------------------------------------------------------------------------------------

pub mod prelude {
    pub use crate::disparity::{DisparityAlgorithm, DisparityMap};
}