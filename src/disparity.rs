//! # General disparity objects
//!
//! This module provides generic disparity traits and structures for use by different algorithms.

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use cv_camstream::{GrayFloatImage, StereoFrame};
use image::GrayImage;
use crate::error::*;

// -----------------------------------------------------------------------------------------------
// DATA STRUCTURES
// -----------------------------------------------------------------------------------------------

/// A generic floating point disparity map.
pub struct DisparityMap {
    data: GrayFloatImage,
    pub max_disp: Option<f32>,
    pub min_disp: Option<f32>
}

// -----------------------------------------------------------------------------------------------
// TRAITS
// -----------------------------------------------------------------------------------------------

pub trait DisparityAlgorithm {
    /// Compute the disparity map of the given stereo frame.
    fn compute(&mut self, frame: &StereoFrame) -> Result<DisparityMap>;
}

// -----------------------------------------------------------------------------------------------
// IMPLEMENTATIONS
// -----------------------------------------------------------------------------------------------

impl DisparityMap {
    pub fn new(width: usize, height: usize) -> Self {
        DisparityMap {
            data: GrayFloatImage::new(width, height),
            min_disp: None,
            max_disp: None
        }
    }

    pub fn put(&mut self, x: usize, y: usize, val: f32) {
        self.data.put(x, y, val)
    }

    /// Converts the image into a dynamic Luma8 image.
    pub fn to_luma(&self) -> GrayImage {

        let mut new = image::GrayImage::new(
            self.data.width() as u32,
            self.data.height() as u32
        );

        for y in 0..new.height() {
            for x in 0..new.width() {
                let mut val = self.data.get(x as usize, y as usize);

                if val < 0.0 {
                    val = 0.0;
                }
                else if val > 255.0 {
                    val = 255.0;
                }

                *new.get_pixel_mut(x, y) = image::Luma([val as u8]);
            }
        }

        new
    }

    /// Converts the image to a normalised GrayImage.
    ///
    /// Normalises by the maximum observed disparity in the map. If the maximum disparity is not 
    /// set then the function is equivalent to `.to_luma()`.
    pub fn to_luma_normalised(&self) -> GrayImage {

        let mut new = image::GrayImage::new(
            self.data.width() as u32, 
            self.data.height() as u32
        );

        let mult = match self.max_disp {
            Some(d) => 255.0 / d,
            None => 1.0
        };

        for y in 0..new.height() {
            for x in 0..new.width() {
                let mut val = self.data.get(x as usize, y as usize) * mult;

                if val < 0.0 {
                    val = 0.0;
                }
                else if val > 255.0 {
                    val = 255.0;
                }

                *new.get_pixel_mut(x, y) = image::Luma([val as u8]);
            }
        }

        new
    }
}