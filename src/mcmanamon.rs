//! # McManamon disparity computation
//!
//! This module provides an implementation of McManamon's disparity algorithm from
//! ("EXOMARS ROVER VEHICLE PERCEPTION SYSTEM ARCHITECTURE AND TEST RESULTS")[http://robotics.estec.esa.int/ASTRA/Astra2013/Papers/Mcmanamon_2811324.pdf]

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use cv_camstream::StereoFrame;
use serde::Deserialize;

use crate::disparity::{DisparityAlgorithm, DisparityMap};
use crate::error::*;

#[cfg(feature = "statistics")]
use plotters::prelude::*;

// -----------------------------------------------------------------------------------------------
// DATA STRUCTURES
// -----------------------------------------------------------------------------------------------

pub struct McManamon {
    params: Params,
    corr_window_x_range: std::ops::Range<isize>,
    corr_window_y_range: std::ops::Range<isize>
}

#[derive(Deserialize, Debug)]
pub struct Params {
    pub min_disparity: usize,
    pub max_disparity: usize,
    pub dyn_disparity_threshold: usize,
    pub correlation_window_size: (usize, usize)
}

// -----------------------------------------------------------------------------------------------
// IMPLEMENTATIONS
// -----------------------------------------------------------------------------------------------

impl McManamon {
    /// Create a new instance of the algorithm with the given parameters.
    pub fn new(params: Params) -> Self {
        let semi_width: isize = (params.correlation_window_size.0 as isize - 1) / 2;
        let corr_window_x_range = -semi_width..semi_width;

        let semi_height: isize = (params.correlation_window_size.1 as isize - 1) / 2;
        let corr_window_y_range = -semi_height..semi_height;
        
        Self { 
            params,
            corr_window_x_range,
            corr_window_y_range
        }
    }

    /// Calculate the correlation criterion for the given position and disparity.
    fn get_criterion(&self, frame: &StereoFrame, x: usize, y: usize, d: usize) -> f32 {
        let mut acc = 0.0f32;
        
        for j in self.corr_window_y_range.clone() {
            for i in self.corr_window_x_range.clone() {
                let xi = (x as isize + i) as usize;
                let yj = (y as isize + j) as usize;
                acc += (
                    frame.left.get(xi, yj) - frame.right.get(xi - d, yj)
                ).abs();
            }
        }

        acc
    }
}

impl DisparityAlgorithm for McManamon {
    /// Compute the disparity map for the given frame.
    fn compute(&mut self, frame: &StereoFrame) -> Result<DisparityMap> {
        // println!("Computing disparity with following parameters: {:#?}", self.params);
        // println!("x_range: {:?}, y_range: {:?}", self.corr_window_x_range, self.corr_window_y_range);

        let mut disp_map = DisparityMap::new(
            frame.width() as usize, 
            frame.height() as usize
        );
        
        // ---- PRE FILTER ----

        // ---- STEREO CORRELATION ---- 

        // Dynamic disparity range tracking variables
        let mut min_dyn_disp = self.params.min_disparity;
        let mut max_dyn_disp = self.params.max_disparity;

        // Vetor of min/max disparity over time, for analysis
        #[cfg(feature = "statistics")]
        let mut min_dyn_disp_history: Vec<(usize, usize)> = vec![(0, 0); frame.height() as usize];
        #[cfg(feature = "statistics")]
        let mut max_dyn_disp_history: Vec<(usize, usize)> = vec![(0, 0); frame.height() as usize];

        // Variables to track maximum and minimum disparity within the map itself, not for range
        // adjustment. Initial values are swapped around so that they don't dominate the result.
        let mut min_disp = self.params.max_disparity as f32;
        let mut max_disp = self.params.min_disparity as f32;

        // Iterate through rows backwards
        for y in (
            self.params.correlation_window_size.1
            ..
            (frame.height() as usize - self.params.correlation_window_size.1)
        ).rev() 
        {
            #[cfg(feature = "statistics")]
            {
                min_dyn_disp_history[y] = (min_dyn_disp, y);
                max_dyn_disp_history[y] = (max_dyn_disp, y);
            }

            // Min and max disparity for this row. Initial value is the opposite limit on disparity
            // so that the minimum parameter does not dominate the values, for example.
            let mut min_disp_this_row = self.params.max_disparity as f32;
            let mut max_disp_this_row = self.params.min_disparity as f32;

            for x in 
                self.params.correlation_window_size.0 + max_dyn_disp
                ..
                (frame.width() as usize - self.params.correlation_window_size.0) 
            {
                
                // Vector of criterions
                let mut crits: Vec<f32> = Vec::with_capacity(
                    max_dyn_disp - min_dyn_disp
                );

                // Calculate criterion for each disparity
                for d in min_dyn_disp..max_dyn_disp {
                    crits.push(self.get_criterion(frame, x, y, d));
                }

                // Find index of minimum value
                let min_index = crits
                    .iter()
                    .enumerate()
                    .fold(0, |min_idx, (idx, &val)| {
                        if val < crits[min_idx] {
                            idx
                        }
                        else {
                            min_idx
                        }
                    });

                // Sub pixel interpolation
                let disp_val: f32;

                // If on the outer edge of the criterion
                if min_index == 0 || min_index == crits.len() - 1 || crits.len() < 3 {
                    disp_val = (min_dyn_disp + min_index) as f32;
                }
                // Otherwise
                else {

                    // Get left and right values of the criterion
                    let c_left = crits[min_index - 1];
                    let c_right = crits[min_index + 1];

                    // If left is higher than right
                    let denom = match c_left > c_right {
                        true => 2.0 * (c_left - crits[min_index]),
                        false => 2.0 * (c_right - crits[min_index])
                    };
                   
                    disp_val = (min_dyn_disp + min_index) as f32 + ((c_left - c_right) / denom);
                }

                // Set disparity value
                disp_map.put(x, y, disp_val);

                // Update dynamic disparity range tracking vars
                if disp_val > max_disp_this_row {
                    max_disp_this_row = disp_val;
                }
                else if disp_val < min_disp_this_row {
                    min_disp_this_row = disp_val;
                }

                // Update disparity tracking variables
                if disp_val > max_disp {
                    max_disp = disp_val;
                }
                else if disp_val < min_disp {
                    min_disp = disp_val;
                }
            }

            // println!("Disparity range row {}: {}..{}", y, min_disp_this_row, max_disp_this_row);

            // Set max disparity range value
            max_dyn_disp = max_disp_this_row.ceil() as usize + self.params.dyn_disparity_threshold;
            
            // Clamp max value
            if max_dyn_disp > self.params.max_disparity {
                max_dyn_disp = self.params.max_disparity;
            }

            // Set temp minimum disparity variable (because we're subtracting into a usize so
            // overflow is possible)
            let mut min = min_disp_this_row.floor() 
                - self.params.dyn_disparity_threshold as f32;
            
            // Clamp this value to zero and the minimum disparity
            if min < 0.0f32 {
                min = 0.0f32;
            }
            if min < self.params.min_disparity as f32 {
                min = self.params.min_disparity as f32;
            }
            
            // Set usize value
            min_dyn_disp = min as usize;

            // println!("Adjusted disparity range: {}..{}", min_disp, max_disp);
        }

        // Set disparity stats in the map
        disp_map.min_disp = Some(min_disp);
        disp_map.max_disp = Some(max_disp);

        // ---- POST FILTER ----

        // ---- PLOTTING ----
        #[cfg(feature = "statistics")]
        {
            let disp_range = BitMapBackend::new(
                "plots/mcmanamon/disp_range.png", 
                (800, 600)
            ).into_drawing_area();
            disp_range.fill(&WHITE).unwrap();

            let mut chart = ChartBuilder::on(&disp_range)
                .caption("Dynamic disparity range", ("sans-serif", 20).into_font())
                .margin(5)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .build_ranged(
                    self.params.min_disparity..self.params.max_disparity, 
                    0..frame.height() as usize
                ).unwrap();
            
            chart.configure_mesh().draw().unwrap();

            chart
                .draw_series(LineSeries::new(
                    min_dyn_disp_history,
                    &RED
                )).unwrap()
                .label("Min disparity")
                .legend(|(x, y)| 
                    PathElement::new(vec![(x, y), (x + 20, y)], &RED
                ));
            chart
                .draw_series(LineSeries::new(
                    max_dyn_disp_history,
                    &BLUE
                )).unwrap()
                .label("Max disparity")
                .legend(|(x, y)| 
                    PathElement::new(vec![(x, y), (x + 20, y)], &BLUE
                ));
            
            chart
                .configure_series_labels()
                .background_style(&WHITE.mix(0.8))
                .border_style(&BLACK)
                .draw().unwrap();

            println!("Stats plotting complete");
        }

        Ok(disp_map)
    }
}
