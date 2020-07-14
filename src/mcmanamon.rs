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

/// Criterion tripple with total, left column and right column values.
#[derive(Copy, Clone, Debug)]
struct CritTripple {
    total: f32,
    left_col: f32,
    right_col: f32
}

#[derive(Copy, Clone, Debug)]
struct CritTrippleInfo {
    tripple: CritTripple,
    x: usize,
    y: usize,
    d: usize,
}

// -----------------------------------------------------------------------------------------------
// IMPLEMENTATIONS
// -----------------------------------------------------------------------------------------------

impl McManamon {
    /// Create a new instance of the algorithm with the given parameters.
    pub fn new(params: Params) -> Self {
        let semi_width: isize = (params.correlation_window_size.0 as isize - 1) / 2;
        let corr_window_x_range = -semi_width..semi_width + 1;

        let semi_height: isize = (params.correlation_window_size.1 as isize - 1) / 2;
        let corr_window_y_range = -semi_height..semi_height + 1;
        
        Self { 
            params,
            corr_window_x_range,
            corr_window_y_range
        }
    }

    /// Calculate the correlation criterion for the given position and disparity.
    fn get_criterion(&self, frame: &StereoFrame, x: usize, y: usize, d: usize) -> CritTripple {
        let mut middle = 0.0f32;
        let mut left_col = 0.0f32;
        let mut right_col = 0.0f32;

        for j in self.corr_window_y_range.clone() {
            for i in self.corr_window_x_range.clone() {
                let xi = (x as isize + i) as usize;
                let yj = (y as isize + j) as usize;

                if i == self.corr_window_x_range.start {
                    left_col += (
                        frame.left.get(xi, yj) - frame.right.get(xi - d, yj)
                    ).abs();
                }
                else if i == self.corr_window_x_range.end - 1 {
                    right_col += (
                        frame.left.get(xi, yj) - frame.right.get(xi - d, yj)
                    ).abs();
                }
                else {
                    middle += (
                        frame.left.get(xi, yj) - frame.right.get(xi - d, yj)
                    ).abs();
                }
            }
        }

        CritTripple {
            total: middle + left_col + right_col,
            left_col,
            right_col
        }
    }

    /// Calculate the correlation criterion tripple for the given position and disparity using the
    /// optimised method.
    fn get_criterion_fast(
        &self, 
        frame: &StereoFrame, 
        x: usize, 
        y: usize, 
        d: usize, 
        left_crit_tripple: CritTripple, 
        below_right_col_crit: f32
    ) -> CritTripple {
        let mut new_crit = 0.0f32;
        let mut left_col = 0.0f32;
        let mut right_col = 0.0f32;
        let old_crit = (
            frame.left.get(
                x + self.corr_window_x_range.end as usize - 1,
                y + self.corr_window_y_range.end as usize
            ) 
            - frame.right.get(
                x + self.corr_window_x_range.end as usize - 1 - d,
                y + self.corr_window_y_range.end as usize
            ) 
        ).abs();

        for j in self.corr_window_y_range.clone() {
            let xi_left = (x as isize + self.corr_window_x_range.start) as usize;
            let xi_right = (x as isize + self.corr_window_x_range.end - 1) as usize;
            let yj = (y as isize + j) as usize;

            left_col += (
                frame.left.get(xi_left, yj) - frame.right.get(xi_left - d, yj)
            ).abs();

            right_col += (
                frame.left.get(xi_right, yj) - frame.right.get(xi_right - d, yj)
            ).abs();

            if j == self.corr_window_y_range.start {
                new_crit = right_col;
            }
        }

        CritTripple {
            total: left_crit_tripple.total - left_crit_tripple.left_col + below_right_col_crit 
                + new_crit - old_crit,
            left_col,
            right_col
        }

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

        // Counter for how many slow (0) and fast (1) criterion calcuations are made
        #[cfg(feature = "statistics")]
        let mut num_crit_assessments = (0, 0);

        // Variables to track maximum and minimum disparity within the map itself, not for range
        // adjustment. Initial values are swapped around so that they don't dominate the result.
        let mut min_disp = self.params.max_disparity as f32;
        let mut max_disp = self.params.min_disparity as f32;

        // Vector for holding criterion values in the row below.
        // Indexed as below_crits[x][d].unwrap()
        let mut below_right_col_crits: Vec<Vec<Option<f32>>> = 
            vec![vec![None; self.params.max_disparity]; frame.width() as usize];

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

            // Vector to hold left column values for the previous window
            let mut left_crits: Vec<Option<CritTripple>> = 
                vec![None; self.params.max_disparity]; 

            for x in 
                self.params.correlation_window_size.0 + max_dyn_disp
                ..
                (frame.width() as usize - self.params.correlation_window_size.0) 
            {

                // Make copy of the crit array below this one and clear the original
                let below_right_cols_copy = below_right_col_crits[x].clone();
                for c in &mut below_right_col_crits[x] {
                    *c = None;
                }

                // Make copy of the left crit values array and clear the original
                let left_crits_copy = left_crits.clone();
                for c in &mut left_crits {
                    *c = None;
                }
                
                // Vector of criterions
                let mut crits: Vec<f32> = Vec::with_capacity(
                    max_dyn_disp - min_dyn_disp
                );

                // Calculate criterion for each disparity
                for d in min_dyn_disp..max_dyn_disp {
                    let crit_tripple: CritTripple;

                    // If bottom row or first pixel in row use slow method
                    if left_crits_copy[d].is_none()
                        ||
                        below_right_cols_copy[d].is_none()
                    {
                        crit_tripple = self.get_criterion(frame, x, y, d);

                        #[cfg(feature = "statistics")]
                        {
                            num_crit_assessments.0 += 1;
                        }
                    }
                    // Otherwise use the fast method
                    else {
                        crit_tripple = self.get_criterion_fast(
                            frame,
                            x, y, d,
                            left_crits_copy[d].unwrap(),
                            below_right_cols_copy[d].unwrap()
                        );

                        #[cfg(feature = "statistics")]
                        {
                            num_crit_assessments.1 += 1;
                        }
                    }

                    // Set left tripple
                    left_crits[d] = Some(crit_tripple);
                    
                    // Set below value
                    below_right_col_crits[x][d] = Some(crit_tripple.right_col);

                    // Set total crit accumulator
                    crits.push(crit_tripple.total);
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

            println!(
                "{} slow calculations and {} fast calculations were made ({}% were fast)", 
                num_crit_assessments.0, 
                num_crit_assessments.1,
                num_crit_assessments.1 as f32 
                    / (num_crit_assessments.0 + num_crit_assessments.1) as f32 * 100.0
            );
        }

        Ok(disp_map)
    }
}
