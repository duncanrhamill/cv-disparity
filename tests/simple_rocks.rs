//! Test the accuracy of the disparity maps using the simple_rocks image set.

// use cv_camstream::{GrayFloatImage, StereoFrame};
// use cv_disparity::{prelude::*, mcmanamon::{McManamon, Params}};
// use image;
// use exr::prelude::simple_image::*;

// #[test]
// fn simple_rocks() -> std::result::Result<(), Box<dyn std::error::Error>> {

//     // Read true depth map
//     let true_depth_map = Image::read_from_file(
//         "res/depth_maps/simple_rocks_01.exr", 
//         read_options::high()
//     )?;

//     for layer in true_depth_map.layers {
//         for channel in layer.channels {
//             // TODO
//             unimplemented!()
//         }
//     }

//     Ok(())

// }