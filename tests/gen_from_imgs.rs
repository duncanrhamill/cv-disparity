//! # Generate from images
//!
//! Loads a pair of stereo images and computes a disparity map

use cv_camstream::{GrayFloatImage, StereoFrame};
use cv_disparity::{prelude::*, mcmanamon::{McManamon, Params}};
use image;
use minifb::{Key, Window, WindowOptions};

const WIDTH: usize = 640 * 2;
const HEIGHT: usize = 480;

#[test]
fn gen_from_imgs() -> Result<(), Box<dyn std::error::Error>> {

    // Load images
    let left_img = image::open("res/renders/simple_rocks_01_left.png")?;
    let right_img = image::open("res/renders/simple_rocks_01_right.png")?;

    let mut buffer: Vec<u32> = vec![0; WIDTH * HEIGHT];

    let mut window = Window::new(
        "Stereo Camera Stream",
        WIDTH,
        HEIGHT,
        WindowOptions::default()
    ).unwrap();

    window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));
    
    let mut disp = McManamon::new(Params {
        min_disparity: 0,
        max_disparity: 100,
        dyn_disparity_threshold: 10,
        correlation_window_size: (11, 11)
    });

    let frame = StereoFrame {
        left: GrayFloatImage::from_dynamic(&left_img),
        left_timestamp: 0,
        right: GrayFloatImage::from_dynamic(&right_img),
        right_timestamp: 0
    };

    let left = left_img.to_luma();
    // let left = disp.pre_filter(&frame).left.to_dynamic_luma8().to_luma();
    let right = disp.compute(&frame)?.to_luma_normalised();

    while window.is_open() && !window.is_key_down(Key::Escape) {

        for y in 0..(HEIGHT) {
            for x in 0..(WIDTH) {
                if x > (WIDTH / 2) - 1 {
                    buffer[x + y * WIDTH] = luma_to_u32(right.get_pixel(
                        (x - (WIDTH / 2)) as u32, 
                        y as u32
                    ));
                }
                else {
                    buffer[x + y * WIDTH] = luma_to_u32(left.get_pixel(x as u32, y as u32));
                }
            }
        }

        window.update_with_buffer(&buffer, WIDTH, HEIGHT).unwrap();
    }

    Ok(())
}

fn luma_to_u32(luma: &image::Luma<u8>) -> u32 {
    (luma[0] as u32) << 24 | (luma[0] as u32) << 16 | (luma[0] as u32) << 8 | luma[0] as u32 
}