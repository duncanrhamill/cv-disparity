//! Test disparity generation with the live real-time stereo bench

// -----------------------------------------------------------------------------------------------
// IMPORTS
// -----------------------------------------------------------------------------------------------

use cv_camstream::prelude::*;
use cv_disparity::{prelude::*, mcmanamon::{McManamon, Params}};
use minifb::{Key, Window, WindowOptions};

// -----------------------------------------------------------------------------------------------
// CONSTANTS
// -----------------------------------------------------------------------------------------------

const WIDTH: usize = 640 * 2;
const HEIGHT: usize = 480;

// -----------------------------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------------------------

#[test]
fn stereo_bench() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting...");

    let mut camstream = CamStreamBuilder::new()
        .stereo()
        .left_path("/dev/video0")?
        .right_path("/dev/video2")?
        .rectif_params_from_file("tests/stereo_bench_drh_01.toml")?
        .interval((1, 30))
        .resolution((640, 480))
        .format(b"MJPG")?
        .build()?;

    println!("Cameras built");

    let mut buffer: Vec<u32> = vec![0; WIDTH * HEIGHT];

    let mut window = Window::new(
        "Stereo Camera Stream",
        WIDTH,
        HEIGHT,
        WindowOptions::default()
    ).unwrap();

    window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));

    // Disparity method
    let mut disp = McManamon::new(Params {
        min_disparity: 0,
        max_disparity: 100,
        dyn_disparity_threshold: 2,
        correlation_window_size: (11, 11)
    });

    // Flag indicating whether or not to compute disparity
    let mut comp_disp = false;

    while window.is_open() && !window.is_key_down(Key::Escape) {
        if window.is_key_down(Key::Enter) {
            comp_disp = !comp_disp;
        }

        let frame = camstream
            .capture()?;

        let left = frame.left.to_dynamic_luma8().to_luma();
        let mut right = frame.right.to_dynamic_luma8().to_luma();

        if comp_disp {
            // Compute disparity
            let disp_map = disp.compute(&frame)?;

            // Convert disparity map to greyscale image
            right = disp_map.to_luma_normalised();
        }
        else {
            // right = disp.pre_filter(&frame).left.to_dynamic_luma8().to_luma();
        }

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