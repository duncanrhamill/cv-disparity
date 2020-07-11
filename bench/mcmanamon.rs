use criterion::{black_box, criterion_group, criterion_main, Criterion};

use cv_camstream::{GrayFloatImage, StereoFrame};
use cv_disparity::{prelude::*, mcmanamon::{McManamon, Params}};
use image;

fn mcmanamon_bench(c: &mut Criterion) {
    
    // Load images
    let left_img = image::open("res/renders/simple_rocks_01_left.png").unwrap();
    let right_img = image::open("res/renders/simple_rocks_01_right.png").unwrap();

    // Build disparity alg
    let mut disp = McManamon::new(Params {
        min_disparity: 0,
        max_disparity: 100,
        dyn_disparity_threshold: 10,
        correlation_window_size: (11, 11)
    });

    // Build frame
    let frame = StereoFrame {
        left: GrayFloatImage::from_dynamic(&left_img),
        left_timestamp: 0,
        right: GrayFloatImage::from_dynamic(&right_img),
        right_timestamp: 0
    };

    // Benchmark compute function
    c.bench_function("mcmanamon simple_rocks_01", |b| b.iter(|| disp.compute(&frame)));
}

criterion_group!(benches, mcmanamon_bench);
criterion_main!(benches);