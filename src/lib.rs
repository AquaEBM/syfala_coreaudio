#[unsafe(no_mangle)]
pub extern "C" fn rust_fat_sine(x: f32) -> f32 {
    f32::copysign(1.0, x.fract() - 0.5)
}