#![no_std]
#![no_main]

use arrayvec::ArrayString;
use core::fmt::Write;

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive};
use embassy_nrf::{bind_interrupts, peripherals, spim};
use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::{
    geometry::Point,
    mono_font::iso_8859_1::FONT_10X20 as Font,
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, StrokeAlignment},
    text::Text,
};

// Load the TGA image
use epd_waveshare::{epd4in2::*, prelude::*};

bind_interrupts!(struct Irqs {
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

fn draw_numbers(
    value: f32,
    unit: &str,
    position: (i32, i32),
    mut display: Display4in2,
) -> Display4in2 {
    // content
    display
}

fn draw_text(mut display: Display4in2) -> Display4in2 {
    let style = MonoTextStyleBuilder::new()
        .text_color(BinaryColor::On)
        .font(&Font)
        .build();

    Text::new("Air Quality", Point::new(20, 30), style)
        .draw(&mut display)
        .unwrap();

    Text::new("Carbon Dioxide:", Point::new(20, 90), style)
        .draw(&mut display)
        .unwrap();

    Text::new("Temperature:", Point::new(20, 130), style)
        .draw(&mut display)
        .unwrap();

    Text::new("Humidity:", Point::new(20, 170), style)
        .draw(&mut display)
        .unwrap();

    display
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("running!");

    let mut delay = embassy_time::Delay {};

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M16;

    let mut spim = spim::Spim::new_txonly(p.SPI3, Irqs, p.P1_02, p.P1_01, config);
    info!("spim initialized");

    // 1_01  din
    // 1_02  clk

    let cs_pin = Output::new(p.P1_03, Level::Low, OutputDrive::Standard);
    let dc = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);
    let rst = Output::new(p.P1_05, Level::Low, OutputDrive::Standard);
    //let busy_in = Output::new(p.P1_06, Level::Low, OutputDrive::Standard);
    let busy_in = Input::new(p.P1_06, embassy_nrf::gpio::Pull::Down);

    // Setup EPD
    info!("setting epd");
    let mut epd = Epd4in2::new(&mut spim, cs_pin, busy_in, dc, rst, &mut delay).unwrap();
    info!("epd set");

    // Use display graphics from embedded-graphics
    let mut display = Display4in2::default();
    info!("display set");
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(BinaryColor::On)
        .stroke_width(3)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();

    // Draw a 3px wide outline around the display.
    display
        .bounding_box()
        .into_styled(border_stroke)
        .draw(&mut display)
        .expect("failed bounding box");

    //let mut buf = ArrayString::<[_; 12]>::new();

    let display = draw_text(display);

    // Display updated frame
    info!("updating frame");
    epd.update_frame(&mut spim, &display.buffer(), &mut delay)
        .unwrap();
    info!("displaying frame");
    epd.display_frame(&mut spim, &mut delay).unwrap();

    // Set the EPD to sleep
    info!("sleeping");
    epd.sleep(&mut spim, &mut delay).unwrap();
}
