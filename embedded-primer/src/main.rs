#![no_std]
#![no_main]
#![allow(dead_code)]

use crc_all::Crc;
use defmt::*;
use embassy_executor::Spawner;
//use embassy_nrf::gpio::{Input, Level, Output, OutputDrive};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::twim::Frequency;
use embassy_nrf::Peripherals;
use embassy_nrf::{
    bind_interrupts, peripherals, spim,
    twim::{self, Error, Instance, Twim},
};
use embassy_time::{self, Delay, Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use embassy_sync::mutex::Mutex;

use embedded_graphics::{
    geometry::Point,
    mono_font::iso_8859_1::FONT_10X20 as Font,
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    // primitives::{PrimitiveStyleBuilder, StrokeAlignment},
    text::Text,
};
use embedded_hal_async::delay::DelayNs;

// Load the TGA image
use epd_waveshare::{
    epd4in2::*,
    //prelude::*
};

// spi interrupt

// i2c interrupt
bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// set ambient air pressure:
const PRESSURE: u16 = 1020;

pub struct SensorData {
    pub co2: f32,
    pub temperature: f32,
    pub humidity: f32,
}
pub const DEFAULT_ADDRESS: u8 = 0x61;
struct SCD30<'a, T: Instance>(Twim<'a, T>);

impl<'a, T> SCD30<'a, T>
where
    T: Instance,
{
    pub fn init(i2c2: Twim<'a, T>) -> Self {
        Self(i2c2)
    }

    async fn get_firmware_version(&mut self) -> Result<[u8; 2], Error> {
        let command: [u8; 2] = [0xd1, 0x00];
        let mut rd_buffer = [0u8; 2];

        self.0.write(DEFAULT_ADDRESS, &command).await?;
        self.0.read(DEFAULT_ADDRESS, &mut rd_buffer).await?;

        let major = u8::from_be(rd_buffer[0]);
        let minor = u8::from_be(rd_buffer[1]);

        Ok([major, minor])
    }

    pub async fn start_continuous_measurement(&mut self, pressure: u16) -> Result<(), Error> {
        // command bytes
        let mut command: [u8; 5] = [0x00, 0x10, 0x00, 0x00, 0x00];
        let argument_bytes = &pressure.to_be_bytes();

        command[2] = argument_bytes[0];
        command[3] = argument_bytes[1];

        let mut crc = Crc::<u8>::new(0x31, 8, 0xff, 0x00, false);

        defmt::info!("{:?}", command);

        crc.update(&pressure.to_be_bytes());
        command[4] = crc.finish();
        defmt::info!("{:?}", command);

        self.0.write(DEFAULT_ADDRESS, &command).await?;

        Ok(())
    }

    pub async fn data_ready(&mut self) -> Result<bool, Error> {
        let command: [u8; 2] = [0x02, 0x02];
        let mut rd_buffer = [0u8; 3];

        self.0.write(DEFAULT_ADDRESS, &command).await?;
        self.0.read(DEFAULT_ADDRESS, &mut rd_buffer).await?;

        Ok(u16::from_be_bytes([rd_buffer[0], rd_buffer[1]]) == 1)
    }

    pub async fn read_measurement(&mut self) -> Result<SensorData, Error> {
        let command: [u8; 2] = [0x03, 0x00];
        let mut rd_buffer = [0u8; 18];

        self.0.write(DEFAULT_ADDRESS, &command).await?;
        self.0.read(DEFAULT_ADDRESS, &mut rd_buffer).await?;

        let data = SensorData {
            co2: f32::from_bits(u32::from_be_bytes([
                rd_buffer[0],
                rd_buffer[1],
                rd_buffer[3],
                rd_buffer[4],
            ])),
            temperature: f32::from_bits(u32::from_be_bytes([
                rd_buffer[6],
                rd_buffer[7],
                rd_buffer[9],
                rd_buffer[10],
            ])),
            humidity: f32::from_bits(u32::from_be_bytes([
                rd_buffer[12],
                rd_buffer[13],
                rd_buffer[15],
                rd_buffer[16],
            ])),
        };
        Ok(data)
    }
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

#[embassy_executor::task]
async fn tick_periodic() -> ! {
    loop {
        info!("tick!");
        // async sleep primitive, suspends the task for 500ms.
        Timer::after(Duration::from_millis(500)).await;
    }
}

/*
#[embassy_executor::task]
async fn print_screen<DELAY>(epd: Epd4in2<SPI, CS, BUSY, DC, RST, DELAY>) -> ! {
    info!("spim initialized");

    // 1_01  din
    // 1_02  clk

    // Setup EPD
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
    //epd.update_frame(&mut spim, &display.buffer(), &mut delay)
    //    .unwrap();
    info!("displaying frame");
    //epd.display_frame(&mut spim, &mut delay).unwrap();

    // Set the EPD to sleep
    info!("sleeping");
    //epd.sleep(&mut spim, &mut delay).unwrap();
    loop {}
}
*/

#[embassy_executor::task]
async fn read_sensor(mut sensor: SCD30<'static, TWISPI0>) -> ! {
    let mut delay = Delay {};
    info!("getting firmware");
    let firmware_version = sensor.get_firmware_version().await.unwrap();
    info!(
        "Firmware Version: {=u8}.{=u8}",
        firmware_version[0], firmware_version[1]
    );

    info!("waiting for ready");
    sensor.start_continuous_measurement(PRESSURE).await.unwrap();
    while let Err(_) = sensor.data_ready().await {
        info!("not ready")
    }
    info!("Data ready.");

    loop {
        let result = sensor.read_measurement().await.unwrap();

        let co2 = result.co2;
        let temp = result.temperature;
        let humidity = result.humidity;

        if !co2.is_nan() {
            defmt::info!(
                "
            CO2 {=f32} ppm
            Temperature {=f32} °C
            Humidity {=f32} %
            ",
                co2,
                temp,
                humidity
            );
        };
        delay.delay_ms(1000).await;
    }
}

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
static P: Mutex<ThreadModeRawMutex, Option<Peripherals>> = Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    info!("running!");

    // i2c Co2 sensor setup
    info!("Initializing TWI...");
    let mut config = twim::Config::default();
    config.frequency = Frequency::K100;
    let twi = Twim::new(p.TWISPI0, Irqs, p.P0_31, p.P0_30, config);
    let sensor = SCD30::init(twi);

    info!("spawning sensor");
    unwrap!(spawner.spawn(read_sensor(sensor)));

    /*
    // SPI e-paper setup
    let mut delay = embassy_time::Delay {};
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M16;
    let mut spim;
    let cs_pin;
    let dc;
    let rst;
    let busy_in;
    spim = spim::Spim::new_txonly(p.SPI3, IrqsSpi, p.P1_02, p.P1_01, config);
    cs_pin = Output::new(p.P1_03, Level::Low, OutputDrive::Standard);
    dc = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);
    rst = Output::new(p.P1_05, Level::Low, OutputDrive::Standard);
    busy_in = Input::new(p.P1_06, embassy_nrf::gpio::Pull::Down);
    let epd = Epd4in2::new(&mut spim, cs_pin, busy_in, dc, rst, &mut delay).unwrap();
    unwrap!(spawner.spawn(print_screen(epd)));
    */

    //let mut buf = [0u8; 16];
    //unwrap!(twi.blocking_write_read(ADDRESS, &mut [0x00], &mut buf));
    //info!("Read: {=[u8]:x}", buf);
}
