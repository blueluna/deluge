#![no_main]
#![no_std]

use core::cmp;
use core::convert::TryFrom;

use panic_rtt_target as _;

use rtt_target::{rprintln, rtt_init_print};

use rtic::app;

use arrayvec::ArrayString;

use embedded_hal::{
    digital::v2::OutputPin,
    adc::OneShot,
};

use crate::hal::target as pac;
use nrf52840_hal as hal;

use hal::{clocks, gpio, timer::Instance};
use core::fmt::Write;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    style::*,
};
use profont;
use st7789::{Orientation, ST7789};

const TIMER_SECOND: u32 = 1_000_000u32;
const TIMER_MINUTE: u32 = TIMER_SECOND * 60;

pub struct Measurements {
    count: u32,
    pub value_min: i16,
    pub value_max: i16,
    pub values: [i16; 8],
}

impl core::default::Default for Measurements {
    fn default() -> Self {
        Self {
            count: 0,
            value_min: i16::MAX,
            value_max: i16::MIN,
            values: [0i16; 8],
        }
    }
}

impl Measurements {
    fn add_value(&mut self, value: i16)
    {
        if self.count >= 32 {
            self.count = 0;
            self.value_min = i16::MAX;
            self.value_max = i16::MIN;
        }
        else {
            self.count = self.count.wrapping_add(1);
        }

        self.value_min = cmp::min(self.value_min, value);
        self.value_max = cmp::max(self.value_max, value);

        let most = self.values.len() - 1;
        for n in (0..most).rev() {
            self.values[n + 1] = self.values[n];
        }
        self.values[0] = value;
    }

    fn percent(value: i16) -> u32
    {
        match u32::try_from(value) {
            Ok(value) => {
                cmp::min((3000u32.saturating_sub(value) * 10) / 100, 100)
            }
            Err(_) => 0,
        }
    }

    fn average_percent(&mut self) -> u32
    {
        let sum: u32 = self.values.iter().map(|v| Self::percent(*v)).sum();
        sum / self.values.len() as u32
    }
}

#[app(device = crate::hal::target, peripherals = true)]
const APP: () = {
    struct Resources {
        led_b: gpio::Pin<gpio::Output<gpio::PushPull>>,
        timer_0: pac::TIMER0,
        adc: hal::saadc::Saadc,
        analog0: gpio::p0::P0_02<gpio::Input<gpio::Floating>>,
        display: ST7789<SPIInterfaceNoCS<hal::spim::Spim<pac::SPIM0>, gpio::Pin<gpio::Output<gpio::PushPull>>>, gpio::Pin<gpio::Output<gpio::PushPull>>>,
        motor: gpio::Pin<gpio::Output<gpio::PushPull>>,
        measurements: Measurements,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Configure to use external clocks, and start them
        let _clocks = clocks::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        cx.device.TIMER0.set_periodic();
        cx.device.TIMER0.enable_interrupt();
        cx.device.TIMER0.timer_start(TIMER_MINUTE * 5);

        let mut delay = hal::Delay::new(cx.core.SYST);

        rtt_init_print!();

        rprintln!("Initialize");

        let port0 = gpio::p0::Parts::new(cx.device.P0);
        let port1 = gpio::p1::Parts::new(cx.device.P1);
        let led_b = port0
            .p0_31
            .into_push_pull_output(gpio::Level::High)
            .degrade();
        let analog0 = port0.p0_02.into_floating_input();

        let _backlight = port0.p0_20.into_push_pull_output(gpio::Level::High); // set medium backlight on
        let rst = port1.p1_09.into_push_pull_output(gpio::Level::Low).degrade(); // reset pin
        let _cs = port0.p0_06.into_push_pull_output(gpio::Level::Low); // keep low while drivign display
        let dc = port0.p0_08.into_push_pull_output(gpio::Level::Low).degrade(); // data/clock switch

        let spiclk = port0.p0_11.into_push_pull_output(gpio::Level::Low).degrade(); // SPI clock to LCD
        let spimosi = port0.p0_12.into_push_pull_output(gpio::Level::Low).degrade(); // SPI MOSI to LCD
        
        let pins = hal::spim::Pins {
            sck: spiclk,
            miso: None,
            mosi: Some(spimosi),
        };

        // create SPI interface
        let spi = hal::spim::Spim::new(cx.device.SPIM0, pins, hal::spim::Frequency::M8, hal::spim::MODE_3, 122);

        // display interface abstraction from SPI and DC
        let di = SPIInterfaceNoCS::new(spi, dc);

        // create driver
        let mut display = ST7789::new(di, rst, 240, 240);

        // initialize
        display.init(&mut delay).unwrap();
        // set default orientation
        let _ = display.set_orientation(Orientation::Portrait);

        let _ = display.clear(Rgb565::BLACK);

        let adc_config = hal::saadc::SaadcConfig{
            resolution: hal::saadc::Resolution::_12BIT,
            oversample: hal::saadc::Oversample::BYPASS,
            reference: hal::saadc::Reference::VDD1_4,
            gain: hal::saadc::Gain::GAIN1_6,
            resistor: hal::saadc::Resistor::BYPASS,
            time: hal::saadc::Time::_40US,
        };
        let adc = hal::saadc::Saadc::new(cx.device.SAADC, adc_config);

         // motor
        let motor = port1.p1_04.into_push_pull_output(gpio::Level::Low).degrade();

        init::LateResources {
            led_b,
            timer_0: cx.device.TIMER0,
            adc,
            analog0,
            display,
            motor,
            measurements: Measurements::default(),
        }
    }

    #[task(binds = TIMER0, resources = [timer_0, adc, analog0, led_b, measurements, display, motor])]
    fn timer(cx: timer::Context) {
        cx.resources.timer_0.timer_reset_event();

        if let Ok(value) = cx.resources.adc.read(cx.resources.analog0) {
            let measurements = cx.resources.measurements;
            measurements.add_value(value);
            let display = cx.resources.display;
            let mut buf = ArrayString::<[_; 128]>::new();

            let text_style = TextStyleBuilder::new(profont::ProFont18Point)
                .text_color(Rgb565::WHITE)
                .background_color(Rgb565::BLACK)
                .build();

            for percent in measurements.values.iter().map(|v| Measurements::percent(*v)) {
                let _ = writeln!(&mut buf, "{: >3} %", percent);
            }

            let last_value_percent = Measurements::percent(measurements.values[0]);
            let average = measurements.average_percent();

            let mut motor_on = false;
            if average < 10 {
                if last_value_percent < 20 {
                    motor_on = true;
                }
            }
            if motor_on {
                let _ = cx.resources.motor.set_high();
            }
            else {
                let _ = cx.resources.motor.set_low();
            }

            let _ = writeln!(&mut buf, "AVG {: >3} %", average);

            let _ = embedded_graphics::fonts::Text::new(
                &buf, Point::new(10, 10))
                .into_styled(text_style)
                .draw(display);
            buf.clear();
        }

        let _ = cx.resources.led_b.set_high();
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
};
