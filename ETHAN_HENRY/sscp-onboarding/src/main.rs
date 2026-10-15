#![no_std]
#![no_main]

use cortex_m::{Peripherals, delay::Delay};
use cortex_m_rt::entry;
use embassy_stm32::gpio::{Level, Output, Speed};
use defmt_rtt as _;
use panic_halt as _;

const BLINK_INTERVAL_MS: u32 = 100;

const CORE_CLOCK_HZ: u32 = 16_000_000;

#[entry]
fn main() -> ! {
    let p: embassy_stm32::Peripherals = embassy_stm32::init(Default::default());
    let core = Peripherals::take().unwrap();

    let mut led = Output::new(p.PD8, Level::Low, Speed::Low);

    let mut delay = Delay::new(core.SYST, CORE_CLOCK_HZ);

    defmt::info!("Hello from the STM32!");
    let mut count: u32 = 0;

    loop {
        // delay.delay_ms(BLINK_INTERVAL_MS);
        // led.toggle();
        led.set_high();
        delay.delay_ms(100);
        led.set_low();
        delay.delay_ms(900);
        count += 1;
        defmt::info!("blink #{}", count);
    }
}
