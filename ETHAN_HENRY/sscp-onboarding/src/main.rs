#![no_std]
#![no_main]

mod imu;
mod ksz8567;

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::delay::Delay;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_stm32::eth::{self, Ethernet, PacketQueue};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::ETH;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts};
use embassy_time::Timer;
use panic_halt as _;
use static_cell::StaticCell;
use sunstruck_messages::Message;

use crate::ksz8567::{Ksz8567, Ksz8567Phy};

const CORE_CLOCK_HZ: u32 = 168_000_000;

/// Vehicle computer's address on the car network (motherboard is .10, steering
/// wheel .30).
const IP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 0, 20);
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 0, 1);b
/// The motherboard broadcasts every fresh message to 255.255.255.255 on this
/// port (sunstruck `TELEM_PORT`) every 100 ms; nothing is sent to the VC directly.
const RX_PORT: u16 = 6003;

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
});

type Device = Ethernet<'static, ETH, Ksz8567Phy>;

/// 25 MHz HSE crystal -> 168 MHz sysclk. Ethernet needs HCLK >= 25 MHz.
fn clock_config() -> Config {
    use embassy_stm32::rcc::*;
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,
        mul: PllMul::MUL336,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(clock_config());
    let core = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(core.SYST, CORE_CLOCK_HZ);

    defmt::info!("Hello from the STM32!");

    // ISM330DLC on SPI2: SCK = PB10, MOSI (SDA) = PB15, MISO (SDO/SA0) = PB14, CS = PB9.
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(1_000_000);
    spi_config.gpio_speed = Speed::Medium;
    let spi = Spi::new_blocking(p.SPI2, p.PB10, p.PB15, p.PB14, spi_config);
    let imu_cs = Output::new(p.PB9, Level::High, Speed::Medium);

    // ---------------------------------------------------------------------
    // Ethernet: STM32 RMII <-> KSZ8567 switch <-> port 5 magjack.
    // ---------------------------------------------------------------------
    let mut eth_nrst = Output::new(p.PD15, Level::High, Speed::Low);
    ksz8567::reset(&mut eth_nrst, &mut delay);

    // KSZ8567 on SPI3: SCK = PB3, MOSI = PD6, MISO = PB4, CS = PA15.
    let ksz_spi = Spi::new_blocking(p.SPI3, p.PB3, p.PD6, p.PB4, spi_config);
    let mut switch = Ksz8567::new(ksz_spi, Output::new(p.PA15, Level::High, Speed::Medium));
    if switch.check_id() {
        if let Err(e) = switch.configure_mcu_port() {
            defmt::error!("KSZ8567 configuration failed: {}", e);
        }
    } else {
        defmt::warn!("KSZ8567 did not answer over SPI; continuing with strap defaults");
    }

    let uid = embassy_stm32::uid::uid();
    // Locally administered unicast MAC derived from the chip's unique ID.
    let mac_address = [0x02, uid[0], uid[2], uid[4], uid[6], uid[8]];

    // PacketQueue is ~12 KB: initialize it in place so it never sits on the stack.
    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let packets = PACKETS.uninit();
    PacketQueue::init(packets);
    // SAFETY: PacketQueue::init zero-fills it, which is a valid PacketQueue.
    let packets = unsafe { packets.assume_init_mut() };

    let device = Ethernet::new_with_phy(
        packets,
        p.ETH,
        Irqs,
        p.PA1,  // REF_CLK
        p.PA7,  // CRS_DV
        p.PC4,  // RXD0
        p.PC5,  // RXD1
        p.PB12, // TXD0
        p.PB13, // TXD1
        p.PB11, // TX_EN
        mac_address,
        Ksz8567Phy,
    );

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(IP_ADDRESS, 24),
        gateway: Some(GATEWAY),
        dns_servers: heapless::Vec::new(),
    });
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let seed = u64::from_le_bytes(uid[..8].try_into().unwrap());
    let (stack, runner) = embassy_net::new(
        device,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    defmt::info!(
        "Ethernet up: IP={} MAC={=[u8]:02x}, listening on UDP {}",
        IP_ADDRESS,
        mac_address,
        RX_PORT
    );

    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(rx_task(stack).unwrap());
    spawner.spawn(switch_status_task(switch).unwrap());

    // ---------------------------------------------------------------------
    // IMU
    // ---------------------------------------------------------------------
    let led = Output::new(p.PD8, Level::Low, Speed::Low);
    // INT1 = accel data ready, INT2 = gyro data ready (both push-pull, active high).
    let int1 = Input::new(p.PB7, Pull::None);
    let int2 = Input::new(p.PD7, Pull::None);

    // The IMU needs ~10 ms after power-up before it accepts commands.
    delay.delay_ms(20);

    let mut imu = imu::Ism330dlc::new(spi, imu_cs);
    if let Err(e) = imu.init(&mut delay) {
        defmt::error!("IMU init failed: {}", e);
        return;
    }
    defmt::info!("IMU initialized");

    imu_loop(imu, int1, int2, led).await
}

/// Drives the network stack.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Device>) -> ! {
    runner.run().await
}

/// UDP datagrams received on [`RX_PORT`], logged by [`switch_status_task`].
static RX_DATAGRAMS: AtomicU32 = AtomicU32::new(0);

/// Every 2 s, logs switch link state and frame counters, plus how many UDP
/// datagrams reached [`rx_task`], to show where packets stop.
#[embassy_executor::task]
async fn switch_status_task(mut switch: Ksz8567<'static>) {
    loop {
        Timer::after_secs(2).await;
        if let Err(e) = switch.log_status() {
            defmt::error!("KSZ8567 status read failed: {}", e);
        }
        defmt::info!(
            "UDP datagrams received on {}: {}",
            RX_PORT,
            RX_DATAGRAMS.load(Ordering::Relaxed)
        );
    }
}

/// Receives UDP datagrams and decodes the sunstruck messages inside them.
#[embassy_executor::task]
async fn rx_task(stack: Stack<'static>) {
    static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut socket = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; 8]),
        RX_BUF.init([0; 2048]),
        TX_META.init([PacketMetadata::EMPTY; 1]),
        TX_BUF.init([0; 64]),
    );
    socket.bind(RX_PORT).unwrap();

    let mut buf = [0u8; 1500];
    loop {
        match socket.recv_from(&mut buf).await {
            Ok((len, meta)) => {
                RX_DATAGRAMS.fetch_add(1, Ordering::Relaxed);
                defmt::debug!("{} bytes from {}", len, meta.endpoint);
                // One datagram can hold several concatenated frames.
                Message::decode_frames(&buf[..len], |msg| {
                    defmt::info!("rx {}", defmt::Debug2Format(&msg));
                });
            }
            Err(e) => defmt::warn!("UDP recv error: {}", e),
        }
    }
}

async fn imu_loop(
    mut imu: imu::Ism330dlc<'static>,
    int1: Input<'static>,
    int2: Input<'static>,
    mut led: Output<'static>,
) -> ! {
    let mut count: u32 = 0;
    loop {
        // Wait for both sensors to have a fresh sample, yielding to the network
        // tasks instead of busy-waiting.
        while !(int1.is_high() && int2.is_high()) {
            Timer::after_micros(500).await;
        }

        match imu.read() {
            Ok(r) => {
                count += 1;
                // Print at ~10 Hz so RTT isn't flooded.
                if count % 10 == 0 {
                    // led.toggle();
                    // defmt::info!(
                    //     "accel [g] x={} y={} z={}",
                    //     r.accel_g[0],
                    //     r.accel_g[1],
                    //     r.accel_g[2],
                    // );
                    // defmt::info!(
                    //     "gyro [dps] x={} y={} z={}",
                    //     r.gyro_dps[0],
                    //     r.gyro_dps[1],
                    //     r.gyro_dps[2],
                    // );
                }
            }
            Err(e) => defmt::error!("IMU read failed: {}", e),
        }
    }
}
