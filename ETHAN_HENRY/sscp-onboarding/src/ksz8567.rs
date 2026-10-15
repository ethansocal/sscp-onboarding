//! Microchip KSZ8567 Ethernet switch, connected to the MCU over RMII.
//!
//! The switch boots from its hardware straps and forwards frames between the
//! copper ports (e.g. port 5) and the MCU port on its own, so there is no MDIO
//! PHY for the MAC to manage: the RMII link is always "up". SPI is used to
//! check the chip, force the MCU port to RMII 100 Mb/s, and read link status
//! and frame counters for debugging.
//!
//! Register numbers come from the KSZ8567R datasheet (DS00002328F). Multi-byte
//! registers are big-endian and SPI auto-increments the address.

use core::task::Context;

use embassy_stm32::eth::Phy;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi, mode::Master};

/// Chip ID registers 0x0000..=0x0002 read back as 00 85 67.
const CHIP_ID: [u8; 3] = [0x00, 0x85, 0x67];

/// SPI opcodes (top 3 bits of the 32-bit command header).
const READ: u32 = 0b011;
const WRITE: u32 = 0b010;

/// Copper (PHY) ports.
pub const PHY_PORTS: core::ops::RangeInclusive<u8> = 1..=5;
/// MAC port wired to the STM32 over RMII (found from the MIB counters).
pub const MCU_PORT: u8 = 6;
/// MAC ports (RGMII/MII/RMII).
const MAC_PORTS: [u8; 2] = [6, 7];

mod reg {
    /// Switch Operation: bit 0 = Start Switch.
    pub const SWITCH_OP: u16 = 0x0300;
    pub const START_SWITCH: u8 = 1 << 0;

    /// Per-port registers live at `port << 12 | offset`.
    pub const fn port(port: u8, offset: u16) -> u16 {
        ((port as u16) << 12) | offset
    }

    /// PHY Basic Status (16 bit, ports 1-5): bit 2 = link up, bit 5 = AN complete.
    pub const PHY_STATUS: u16 = 0x102;
    pub const PHY_LINK_UP: u16 = 1 << 2;
    /// Port Status (8 bit): bits 4:3 speed (00 = 10, 01 = 100, 10 = 1000), bit 2 full duplex.
    pub const PORT_STATUS: u16 = 0x030;

    /// XMII Port Control 0 (ports 6-7): bit 6 = full duplex, bit 4 = 100 Mb/s.
    pub const XMII_CTRL0: u16 = 0x300;
    pub const XMII0_FULL_DUPLEX: u8 = 1 << 6;
    pub const XMII0_SPEED_100: u8 = 1 << 4;
    /// XMII Port Control 1 (ports 6-7): bit 6 = 10/100 (0 = 1000 Mb/s),
    /// bit 2 = RMII REFCLK input, bits 1:0 = interface (01 = RMII).
    pub const XMII_CTRL1: u16 = 0x301;
    pub const XMII1_NOT_GIGABIT: u8 = 1 << 6;
    pub const XMII1_REFCLK_IN: u8 = 1 << 2;
    pub const XMII1_TYPE_MASK: u8 = 0b11;
    pub const XMII1_TYPE_RMII: u8 = 0b01;

    /// Port MIB Control and Status (32 bit): bit 25 = read enable/busy, bits 23:16 = index.
    pub const MIB_CTRL: u16 = 0x500;
    pub const MIB_READ: u32 = 1 << 25;
    /// Port MIB Data (32 bit).
    pub const MIB_DATA: u16 = 0x504;
}

mod mib {
    pub const RX_CRC_ERROR: u8 = 0x06;
    pub const RX_BROADCAST: u8 = 0x0A;
    pub const RX_MULTICAST: u8 = 0x0B;
    pub const RX_UNICAST: u8 = 0x0C;
    pub const TX_BROADCAST: u8 = 0x18;
    pub const TX_MULTICAST: u8 = 0x19;
    pub const TX_UNICAST: u8 = 0x1A;
    pub const TX_DROP: u8 = 0x83;
}

/// [`Phy`] for the switch's RMII port. Link is always reported up; the switch
/// side is configured by [`Ksz8567::configure_mcu_port`].
pub struct Ksz8567Phy;

impl Phy for Ksz8567Phy {
    fn phy_reset(&mut self) {}
    fn phy_init(&mut self) {}
    fn poll_link(&mut self, _cx: &mut Context) -> bool {
        true
    }
}

/// Holds the switch in reset for ~25 ms, then releases it and waits ~100 ms
/// so its SPI interface is ready.
///
/// Call this before `Ethernet::new` so the RMII lines are still high-Z while the
/// switch latches its straps.
pub fn reset(nrst: &mut Output<'_>, delay: &mut cortex_m::delay::Delay) {
    nrst.set_low();
    delay.delay_ms(25);
    nrst.set_high();
    delay.delay_ms(100);
}

pub struct Ksz8567<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs: Output<'d>,
}

impl<'d> Ksz8567<'d> {
    /// `cs` should start high.
    pub fn new(spi: Spi<'d, Blocking, Master>, cs: Output<'d>) -> Self {
        Self { spi, cs }
    }

    fn read(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), spi::Error> {
        // [opcode:3][address:24][turnaround:5], then data.
        let header = (READ << 29) | ((addr as u32) << 5);
        self.cs.set_low();
        let res = self
            .spi
            .blocking_write(&header.to_be_bytes())
            .and_then(|()| self.spi.blocking_transfer_in_place(buf));
        self.cs.set_high();
        res
    }

    fn write(&mut self, addr: u16, data: &[u8]) -> Result<(), spi::Error> {
        let header = (WRITE << 29) | ((addr as u32) << 5);
        self.cs.set_low();
        let res = self
            .spi
            .blocking_write(&header.to_be_bytes())
            .and_then(|()| self.spi.blocking_write(data));
        self.cs.set_high();
        res
    }

    pub fn read8(&mut self, addr: u16) -> Result<u8, spi::Error> {
        let mut buf = [0u8; 1];
        self.read(addr, &mut buf)?;
        Ok(buf[0])
    }

    pub fn read16(&mut self, addr: u16) -> Result<u16, spi::Error> {
        let mut buf = [0u8; 2];
        self.read(addr, &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    pub fn read32(&mut self, addr: u16) -> Result<u32, spi::Error> {
        let mut buf = [0u8; 4];
        self.read(addr, &mut buf)?;
        Ok(u32::from_be_bytes(buf))
    }

    pub fn write8(&mut self, addr: u16, value: u8) -> Result<(), spi::Error> {
        self.write(addr, &[value])
    }

    pub fn write32(&mut self, addr: u16, value: u32) -> Result<(), spi::Error> {
        self.write(addr, &value.to_be_bytes())
    }

    /// Reads the chip ID and logs whether the switch answered. Returns `true` if
    /// the ID matched.
    pub fn check_id(&mut self) -> bool {
        let mut id = [0u8; 4];
        if let Err(e) = self.read(0x0000, &mut id) {
            defmt::error!("KSZ8567 SPI read failed: {}", e);
            return false;
        }
        if id[..3] == CHIP_ID {
            defmt::info!("KSZ8567 alive, revision {=u8:#04x}", id[3]);
            true
        } else {
            defmt::warn!("KSZ8567 unexpected chip ID: {=[u8]:02x}", id);
            false
        }
    }

    /// Makes sure the switch is forwarding and sets [`MCU_PORT`] to RMII Clock
    /// Mode (switch drives the 50 MHz REF_CLK into the STM32), 100 Mb/s full
    /// duplex.
    ///
    /// On this board port 6 is strapped as MII, so in MII mode the switch drives
    /// a 25 MHz clock and the STM32 can't decode any frames. Call this before
    /// `Ethernet::new` so the MAC starts on the right clock.
    pub fn configure_mcu_port(&mut self) -> Result<(), spi::Error> {
        let op = self.read8(reg::SWITCH_OP)?;
        if op & reg::START_SWITCH == 0 {
            defmt::warn!("KSZ8567 switch was not started (op={=u8:#04x}), starting it", op);
            self.write8(reg::SWITCH_OP, op | reg::START_SWITCH)?;
        }

        let port = MCU_PORT;
        let ctrl0 = self.read8(reg::port(port, reg::XMII_CTRL0))?;
        let ctrl1 = self.read8(reg::port(port, reg::XMII_CTRL1))?;
        let new0 = ctrl0 | reg::XMII0_SPEED_100 | reg::XMII0_FULL_DUPLEX;
        let new1 = (ctrl1 & !(reg::XMII1_TYPE_MASK | reg::XMII1_REFCLK_IN))
            | reg::XMII1_TYPE_RMII
            | reg::XMII1_NOT_GIGABIT;
        self.write8(reg::port(port, reg::XMII_CTRL0), new0)?;
        self.write8(reg::port(port, reg::XMII_CTRL1), new1)?;
        defmt::info!(
            "KSZ8567 port {} -> RMII clock mode, 100 Mb/s full (ctrl0 {=u8:#04x}->{=u8:#04x}, ctrl1 {=u8:#04x}->{=u8:#04x})",
            port,
            ctrl0,
            self.read8(reg::port(port, reg::XMII_CTRL0))?,
            ctrl1,
            self.read8(reg::port(port, reg::XMII_CTRL1))?,
        );
        Ok(())
    }

    /// Reads (and clears) one MIB counter.
    fn mib(&mut self, port: u8, index: u8) -> Result<u32, spi::Error> {
        self.write32(
            reg::port(port, reg::MIB_CTRL),
            reg::MIB_READ | ((index as u32) << 16),
        )?;
        for _ in 0..100 {
            if self.read32(reg::port(port, reg::MIB_CTRL))? & reg::MIB_READ == 0 {
                break;
            }
        }
        self.read32(reg::port(port, reg::MIB_DATA))
    }

    /// Logs link state of every copper port and the frame counters (since the
    /// last call) for the copper ports with link and the MCU port.
    pub fn log_status(&mut self) -> Result<(), spi::Error> {
        for port in PHY_PORTS {
            let status = self.read16(reg::port(port, reg::PHY_STATUS))?;
            if status & reg::PHY_LINK_UP == 0 {
                defmt::info!("KSZ8567 port {}: no link", port);
                continue;
            }
            let ps = self.read8(reg::port(port, reg::PORT_STATUS))?;
            let speed = match (ps >> 3) & 0b11 {
                0b00 => 10,
                0b01 => 100,
                _ => 1000,
            };
            let rx_b = self.mib(port, mib::RX_BROADCAST)?;
            let rx_m = self.mib(port, mib::RX_MULTICAST)?;
            let rx_u = self.mib(port, mib::RX_UNICAST)?;
            let crc = self.mib(port, mib::RX_CRC_ERROR)?;
            defmt::info!(
                "KSZ8567 port {}: link UP {} Mb/s {} | from cable: bcast={} mcast={} ucast={} crc_err={}",
                port,
                speed,
                if ps & (1 << 2) != 0 { "full" } else { "half" },
                rx_b,
                rx_m,
                rx_u,
                crc,
            );
        }

        for port in MAC_PORTS {
            let ctrl1 = self.read8(reg::port(port, reg::XMII_CTRL1))?;
            let tx_b = self.mib(port, mib::TX_BROADCAST)?;
            let tx_m = self.mib(port, mib::TX_MULTICAST)?;
            let tx_u = self.mib(port, mib::TX_UNICAST)?;
            let tx_drop = self.mib(port, mib::TX_DROP)?;
            let rx_b = self.mib(port, mib::RX_BROADCAST)?;
            let rx_u = self.mib(port, mib::RX_UNICAST)?;
            let crc = self.mib(port, mib::RX_CRC_ERROR)?;
            defmt::info!(
                "KSZ8567 port {}{} ctrl1={=u8:#04x}: out bcast={} mcast={} ucast={} drop={} | in bcast={} ucast={} crc_err={}",
                port,
                if port == MCU_PORT { " (MCU)" } else { "" },
                ctrl1,
                tx_b,
                tx_m,
                tx_u,
                tx_drop,
                rx_b,
                rx_u,
                crc,
            );
        }
        Ok(())
    }
}
