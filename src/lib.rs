extern crate libgpiod;
extern crate libspidev;
extern crate num;
#[macro_use]
extern crate num_derive;

use std::io;
use std::{thread, time};
use libgpiod::{GpioChip, GpioLineValue, OutputMode};
use libspidev::SpiDev;

const RESET_GPIO : u32 = 17;
const NSS_GPIO : u32 = 25;
const DIO0_GPIO : u32 = 7;

const FX_OSC: u32 = 32_000_000; // Oscillator frequency
const F_STEP: u32 = FX_OSC << 19; 

const OPMODE_MODE_MASK: u8 = 0x07;
const OPMODE_LOW_FREQUENCY_MODE_MASK: u8 = 0x08;
const OPMODE_MODULATION_TYPE_BIT0_MASK: u8 = 0x20;
const OPMODE_MODULATION_TYPE_BIT1_MASK: u8 = 0x40;
const OPMODE_LORA_MODE_MASK: u8 = 0x80;

pub struct LoraHat {
    reset_line: GpioLineValue,
    nss_line: GpioLineValue,
    dio0_line: GpioLineValue,
    spi: SpiDev,
}

pub enum Modulation {
    FSK = 0x00,
    OOK = 0x01,
}

#[derive(FromPrimitive)]
pub enum TransceiverMode {
    Sleep = 0x00,
    Standby = 0x01,
    FsTx = 0x02,
    Tx = 0x03,
    FsRx = 0x04,
    Rx = 0x05,
    RxSingle = 0x06,
    CAD = 0x07,
}

pub enum Lna {
    Off = 0x00,
    LowGain = 0x20,
    MaxGain = 0x23,
}

pub enum SpreadingFactor {
    SF7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12,
}

enum Registers {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegLna = 0x0C,
    RegFifoAddrPtr = 0x0D,
    RegFifoTxBaseAddr = 0x0E,
    RegFifoRxBaseAddr = 0x0F,
    FifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegPktSnrValue = 0x19,
    RegModemConfig1 = 0x1D,
    RegModemConfig2 = 0x1E,
    RegSymbTimeoutLsb = 0x1F,
    RegPayloadLength = 0x22,
    RegMaxPayloadLength = 0x23,
    RegHopPeriod = 0x24,
    RegModemConfig3 = 0x26,
    RegSyncWord = 0x39,
    RegDioMapping1 = 0x40,
    RegDioMapping2 = 0x41,
    RegVersion = 0x42,
    RegPaDac = 0x4D,
}

impl LoraHat {
    pub fn new() -> io::Result<LoraHat>{
        let chip = GpioChip::new(&"/dev/gpiochip0")?;
        let spi = SpiDev::new(&"/dev/spidev0.0")?;

        let reset_line = chip.request_line_values_output(&vec!(RESET_GPIO), OutputMode::None, false, "LoraHat reset")?;
        let nss_line = chip.request_line_values_output(&vec!(NSS_GPIO), OutputMode::None, false, "LoraHat SPI CS")?;
        let dio0_line = chip.request_line_values_input(&vec!(DIO0_GPIO), false, "LoraHat DIO0")?;

        let hat = LoraHat {
            reset_line,
            nss_line,
            dio0_line,
            spi,
        };

        hat.reset()?;

        Ok(hat)
    }

    pub fn reset(&self) -> io::Result<()> {
        self.reset_line.set_line_value(1)?;

        thread::sleep(time::Duration::from_millis(100));

        self.reset_line.set_line_value(0)?;

        thread::sleep(time::Duration::from_millis(100));

        self.reset_line.set_line_value(1)?;

        thread::sleep(time::Duration::from_millis(100));

        Ok(())
    }

    pub fn version(&self) -> io::Result<u8> {
        self.read_register(Registers::RegVersion)
    }

    pub fn transceiver_mode(&self) -> io::Result<TransceiverMode> {
        let opmode_u8 = self.read_register(Registers::RegOpMode)?;

        Ok(num::FromPrimitive::from_u8(opmode_u8 & OPMODE_MODE_MASK).unwrap())
    }

    pub fn set_lora_mode_on(&self, on: &bool) -> io::Result<()> {
        let mode = match on {
            true => self.read_register(Registers::RegOpMode)? | OPMODE_LORA_MODE_MASK,
            false => self.read_register(Registers::RegOpMode)? & !OPMODE_LORA_MODE_MASK,
        };

        self.write_register(Registers::RegOpMode, &mode)
    }

    pub fn set_low_frequency_mode_on(&self, on: &bool) -> io::Result<()> {
        let mode = match on {
            true => self.read_register(Registers::RegOpMode)? | OPMODE_LOW_FREQUENCY_MODE_MASK,
            false => self.read_register(Registers::RegOpMode)? & !OPMODE_LOW_FREQUENCY_MODE_MASK,
        };

        self.write_register(Registers::RegOpMode, &mode)
    }

    pub fn set_modulation_mode(&self, modulation : &Modulation) -> io::Result<()> {
        let mode = match modulation {
            Modulation::FSK => self.read_register(Registers::RegOpMode)? & !0x20 & !0x40,
            Modulation::OOK => self.read_register(Registers::RegOpMode)? | 0x20 & !0x40,
        };

        self.write_register(Registers::RegOpMode, &mode)
    }

    pub fn set_transceiver_mode(&self, opmode: TransceiverMode) -> io::Result<()> {
        let mode = (self.transceiver_mode()? as u8) | (opmode as u8);
        self.write_register(Registers::RegOpMode, &mode)
    }

    pub fn frequency(&self) -> io::Result<u32> {
        let freq_msb = self.read_register(Registers::RegFrfMsb)? as u32;
        let freq_mid = self.read_register(Registers::RegFrfMid)? as u32;
        let freq_lsb = self.read_register(Registers::RegFrfLsb)? as u32;

        let freq = ((freq_msb << 16) + (freq_mid << 8) + (freq_lsb << 0)) * F_STEP;

        Ok(freq)
    }

    pub fn set_frequency(&self, frequency: &u32) -> io::Result<()> {
        let freq_u64 : u32 = *frequency / F_STEP;

        self.write_register(Registers::RegFrfMsb, &((freq_u64 >>16) as u8))?;
        self.write_register(Registers::RegFrfMid, &((freq_u64 >> 8) as u8))?;
        self.write_register(Registers::RegFrfLsb, &((freq_u64 >> 0) as u8))?;

        Ok(())
    }

    fn select_receiver(&self) -> io::Result<()> {
        self.nss_line.set_line_value(0)
    }

    fn unselect_receiver(&self) -> io::Result<()> {
        self.nss_line.set_line_value(1)
    }

    fn read_register(&self, register: Registers) -> io::Result<u8> {
        let spibuf_tx : [u8;2] = [register as u8 & 0x7F, 0x00];
        let mut spibuf_rx : [u8;2] = [0x00, 0x00];

        self.select_receiver()?;

        self.spi.transfer(&spibuf_tx, &mut spibuf_rx)?;

        self.unselect_receiver()?;

        Ok(spibuf_rx[1])
    }

    fn write_register(&self, register: Registers, value: &u8) -> io::Result<()> {
        let spibuf_tx : [u8;2] = [register as u8 | 0x80, *value];
        let mut spibuf_rx : [u8;2] = [0x00, 0x00];

        self.select_receiver()?;

        self.spi.transfer(&spibuf_tx, &mut spibuf_rx)?;

        self.unselect_receiver()?;

        Ok(())
    }
}