extern crate embedded_hal;
extern crate num;
#[macro_use]
extern crate num_derive;

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::{InputPin,OutputPin};
use embedded_hal::blocking::delay::DelayMs;

const FX_OSC: u32 = 32_000_000; // Oscillator frequency Hz
const F_STEP: u32 = FX_OSC >> 19; 

const OPMODE_MODE_MASK: u8 = 0x07;
const OPMODE_LOW_FREQUENCY_MODE_MASK: u8 = 0x08;
const OPMODE_MODULATION_TYPE_BIT0_MASK: u8 = 0x20;
const OPMODE_MODULATION_TYPE_BIT1_MASK: u8 = 0x40;
const OPMODE_LORA_MODE_MASK: u8 = 0b10000000;
const PA_SELECT_MASK: u8 = 0b10000000;
const MAX_POWER_MASK: u8 = 0b01110000;
const OUTPUT_POWER_MASK: u8 = 0b00001111;

/// Error type combining SPI and Pin errors for utility
#[derive(Debug, Clone, PartialEq)]
pub enum Error<SpiError, PinError> {
    Spi(SpiError),
    Pin(PinError),
    InvalidVersion,
    InputOutOfRange,
    UnexpectedResult,
    Aborted,
}

#[allow(dead_code)]
pub struct SX1276<I: InputPin, O: OutputPin, S: spi::Transfer<u8>, D: DelayMs<u32>>{
    reset_line: O,
    nss_line: O,
    dio0_line: I,
    spi: S,
    delay: D,
}

pub enum Modulation {
    FSK = 0x00,
    OOK = 0x01,
}

pub enum PaSelect {
    RfoPin,
    PaBoost,
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

#[allow(dead_code)]
enum Registers {
    Fifo = 0x00,
    OpMode = 0x01,
    FrfMsb = 0x06,
    FrfMid = 0x07,
    FrfLsb = 0x08,
    PaConfig = 0x09,
    PaRamp = 0x0A,
    Lna = 0x0C,
    FifoAddrPtr = 0x0D,
    FifoTxBaseAddr = 0x0E,
    FifoRxBaseAddr = 0x0F,
    FifoRxCurrentAddr = 0x10,
    IrqFlagsMask = 0x11,
    IrqFlags = 0x12,
    RxNbBytes = 0x13,
    PktSnrValue = 0x19,
    ModemConfig1 = 0x1D,
    ModemConfig2 = 0x1E,
    SymbTimeoutLsb = 0x1F,
    PayloadLength = 0x22,
    MaxPayloadLength = 0x23,
    HopPeriod = 0x24,
    ModemConfig3 = 0x26,
    SyncWord = 0x39,
    DioMapping1 = 0x40,
    DioMapping2 = 0x41,
    Version = 0x42,
    PaDac = 0x4D,
}

impl<SpiError, PinError, I: InputPin, O: OutputPin<Error = PinError>, S: spi::Transfer<u8, Error = SpiError>, D: DelayMs<u32>> SX1276<I, O, S, D> {

    pub fn new(spi: S, reset_line: O, nss_line: O, dio0_line: I, delay: D) -> Result::<SX1276<I, O, S, D>, Error<SpiError, PinError>>{
        let mut hat = SX1276 {
            reset_line,
            nss_line,
            dio0_line,
            spi,
            delay,
        };

        hat.reset()?;

        // Support only SX1276 for now. If another board is detect return an error.
        if hat.version()? != 0x12 {
           Err(Error::InvalidVersion)
        }
        else {
            Ok(hat)
        }
    }

    pub fn reset(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.reset_line.set_high().map_err(|e| Error::Pin(e))?;

        self.delay.delay_ms(100);

        self.reset_line.set_low().map_err(|e| Error::Pin(e))?;

        self.delay.delay_ms(100);

        self.reset_line.set_high().map_err(|e| Error::Pin(e))?;

        self.delay.delay_ms(100);

        Ok(())
    }

    pub fn version(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::Version)
    }

    pub fn frequency(&mut self) -> Result<u32, Error<SpiError, PinError>> {
        let freq_msb = self.read_register(Registers::FrfMsb)? as u32;
        let freq_mid = self.read_register(Registers::FrfMid)? as u32;
        let freq_lsb = self.read_register(Registers::FrfLsb)? as u32;

        let freq = ((freq_msb << 16) + (freq_mid << 8) + (freq_lsb << 0)) * F_STEP;

        Ok(freq)
    }

    pub fn set_frequency(&mut self, frequency: &u32) -> Result<(), Error<SpiError, PinError>> {
        let freq : u32 = *frequency / F_STEP;

        println!("Freq {}",freq);

        self.write_register(Registers::FrfMsb, &(((freq >>16) & 0xFF) as u8))?;
        self.write_register(Registers::FrfMid, &(((freq >> 8) & 0xFF) as u8))?;
        self.write_register(Registers::FrfLsb, &(((freq >> 0) & 0xFF) as u8))?;

        Ok(())
    }

    pub fn set_pa_select(&mut self, pa_select: PaSelect) -> Result<(), Error<SpiError, PinError>> {
        let pa_config = self.read_register(Registers::PaConfig)?;
        match pa_select {
            PaSelect::RfoPin => self.write_register(Registers::PaConfig,&(pa_config & !PA_SELECT_MASK)),
            PaSelect::PaBoost => self.write_register(Registers::PaConfig,&(pa_config | PA_SELECT_MASK)),
        }
    }

    pub fn pa_select(&mut self) -> Result<PaSelect, Error<SpiError, PinError>> {
        let pa_select = self.read_register(Registers::PaConfig)? & PA_SELECT_MASK;
        match pa_select {
            0b00000000 => Ok(PaSelect::RfoPin),
            0b10000000 => Ok(PaSelect::PaBoost),
            _ => Err(Error::UnexpectedResult),
        } 
    }

    // pub fn set_max_power(&mut self, max_power: &u8) -> Result<(), Error<SpiError, PinError>> {
    //     if *max_power > 15 {
    //         return Err(Error::InputOutOfRange);
    //     }
    //     let pa_config = self.read_register(Registers::PaConfig)?;
    //     self.write_register(Registers::PaConfig, ) 
    // }

    pub fn transceiver_mode(&mut self) -> Result<TransceiverMode, Error<SpiError, PinError>> {
        let opmode_u8 = self.read_register(Registers::OpMode)?;

        Ok(num::FromPrimitive::from_u8(opmode_u8 & OPMODE_MODE_MASK).unwrap())
    }

    pub fn set_lora_mode_on(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mode = match on {
            true => self.read_register(Registers::OpMode)? | OPMODE_LORA_MODE_MASK,
            false => self.read_register(Registers::OpMode)? & !OPMODE_LORA_MODE_MASK,
        };

        self.write_register(Registers::OpMode, &mode)
    }

    pub fn set_low_frequency_mode_on(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mode = match on {
            true => self.read_register(Registers::OpMode)? | OPMODE_LOW_FREQUENCY_MODE_MASK,
            false => self.read_register(Registers::OpMode)? & !OPMODE_LOW_FREQUENCY_MODE_MASK,
        };

        self.write_register(Registers::OpMode, &mode)
    }

    pub fn set_modulation_mode(&mut self, modulation : &Modulation) -> Result<(), Error<SpiError, PinError>> {
        let mode = match modulation {
            Modulation::FSK => self.read_register(Registers::OpMode)? & !OPMODE_MODULATION_TYPE_BIT0_MASK & !OPMODE_MODULATION_TYPE_BIT1_MASK,
            Modulation::OOK => self.read_register(Registers::OpMode)? | OPMODE_MODULATION_TYPE_BIT0_MASK & !OPMODE_MODULATION_TYPE_BIT1_MASK,
        };

        self.write_register(Registers::OpMode, &mode)
    }

    pub fn set_transceiver_mode(&mut self, opmode: TransceiverMode) -> Result<(), Error<SpiError, PinError>> {
        let mode = (self.transceiver_mode()? as u8) | (opmode as u8);
        self.write_register(Registers::OpMode, &mode)
    }

    pub fn set_sync_word(&mut self, sync_word: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::SyncWord, sync_word)
    }

    pub fn sync_word(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::SyncWord)
    }

    fn select_receiver(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.nss_line.set_low().map_err(|e| Error::Pin(e))?;
        Ok(())
    }

    fn unselect_receiver(&mut self) -> Result<(), Error<SpiError, PinError>> {
        self.nss_line.set_high().map_err(|e| Error::Pin(e))?;
        Ok(())
    }

    fn read_register(&mut self, register: Registers) -> Result<u8, Error<SpiError, PinError>> {
        let mut spibuf_tx : [u8;2] = [register as u8 & 0x7F, 0x00];

        self.select_receiver()?;

        let spibuf_rx = self.spi.transfer(&mut spibuf_tx).map_err(|e| Error::Spi(e))?;

        self.unselect_receiver()?;

        Ok(spibuf_rx[1])
    }

    fn write_register(&mut self, register: Registers, value: &u8) -> Result<(), Error<SpiError, PinError>> {
        let mut spibuf_tx : [u8;2] = [register as u8 | 0x80, *value];

        self.select_receiver()?;

        self.spi.transfer(&mut spibuf_tx).map_err(|e| Error::Spi(e))?;

        self.unselect_receiver()?;

        Ok(())
    }
}