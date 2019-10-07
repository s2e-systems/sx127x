extern crate embedded_hal;
extern crate num;
#[macro_use]
extern crate bitfield;
#[macro_use]
extern crate num_derive;

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::{InputPin,OutputPin};
use embedded_hal::blocking::delay::DelayMs;

const FX_OSC: u32 = 32_000_000; // Oscillator frequency Hz
const F_STEP: u32 = FX_OSC >> 19; 
const MAX_FREQUENCY_DEVIATION: u32 = F_STEP * (1 << 14 - 1);

bitfield!{
    struct PaConfig(u8);
    pa_select, set_pa_select: 7;
    max_power, set_max_power: 6,4;
    output_power, set_output_power: 3,0;
}

bitfield!{
    struct OpMode(u8);
    long_range_mode, set_long_range_mode: 7;
    modulation_type, set_modulation_type: 6,5;
    low_frequency_mode_on, set_low_frequency_mode_on: 3;
    mode, set_mode: 2,0; 
}

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

#[derive(FromPrimitive)]
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
    FdevMsb = 0x04,
    FdevLsb = 0x05,
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

        self.write_register(Registers::FrfMsb, &(((freq >>16) & 0xFF) as u8))?;
        self.write_register(Registers::FrfMid, &(((freq >> 8) & 0xFF) as u8))?;
        self.write_register(Registers::FrfLsb, &(((freq >> 0) & 0xFF) as u8))?;

        Ok(())
    }

    pub fn set_pa_select(&mut self, pa_select: PaSelect) -> Result<(), Error<SpiError, PinError>> {
        let mut pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        
        match pa_select {
            PaSelect::RfoPin => pa_config.set_pa_select(false),
            PaSelect::PaBoost => pa_config.set_pa_select(true),
        };

        self.write_register(Registers::PaConfig, &pa_config.0)
    }

    pub fn pa_select(&mut self) -> Result<PaSelect, Error<SpiError, PinError>> {
        let pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        match pa_config.pa_select() {
            false => Ok(PaSelect::RfoPin),
            true => Ok(PaSelect::PaBoost),
        }
    }

    pub fn set_max_power(&mut self, max_power: &u8) -> Result<(), Error<SpiError, PinError>> {
        if *max_power > 7 {
            return Err(Error::InputOutOfRange);
        }
        let mut pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        pa_config.set_max_power(*max_power);
        self.write_register(Registers::PaConfig, &pa_config.0) 
    }

    pub fn max_power(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        let pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        Ok(pa_config.max_power())
    }

    pub fn set_output_power(&mut self, output_power: &u8) -> Result<(), Error<SpiError, PinError>> {
        if *output_power > 15 {
            return Err(Error::InputOutOfRange);
        }
        let mut pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        pa_config.set_output_power(*output_power);
        self.write_register(Registers::PaConfig, &pa_config.0) 
    }

    pub fn output_power(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        let pa_config = PaConfig(self.read_register(Registers::PaConfig)?);
        Ok(pa_config.output_power())
    }

    pub fn set_lora_mode(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut op_mode = OpMode(self.read_register(Registers::OpMode)?);
        op_mode.set_long_range_mode(*on);
        self.write_register(Registers::OpMode, &op_mode.0)
    }

    pub fn lora_mode(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let op_mode = OpMode(self.read_register(Registers::OpMode)?);
        Ok(op_mode.long_range_mode())
    }

    pub fn set_transceiver_mode(&mut self, transceiver_mode: TransceiverMode) -> Result<(), Error<SpiError, PinError>> {
        let mut op_mode = OpMode(self.read_register(Registers::OpMode)?);
        op_mode.set_mode(transceiver_mode as u8);
        self.write_register(Registers::OpMode, &op_mode.0)
    }

    pub fn transceiver_mode(&mut self) -> Result<TransceiverMode, Error<SpiError, PinError>> {
        let opmode = OpMode(self.read_register(Registers::OpMode)?);

        Ok(num::FromPrimitive::from_u8(opmode.mode()).unwrap())
    }

    pub fn set_low_frequency_mode(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut op_mode = OpMode(self.read_register(Registers::OpMode)?);
        op_mode.set_low_frequency_mode_on(*on);
        self.write_register(Registers::OpMode, &op_mode.0)
    }

    pub fn low_frequency_mode(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let op_mode = OpMode(self.read_register(Registers::OpMode)?);
        Ok(op_mode.low_frequency_mode_on())
    }

    pub fn set_modulation_mode(&mut self, modulation : Modulation) -> Result<(), Error<SpiError, PinError>> {
        let mut op_mode = OpMode(self.read_register(Registers::OpMode)?);
        op_mode.set_modulation_type(modulation as u8);
        self.write_register(Registers::OpMode, &op_mode.0)
    }

    pub fn modulation_mode(&mut self) -> Result<Modulation, Error<SpiError, PinError>> {
        let op_mode = OpMode(self.read_register(Registers::OpMode)?);
        Ok(num::FromPrimitive::from_u8(op_mode.modulation_type()).unwrap())
    }

    pub fn set_frequency_deviation(&mut self, frequency_deviation: &u32) -> Result<(), Error<SpiError, PinError>> {
        if *frequency_deviation > MAX_FREQUENCY_DEVIATION {
            return Err(Error::InputOutOfRange);
        }

        let freq_dev : u32 = *frequency_deviation / F_STEP;

        self.write_register(Registers::FdevMsb, &(((freq_dev >> 8) & 0x3F) as u8))?;
        self.write_register(Registers::FdevLsb, &(((freq_dev >> 0) & 0xFF) as u8))?;

        Ok(())
    }

    pub fn frequency_deviation(&mut self) -> Result<u32, Error<SpiError,PinError>> {
        let freq_dev_msb = (self.read_register(Registers::FdevMsb)? & 0x3f) as u32;
        let freq_dev_lsb = self.read_register(Registers::FdevLsb)? as u32;

        let freq_dev = ((freq_dev_msb << 8) + (freq_dev_lsb << 0)) * F_STEP;

        Ok(freq_dev)
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