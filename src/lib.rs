#![no_std]

extern crate embedded_hal;
extern crate num;
#[macro_use]
extern crate bitfield;
#[macro_use]
extern crate num_derive;

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;

const FX_OSC: u64 = 32_000_000; // Oscillator frequency Hz

bitfield!{
    struct PaConfig(u8);
    pa_select, set_pa_select: 7;
    max_power, set_max_power: 6,4;
    output_power, set_output_power: 3,0;
}

bitfield!{
    struct OpMode(u8);
    long_range_mode, set_long_range_mode: 7;
    low_frequency_mode_on, set_low_frequency_mode_on: 3;
    mode, set_mode: 2,0; 
}

bitfield!{
    struct PaRamp(u8);
    pa_ramp, set_pa_ramp: 3,0;
}

bitfield!{
    struct Ocp(u8);
    ocp_on, set_ocp_on: 5;
    ocp_trim, set_ocp_trim: 4,0;
}

bitfield!{
    struct Lna(u8);
    lna_gain, set_lna_gain: 7,5;
    lna_boost_hf, set_lna_boost_hf: 1,0;
}

bitfield!{
    struct IrqFlagsMask(u8);
    rx_timeout_mask, set_rx_timeout_mask: 7;
    rx_done_mask, set_rx_done_mask: 6;
    payload_crc_error_mask, set_payload_crc_error_mask: 5;
    valid_header_mask, set_valid_header_mask: 4;
    tx_done_mask, set_tx_done_mask: 3;
    cad_done_mask, set_cad_done_mask: 2;
    fhss_change_channel_mask, set_fhss_change_channel_mask: 1;
    cad_detected_mask, set_cad_detected_mask: 0;
}

bitfield!{
    struct IrqFlags(u8);
    rx_timeout, clear_rx_timeout: 7;
    rx_done, clear_rx_done: 6;
    payload_crc_error, clear_payload_crc_error: 5;
    valid_header, clear_valid_header: 4;
    tx_done, clear_tx_done: 3;
    cad_done, clear_cad_done: 2;
    fhss_change_channel, clear_fhss_change_channel: 1;
    cad_detected, clear_cad_detected: 0;
}

bitfield!{
    struct ModemStat(u8);
    rx_coding_rate, _ : 7,5;
    modem_clear, _ : 4;
    header_info_valid, _ : 3;
    rx_ongoing, _ : 2;
    signal_synchronized, _ : 1;
    signal_detected, _ : 0;
}

bitfield!{
    struct ModemConfig1(u8);
    bw, set_bw: 7,4;
    coding_rate, set_coding_rate: 3,1;
    implicit_header_mode, set_implicit_header_mode: 0;
}

bitfield!{
    struct ModemConfig2(u8);
    spreading_factor, set_spreading_factor: 7,4;
    tx_continuous_mode, set_tx_continuous_mode: 3;
    rx_payload_crc_on, set_rx_payload_crc_on: 2;
    symb_timeout_msb, set_symb_timeout_msb: 1, 0;
}

bitfield!{
    struct ModemConfig3(u8);
    low_data_rate_optimize, set_low_data_rate_optimize: 3;
    agc_auto_on, set_agc_auto_on: 2;
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
pub struct SX1276<O1: OutputPin, O2: OutputPin, S: spi::Transfer<u8>, D: DelayMs<u32>>{
    reset_line: O1,
    nss_line: O2,
    spi: S,
    delay: D,
}

pub struct ModemStatus {
    modem_clear: bool,
    header_info_valid: bool,
    rx_ongoing: bool,
    signal_synchronized: bool,
    signal_detected: bool,
}

impl ModemStatus {
    pub fn modem_clear(&self) -> &bool {
        &self.modem_clear
    }

    pub fn header_info_valid(&self) -> &bool {
        &self.header_info_valid
    }

    pub fn rx_ongoing(&self) -> &bool {
        &self.rx_ongoing
    }

    pub fn signal_synchronized(&self) -> &bool {
        &self.signal_synchronized
    }

    pub fn signal_detected(&self) -> &bool {
        &self.signal_detected
    }
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum Modulation {
    FSK = 0x00,
    OOK = 0x01,
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum PaRampValue {
    Ramp3_4ms = 0b0000,
    Ramp2_0ms = 0b0001,
    Ramp1_0ms = 0b0010,
    Ramp500us = 0b0011,
    Ramp250us = 0b0100,
    Ramp125us = 0b0101,
    Ramp100us = 0b0110,
    Ramp62us  = 0b0111,
    Ramp50us  = 0b1000,
    Ramp40us  = 0b1001,
    Ramp31us  = 0b1010,
    Ramp25us  = 0b1011,
    Ramp20us  = 0b1100,
    Ramp15us  = 0b1101,
    Ramp12us  = 0b1110,
    Ramp10us  = 0b1111,
}

pub enum PaSelect {
    RfoPin,
    PaBoost,
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
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

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum LnaGainValue {
    G1 = 0b001,
    G2 = 0b010,
    G3 = 0b011,
    G4 = 0b100,
    G5 = 0b101,
    G6 = 0b110,
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum BandwidthValue {
    Bw7_8kHz = 0b0000,
    Bw10_4kHz = 0b0001,
    Bw15_6kHz = 0b0010,
    Bw20_8kHz = 0b0011,
    Bw31_25kHz = 0b0100,
    Bw41_7kHz = 0b0101,
    Bw62_5kHz = 0b0110,
    Bw125kHz = 0b0111,
    Bw250kHz = 0b1000,
    Bw500kHz = 0b1001,
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum CodingRateValue {
    Cr4_5 = 0b001,
    Cr4_6 = 0b010,
    Cr4_7 = 0b011,
    Cr4_8 = 0b100,
}

#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum SpreadingFactorValue {
    SF6 = 6,
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12,
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
    Ocp = 0x0B,
    Lna = 0x0C,
    FifoAddrPtr = 0x0D,
    FifoTxBaseAddr = 0x0E,
    FifoRxBaseAddr = 0x0F,
    FifoRxCurrentAddr = 0x10,
    IrqFlagsMask = 0x11,
    IrqFlags = 0x12,
    RxNbBytes = 0x13,
    RxHeaderCntValueMsb = 0x14,
    RxHeaderCntValueLsb = 0x15,
    RxPacketCntValueMsb = 0x16,
    RxPacketCntValueLsb = 0x17,
    ModemStat = 0x18,
    PktSnrValue = 0x19,
    PktRssiValue = 0x1A,
    RssiValue = 0x1B,
    HopChannel = 0x1C,
    ModemConfig1 = 0x1D,
    ModemConfig2 = 0x1E,
    SymbTimeoutLsb = 0x1F,
    PreambleMsb = 0x20,
    PreambleLsb = 0x21,
    PayloadLength = 0x22,
    MaxPayloadLength = 0x23,
    HopPeriod = 0x24,
    FifoRxByteAddr = 0x25,
    ModemConfig3 = 0x26,
    SyncWord = 0x39,
    DioMapping1 = 0x40,
    DioMapping2 = 0x41,
    Version = 0x42,
    PaDac = 0x4D,
}

impl<SpiError, PinError, O1: OutputPin<Error = PinError>, O2: OutputPin<Error = PinError>, S: spi::Transfer<u8, Error = SpiError>, D: DelayMs<u32>> SX1276<O1, O2, S, D> {

    pub fn new(spi: S, reset_line: O1, nss_line: O2, delay: D) -> Result::<SX1276<O1, O2, S, D>, Error<SpiError, PinError>>{
        let mut hat = SX1276 {
            reset_line,
            nss_line,
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

    pub fn write_fifo(&mut self, value: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::Fifo, value)
    }

    pub fn read_fifo(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::Fifo)
    }
    pub fn frequency(&mut self) -> Result<u32, Error<SpiError, PinError>> {
        let freq_msb = self.read_register(Registers::FrfMsb)? as u64;
        let freq_mid = self.read_register(Registers::FrfMid)? as u64;
        let freq_lsb = self.read_register(Registers::FrfLsb)? as u64;

        let freq = ((((freq_msb << 16) + (freq_mid << 8) + (freq_lsb << 0))) * FX_OSC) >> 19;

        Ok(freq as u32)
    }

    pub fn set_frequency(&mut self, frequency: &u32) -> Result<(), Error<SpiError, PinError>> {
        let freq : u64 = (((*frequency) as u64) << 19) / FX_OSC;

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

    pub fn set_pa_ramp(&mut self, pa_ramp_value: PaRampValue) -> Result<(), Error<SpiError, PinError>> {
        let mut pa_ramp = PaRamp(self.read_register(Registers::PaRamp)?);
        pa_ramp.set_pa_ramp(pa_ramp_value as u8);
        self.write_register(Registers::PaRamp, &pa_ramp.0)
    }

    pub fn pa_ramp(&mut self) -> Result<PaRampValue, Error<SpiError, PinError>> {
        let pa_ramp = PaRamp(self.read_register(Registers::PaRamp)?);
        Ok(num::FromPrimitive::from_u8(pa_ramp.pa_ramp()).unwrap())
    }

    pub fn set_over_current_protection(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut ocp = Ocp(self.read_register(Registers::Ocp)?);
        ocp.set_ocp_on(*on);
        self.write_register(Registers::Ocp, &ocp.0)
    }

    pub fn over_current_protection(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let ocp = Ocp(self.read_register(Registers::Ocp)?);
        Ok(ocp.ocp_on())
    }

    pub fn set_ocp_trim(&mut self, ocp_trim: &u8) -> Result<(), Error<SpiError, PinError>> {
        if *ocp_trim > 0x1F {
            return Err(Error::InputOutOfRange);
        }
        let mut ocp = Ocp(self.read_register(Registers::Ocp)?);
        ocp.set_ocp_trim(*ocp_trim);
        self.write_register(Registers::Ocp, &ocp.0)
    }

    pub fn ocp_trim(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        let ocp = Ocp(self.read_register(Registers::Ocp)?);
        Ok(ocp.ocp_trim())
    }

    pub fn set_lna_gain(&mut self, lna_gain: LnaGainValue) -> Result<(), Error<SpiError, PinError>> {
        let mut lna = Lna(self.read_register(Registers::Lna)?);
        lna.set_lna_gain(lna_gain as u8);
        self.write_register(Registers::Lna, &lna.0)
    }

    pub fn lna_gain(&mut self) -> Result<LnaGainValue, Error<SpiError, PinError>> {
        let lna = Lna(self.read_register(Registers::Lna)?);
        Ok(num::FromPrimitive::from_u8(lna.lna_gain()).unwrap())
    }

    pub fn set_lna_boost_hf(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut lna = Lna(self.read_register(Registers::Lna)?);
        match *on {
            true => lna.set_lna_boost_hf(0b11),
            false => lna.set_lna_boost_hf(0b00),
        };
        self.write_register(Registers::Lna, &lna.0)
    }

    pub fn set_fifo_addr_ptr(&mut self, addr: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::FifoAddrPtr, addr)
    }

    pub fn fifo_addr_ptr(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::FifoAddrPtr)
    }

    pub fn set_fifo_tx_base_addr(&mut self, addr: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::FifoTxBaseAddr, addr)
    }

    pub fn fifo_tx_base_addr(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::FifoTxBaseAddr)
    }

    pub fn set_fifo_rx_base_addr(&mut self, addr: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::FifoRxBaseAddr, addr)
    }

    pub fn fifo_rx_base_addr(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::FifoRxBaseAddr)
    }

    pub fn fifo_rx_current_addr(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::FifoRxCurrentAddr)
    }

    pub fn set_hop_period(&mut self, period: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::HopPeriod, period)
    }

    pub fn set_irq_rx_timeout_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_rx_timeout_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_rx_timeout_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.rx_timeout_mask())
    }

    pub fn set_irq_rx_done_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_rx_done_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_rx_done_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.rx_done_mask())
    }

    pub fn set_irq_payload_crc_error_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_payload_crc_error_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_payload_crc_error_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.payload_crc_error_mask())
    }

    pub fn set_irq_valid_header_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_valid_header_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_valid_header_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.valid_header_mask())
    }

    pub fn set_irq_tx_done_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_tx_done_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_tx_done_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.tx_done_mask())
    }

    pub fn set_irq_cad_done_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_cad_done_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_cad_done_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.cad_done_mask())
    }

    pub fn set_irq_fhss_change_channel_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_fhss_change_channel_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_fhss_change_channel_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.fhss_change_channel_mask())
    }

    pub fn set_irq_cad_detected_mask(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        irq_flags_mask.set_cad_detected_mask(*on);
        self.write_register(Registers::IrqFlagsMask, &irq_flags_mask.0)
    }

    pub fn irq_cad_detected_mask(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags_mask = IrqFlagsMask(self.read_register(Registers::IrqFlagsMask)?);
        Ok(irq_flags_mask.cad_detected_mask())
    }

    pub fn clear_irq_rx_timeout(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_rx_timeout(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_rx_timeout(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.rx_timeout())
    }

    pub fn clear_irq_rx_done(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_rx_done(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_rx_done(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.rx_done())
    }

    pub fn clear_irq_payload_crc_error(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_payload_crc_error(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_payload_crc_error(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.payload_crc_error())
    }

    pub fn clear_irq_valid_header(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_valid_header(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_valid_header(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.valid_header())
    }

    pub fn clear_irq_tx_done(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_tx_done(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_tx_done(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.tx_done())
    }

    pub fn clear_irq_cad_done(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_cad_done(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_cad_done(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.cad_done())
    }

    pub fn clear_irq_fhss_change_channel(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_fhss_change_channel(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_fhss_change_channel(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.fhss_change_channel())
    }

    pub fn clear_irq_cad_detected(&mut self) -> Result<(), Error<SpiError, PinError>> {
        let mut irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        irq_flags.clear_cad_detected(true);
        self.write_register(Registers::IrqFlags, &irq_flags.0)
    }

    pub fn irq_cad_detected(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let irq_flags = IrqFlags(self.read_register(Registers::IrqFlags)?);
        Ok(irq_flags.cad_detected())
    }

    pub fn rx_number_bytes(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::RxNbBytes)
    }

    pub fn rx_header_count(&mut self) -> Result<u16, Error<SpiError, PinError>> {
        let valid_header_count_msb = self.read_register(Registers::RxHeaderCntValueMsb)?;
        let valid_header_count_lsb = self.read_register(Registers::RxHeaderCntValueLsb)?;

        let valid_header_count : u16 = ((valid_header_count_msb as u16) << 8) + valid_header_count_lsb as u16;

        Ok(valid_header_count)
    }

    pub fn rx_packet_count(&mut self) -> Result<u16, Error<SpiError, PinError>> {
        let valid_packet_count_msb = self.read_register(Registers::RxPacketCntValueMsb)?;
        let valid_packet_count_lsb = self.read_register(Registers::RxPacketCntValueLsb)?;

        let valid_packet_count : u16 = ((valid_packet_count_msb as u16) << 8) + valid_packet_count_lsb as u16;

        Ok(valid_packet_count)
    }

    pub fn rx_coding_rate(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        let modem_stat = ModemStat(self.read_register(Registers::ModemStat)?);
        Ok(modem_stat.rx_coding_rate())
    }

    pub fn modem_status(&mut self) -> Result<ModemStatus, Error<SpiError, PinError>> {
        let modem_stat = ModemStat(self.read_register(Registers::ModemStat)?);
        Ok(ModemStatus{
            modem_clear: modem_stat.modem_clear(),
            header_info_valid: modem_stat.header_info_valid(),
            rx_ongoing: modem_stat.rx_ongoing(),
            signal_synchronized: modem_stat.signal_synchronized(),
            signal_detected: modem_stat.signal_detected(),
        })
    }

    /************************ TO TEST LATER *********************** */
    // TODO: Convert the two's complement later
    pub fn packet_snr_value(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::PktSnrValue)
    }

    pub fn packet_rssi_value(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::PktRssiValue)
    }

    pub fn rssi_value(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::RssiValue)
    }

    // TODO: HOP CHANNEL FUNCTION
    // pub fn hop_channel(&mut self) -> Result<HopChannel, Error<SpiError, PinError>>
    /* *********************************************** */

    pub fn set_bandwidth(&mut self, bw: BandwidthValue) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        modem_config1.set_bw(bw as u8);
        self.write_register(Registers::ModemConfig1, &modem_config1.0)
    }

    pub fn bandwidth(&mut self) -> Result<BandwidthValue, Error<SpiError, PinError>> {
        let modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        Ok(num::FromPrimitive::from_u8(modem_config1.bw()).unwrap())
    }

    pub fn set_coding_rate(&mut self, cr: CodingRateValue) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        modem_config1.set_coding_rate(cr as u8);
        self.write_register(Registers::ModemConfig1, &modem_config1.0)
    }

    pub fn coding_rate(&mut self) -> Result<CodingRateValue, Error<SpiError, PinError>> {
        let modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        Ok(num::FromPrimitive::from_u8(modem_config1.coding_rate()).unwrap())
    }

    pub fn set_implicit_header_mode(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        modem_config1.set_implicit_header_mode(*on);
        self.write_register(Registers::ModemConfig1, &modem_config1.0)
    }

    pub fn implicit_header_mode(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let modem_config1 = ModemConfig1(self.read_register(Registers::ModemConfig1)?);
        Ok(modem_config1.implicit_header_mode())
    }

    pub fn set_spreading_factor(&mut self, sf: SpreadingFactorValue) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        modem_config2.set_spreading_factor(sf as u8);
        self.write_register(Registers::ModemConfig2, &modem_config2.0)
    }

    pub fn spreading_factor(&mut self) -> Result<SpreadingFactorValue, Error<SpiError, PinError>> {
        let modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        Ok(num::FromPrimitive::from_u8(modem_config2.spreading_factor()).unwrap())
    }

    pub fn set_tx_continuous_mode(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        modem_config2.set_tx_continuous_mode(*on);
        self.write_register(Registers::ModemConfig2, &modem_config2.0)
    }

    pub fn tx_continuous_mode(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        Ok(modem_config2.tx_continuous_mode())
    }

    pub fn set_rx_payload_crc(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        modem_config2.set_rx_payload_crc_on(*on);
        self.write_register(Registers::ModemConfig2, &modem_config2.0)
    }

    pub fn rx_payload_crc(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let modem_config2 = ModemConfig2(self.read_register(Registers::ModemConfig2)?);
        Ok(modem_config2.rx_payload_crc_on())
    }

    pub fn set_symb_timeout(&mut self, value: &u8) -> Result<(), Error<SpiError, PinError>> {
        self.write_register(Registers::SymbTimeoutLsb, value)
    }

    // TODO: PREAMBLE_LENGTH

    pub fn set_payload_length(&mut self, length: &u8) -> Result<(), Error<SpiError, PinError>> {
        if *length < 1 {
            return Err(Error::InputOutOfRange);
        }
        self.write_register(Registers::PayloadLength, length)
    }

    pub fn payload_length(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::PayloadLength)
    }

    pub fn set_payload_max_length(&mut self, length: &u8) -> Result<(), Error<SpiError, PinError>> {
        if *length < 1 {
            return Err(Error::InputOutOfRange);
        }
        self.write_register(Registers::MaxPayloadLength, length)
    }

    pub fn payload_max_length(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::MaxPayloadLength)
    }

    //TODO: Freq hopping period

    pub fn fifo_rx_byte_addr_ptr(&mut self) -> Result<u8, Error<SpiError, PinError>> {
        self.read_register(Registers::FifoRxByteAddr)
    }

    pub fn set_low_data_rate_optimize(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config3 = ModemConfig3(self.read_register(Registers::ModemConfig3)?);
        modem_config3.set_low_data_rate_optimize(*on);
        self.write_register(Registers::ModemConfig3, &modem_config3.0)
    }

    pub fn low_data_rate_optimize(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let modem_config3 = ModemConfig3(self.read_register(Registers::ModemConfig3)?);
        Ok(modem_config3.low_data_rate_optimize())
    }

    pub fn set_agc_auto(&mut self, on: &bool) -> Result<(), Error<SpiError, PinError>> {
        let mut modem_config3 = ModemConfig3(self.read_register(Registers::ModemConfig3)?);
        modem_config3.set_agc_auto_on(*on);
        self.write_register(Registers::ModemConfig3, &modem_config3.0)
    }

    pub fn agc_auto(&mut self) -> Result<bool, Error<SpiError, PinError>> {
        let modem_config3 = ModemConfig3(self.read_register(Registers::ModemConfig3)?);
        Ok(modem_config3.agc_auto_on())
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