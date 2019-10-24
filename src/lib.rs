//! Platform agnostic LoRa interface driver for the Semtech SX127x chips.
//! 
//! The software interface follows closely the register description table specification given in the datasheets of the
//! [SX1276](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1276). The read (getter) functions are named like the register using
//! Rust naming conventions, e.g. ```self.coding_rate()``` returns the CodingRate value. The write (setter) functions are prepended with *set_* 
//! or *clear_* when the write operation results in the bit being cleared. In some cases, for example for setting carrier frequency, convinience functions
//! are provided to abstract numerical or other conversions. In those cases, the word *_register* has been applied to the getter and setter functions.
//! For example, ```self.set_frequency_register(value: u32)``` would directly set value to the frequency registers whereas ```self.set_frequency(value_khz: u32)```
//! converts the input in kHz to the equivalent integer value before setting the registers.
//! 
//! This driver was built using [`embedded-hal`] traits and does not depend on std.
//! 
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//! 
//! **Note:** Only the LoRa interface is implemented.

#![no_std]
extern crate embedded_hal;
extern crate num;
#[macro_use]
extern crate num_derive;

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;

macro_rules! sx127x_getter_function {
    ( $(#[$outer:meta])* $function_name:ident, 7, 0, $register_name:ident, $reg_type:ty ) => {
        $(#[$outer])*
        pub fn $function_name (&mut self) -> Result<$reg_type, Error<SpiError, PinError>> {
            let var = self.read_register(Registers::$register_name)?;

            Ok ( <$reg_type>::from(var) )
        }
    };

    ( $(#[$outer:meta])* $function_name:ident, $msb:tt, _, $register_name:ident, $reg_type:ty ) => {
        $(#[$outer])*
        pub fn $function_name (&mut self) -> Result<$reg_type, Error<SpiError, PinError>> {
            let var = self.read_register(Registers::$register_name)?;
            
            Ok( <$reg_type>::from(((var & 1<<$msb) >> $msb) != 0) )
        }
    };

    ( $(#[$outer:meta])* $function_name:ident, $msb:tt, $lsb:tt, $register_name:ident, $reg_type:ty ) => {
        $(#[$outer])*
        pub fn $function_name (&mut self) -> Result<$reg_type, Error<SpiError, PinError>> {
            let var = self.read_register(Registers::$register_name)?;
            
            let mut bitmask = 0;
            for i in $lsb..=$msb {
                bitmask = bitmask | (1<<i);
            }

            Ok(num::FromPrimitive::from_u8(((var & bitmask) >> $lsb)).unwrap())
        }
    };

    ( $(#[$outer:meta])* $function_name:ident, [$($register_name:ident)+], $reg_type:ty ) => {
        $(#[$outer])*
        #[allow(unused_assignments)]
        pub fn $function_name (&mut self) -> Result<$reg_type, Error<SpiError, PinError>> {
            let mut index = 0;
            let mut result : $reg_type = 0; 

            $(
                result = result + ((self.read_register(Registers::$register_name)? as $reg_type) << (index*8));
                index = index + 1;
            )+

            Ok(result)
        }
    };
}

macro_rules! sx127x_setter_function {
    ( $function_name:ident, 7, 0, $register_name:ident, $reg_type:ty ) => {
        pub fn $function_name (&mut self, value: $reg_type) -> Result<(), Error<SpiError, PinError>> {
            self.write_register(Registers::$register_name, &(value as u8) )
        }
    };

    ( $function_name:ident, $msb:tt, _, $register_name:ident, $reg_type:ty ) => {
        pub fn $function_name (&mut self, value: $reg_type) -> Result<(), Error<SpiError, PinError>> {
            let mut reg_value = self.read_register(Registers::$register_name)?;

            reg_value = match value as u8{
                1 => reg_value | (1<<$msb),
                0 => reg_value & !(1<<$msb),
                _ => reg_value,
            };

            self.write_register(Registers::$register_name, &reg_value)
        }
    };

    ( $function_name:ident, $msb:tt, $lsb:tt, $register_name:ident, $reg_type:ty ) => {
        pub fn $function_name (&mut self, value: $reg_type) -> Result<(), Error<SpiError, PinError>> {
            // Create a bitmask and check the input range
            let mut bitmask = 0;
            for i in $lsb..=$msb {
                bitmask = bitmask | (1<<i);
            }

            let write_value = value as u8;

            if write_value > (bitmask >> $lsb) {
                return Err(Error::InputOutOfRange); 
            }

            let mut reg_value = self.read_register(Registers::$register_name)?;

            // Zero all the bits of the value to be changed before writting the new value
            reg_value = reg_value & !bitmask;
            
            reg_value = reg_value | (write_value << $lsb);

            self.write_register(Registers::$register_name, &reg_value)
        }
    };

    ( $(#[$outer:meta])* $function_name:ident, [$($register_name:ident)+], $reg_type:ty ) => {
        $(#[$outer])*
        #[allow(unused_assignments)]
        pub fn $function_name (&mut self, value: $reg_type) -> Result<(), Error<SpiError, PinError>> {
            let mut index = 0;

            $(
                self.write_register(Registers::$register_name, &((value >> (index*8)) as u8))?;
                index = index + 1;
            )+

            Ok(())
        }
    };
} 

macro_rules! sx127x_interface {
    ( $(#[$outer:meta])* $register_name:ident, $msb:tt, $lsb:tt, $read_function_name:ident, _, $reg_type:ty ) => {
        sx127x_getter_function!($(#[$outer])* $read_function_name, $msb, $lsb, $register_name, $reg_type); 
    };

    ( $(#[$outer:meta])* $register_name:ident, $msb:tt, $lsb:tt, $read_function_name:ident, $write_function_name:ident, $reg_type:ty ) => {
        sx127x_getter_function!($(#[$outer])* $read_function_name, $msb, $lsb, $register_name, $reg_type);
        sx127x_setter_function!($write_function_name, $msb, $lsb, $register_name, $reg_type);
    };

    ( $( $(#[$outer:meta])* $register_name:ident, $msb:tt, $lsb:tt, $read_function_name:ident, $write_function_name:tt, $reg_type:ty ;)* ) => {
        $( sx127x_interface!($(#[$outer])* $register_name, $msb, $lsb, $read_function_name, $write_function_name, $reg_type); )*
    };
}

macro_rules! sx127x_multiregister_interface {

    ( $(#[$outer:meta])* [$($register_name:ident)+], $read_function_name:ident, $write_function_name:ident, $reg_type:ty ) => {
        sx127x_getter_function!($(#[$outer])* $read_function_name, [$($register_name)+], $reg_type); 
        sx127x_setter_function!($(#[$outer])* $write_function_name, [$($register_name)+], $reg_type); 
    };

    ( $(#[$outer:meta])* [$($register_name:ident)+], $read_function_name:ident, _, $reg_type:ty ) => {
        sx127x_getter_function!($(#[$outer])* $read_function_name, [$($register_name)+], $reg_type); 
    };

    ( $( $(#[$outer:meta])* [$($register_name:ident),+], $read_function_name:ident, $write_function_name:tt, $reg_type:ty ;)* ) => {
        $( sx127x_multiregister_interface!($(#[$outer])* [$($register_name)+], $read_function_name, $write_function_name, $reg_type); )*
    };
}

const FX_OSC: u64 = 32_000_000; // Oscillator frequency Hz

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

/// Convinience struct to collect all the modem status read from RegModemStat.
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

impl num::FromPrimitive for ModemStatus {
    fn from_u8(val: u8) ->  core::option::Option<ModemStatus> {
        Some(ModemStatus{
            modem_clear: (val & 0b10000 != 0),
            header_info_valid: (val & 0b01000 != 0),
            rx_ongoing: (val & 0b00100 != 0),
            signal_synchronized: (val & 0b00010 != 0),
            signal_detected: (val & 0b00001 != 0),
        })
    }

    fn from_i64(_val: i64) ->  core::option::Option<ModemStatus> {
        None
    }

    fn from_u64(_val: u64) ->  core::option::Option<ModemStatus> {
        None
    }
}

/// PA Ramp settings
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

/// PA selection
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum PaSelect {
    RfoPin = 0,
    PaBoost = 1,
}

impl From<bool> for PaSelect {
    fn from (i: bool) -> Self {
        num::FromPrimitive::from_u8(i as u8).unwrap()
    }
}

/// Transceiver moder
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

/// LNA gain values. G1 -> Max gain
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum LnaGainValue {
    G1 = 0b001, 
    G2 = 0b010,
    G3 = 0b011,
    G4 = 0b100,
    G5 = 0b101,
    G6 = 0b110,
}

/// LNA boost values
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum LnaBoostValue {
    Off = 0b00,
    On = 0b11,
}

#[cfg(feature = "sx1272")]
/// Bandwidth values for SX1272/3 chip
pub enum BandwidthValue {
    Bw125kHz = 0b00,
    Bw250kHz = 0b01,
    Bw500kHz = 0b10,
}

#[cfg(not(feature = "sx1272"))]
/// Bandwidth values for SX1276/7/8 chip
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

/// Coding rate values
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
pub enum CodingRateValue {
    Cr4_5 = 0b001,
    Cr4_6 = 0b010,
    Cr4_7 = 0b011,
    Cr4_8 = 0b100,
}

/// Spreading factor values
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

/// Detection optimize values
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
#[allow(non_camel_case_types)]
pub enum DetectionOptimizeValue {
    SF7_to_SF12 = 0x03,
    SF6 = 0x05,
}

/// Detection threshold values
#[derive(FromPrimitive, Debug, Clone, PartialEq)]
#[allow(non_camel_case_types)]
pub enum DetectionThresholdValue {
    SF7_to_SF12 = 0x0A,
    SF6 = 0x0C,
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
    FifoRxBytesNb = 0x13,
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
    FeiMsb = 0x28,
    FeiMid = 0x29,
    FeiLsb = 0x2A,
    RssiWideband = 0x2C,
    DetectOptimize = 0x31,
    InvertIQ = 0x33,
    DetectionThreshold = 0x37,
    SyncWord = 0x39,
    DioMapping1 = 0x40,
    DioMapping2 = 0x41,
    Version = 0x42,
    PaDac = 0x4D,
}

/// Interface for SX1272/3/6/7/8 chips.
pub struct SX127x<O1: OutputPin, O2: OutputPin, S: spi::Transfer<u8>, D: DelayMs<u32>>{
    reset_line: O1,
    nss_line: O2,
    spi: S,
    delay: D,
}

impl<SpiError, PinError, O1: OutputPin<Error = PinError>, O2: OutputPin<Error = PinError>, S: spi::Transfer<u8, Error = SpiError>, D: DelayMs<u32>> SX127x<O1, O2, S, D> {

    pub fn new(spi: S, reset_line: O1, nss_line: O2, delay: D) -> Result::<SX127x<O1, O2, S, D>, Error<SpiError, PinError>>{
        let mut hat = SX127x {
            reset_line,
            nss_line,
            spi,
            delay,
        };

        hat.reset()?;

        // Support only SX1276 for now. If another board is detect return an error.
        if cfg!(not(sx1272)) && hat.version()? != 0x12 {
           Err(Error::InvalidVersion)
        }
        else if cfg!(sx1272) && hat.version()? != 0x22 {
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

        self.delay.delay_ms(10);

        self.reset_line.set_high().map_err(|e| Error::Pin(e))?;

        self.delay.delay_ms(100);

        Ok(())
    }

    sx127x_interface!(
        /// LoRa (TM) base-band FIFO data input/output. FIFO is cleared and not
        /// accessible when device is in SLEEP mode
        Fifo, 7, 0, read_fifo, write_fifo, u8;

        /// Change between 0 -> FSK/OOK Mode and 1 -> LoRa (TM) Mode.
        /// This bit can be modified only in Sleep mode. A write operation on
        /// other device modes is ignored.
        OpMode, 7, _, lora_mode, set_lora_mode, bool;

        /// This bit operates when device is in Lora mode; if set it allows
        /// access to FSK registers page located in address space
        /// (0x0D:0x3F) while in LoRa mode.
        /// 0 -> Access LoRa registers page 0x0D: 0x3F
        /// 1 -> Access FSK registers page (in mode LoRa) 0x0D: 0x3F
        OpMode, 6, _, access_shared_reg, set_access_shared_reg, bool;

        /// Access Low Frequency Mode registers
        /// 0 -> High Frequency Mode (access to HF test registers)
        /// 1 -> Low Frequency Mode (access to LF test registers)
        OpMode, 3, _, low_frequency_mode, set_low_frequency_mode, bool;

        /// Device modes
        OpMode, 2, 0, transceiver_mode, set_transceiver_mode, TransceiverMode;

        // TODO: Frequency registers
        /// Selects PA output pin
        /// 0 -> RFO pin. Output power is limited to +14 dBm.
        /// 1 -> PA_BOOST pin. Output power is limited to +20 dBm
        PaConfig, 7, _, pa_select, set_pa_select, PaSelect;

        /// Select max output power: Pmax=10.8+0.6*MaxPower \[dBm\]
        PaConfig, 6, 4, max_power, set_max_power, u8;

        /// Output power
        /// Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)
        /// Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
        PaConfig, 3, 0, output_power, set_output_power, u8;

        /// Rise/Fall time of ramp up/down in FSK (probably meant LoRa in datasheet)
        PaRamp, 3, 0, pa_ramp, set_pa_ramp, PaRampValue;

        /// Overload current protection (OCP) for PA
        Ocp, 5, _, over_current_protection, set_over_current_protection, bool;

        /// Trimming of OCP current:
        /// Imax = 45+5*OcpTrim \[mA\] if OcpTrim <= 15 (120 mA) 
        /// Imax = -30+10*OcpTrim \[mA\] if 15 < OcpTrim <= 27 (130 to 240 mA)
        /// Imax = 240mA for higher settings
        /// Default Imax = 100mA
        Ocp, 4, 0, ocp_trim, set_ocp_trim, u8;

        /// LNA gain setting
        Lna, 7, 5, lna_gain, set_lna_gain, LnaGainValue;

        /// High Frequency (RFI_HF) LNA current adjustment
        /// 00 -> Default LNA current
        /// 11 -> Boost on, 150% LNA current
        Lna, 1, 0, lna_boost_hf, set_lna_boost_hf, LnaBoostValue;

        /// SPI interface address pointer in FIFO data buffer.
        FifoAddrPtr, 7, 0, fifo_addr_ptr, set_fifo_addr_ptr, u8;

        /// Write base address in FIFO data buffer for TX modulator
        FifoTxBaseAddr, 7, 0, fifo_tx_base_addr, set_fifo_tx_base_addr, u8;

        /// Read base address in FIFO data buffer for RX demodulator
        FifoRxBaseAddr, 7, 0, fifo_rx_base_addr, set_fifo_rx_base_addr, u8;

        /// Start address (in data buffer) of last packet received
        FifoRxCurrentAddr, 7, 0, fifo_rx_current_addr, _, u8;

        /// Timeout interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 7, _, irq_rx_timeout_mask, set_irq_rx_timeout_mask, bool;

        /// Packet reception complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 6, _, irq_rx_done_mask, set_irq_rx_done_mask, bool;

        /// Payload CRC error interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 5, _, irq_payload_crc_error_mask, set_irq_payload_crc_error_mask, bool;

        /// Valid header received in Rx mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 4, _, irq_valid_header_mask, set_irq_valid_header_mask, bool;

        /// FIFO Payload transmission complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 3, _, irq_tx_done_mask, set_irq_tx_done_mask, bool;

        /// CAD complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 2, _, irq_cad_done_mask, set_irq_cad_done_mask, bool;

        /// FHSS change channel interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 1, _, irq_fhss_change_channel_mask, set_irq_fhss_change_channel_mask, bool;

        /// Cad Detected Interrupt Mask: setting this bit masks the corresponding IRQ in RegIrqFlags
        IrqFlagsMask, 0, _, irq_cad_detected_mask, set_irq_cad_detected_mask, bool;

        /// Timeout interrupt: writing a 1 clears the IRQ
        IrqFlags, 7, _, irq_rx_timeout, clear_irq_rx_timeout, bool;

        /// Packet reception complete interrupt: writing a 1 clears the IRQ
        IrqFlags, 6, _, irq_rx_done, clear_irq_rx_done, bool;

        /// Payload CRC error interrupt: writing a 1 clears the IRQ
        IrqFlags, 5, _, irq_payload_crc_error, clear_irq_payload_crc_error, bool;

        /// Valid header received in Rx: writing a 1 clears the IRQ
        IrqFlags, 4, _, irq_valid_header, clear_irq_valid_header, bool;

        /// FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ
        IrqFlags, 3, _, irq_tx_done, clear_irq_tx_done, bool;

        /// CAD complete: write to clear: writing a 1 clears the IRQ
        IrqFlags, 2, _, irq_cad_done, clear_irq_cad_done, bool;

        /// FHSS change channel interrupt: writing a 1 clears the IRQ
        IrqFlags, 1, _, irq_fhss_change_channel, clear_irq_fhss_change_channel, bool;

        /// Valid Lora signal detected during CAD operation: writing a 1 clears the IRQ
        IrqFlags, 0, _, irq_cad_detected, clear_irq_cad_detected, bool;

        /// Number of payload bytes of latest packet received
        FifoRxBytesNb, 7, 0, rx_number_bytes, _, u8;

        /// Coding rate of last header received
        ModemStat, 7, 5, rx_coding_rate, _, u8;

        /// Modem status containing: Modem clear; Header info valid; RX on-going; Signal synchronized; Signal detected
        ModemStat, 4, 0, modem_status, _, ModemStatus;

        /// Estimation of SNR on last packet received. In two’s compliment format mutiplied by 4.
        PktSnrValue, 7, 0, packet_snr, _, u8;

        /// RSSI of the latest packet received (dBm):
        ///     RSSI(dBm) = -157 + Rssi (using HF output port, SNR >= 0)
        /// or
        ///     RSSI(dBm) = -164 + Rssi (using LF output port, SNR >= 0)
        PktRssiValue, 7, 0, packet_rssi, _, u8;

        /// Current RSSI value (dBm):
        ///     RSS(dBm) = -157 + Rssi (using HF output port, SNR >= 0)
        /// or
        ///     RSSI(dBm) = -164 + Rssi (using LF output port, SNR >= 0)
        RssiValue, 7, 0, rssi, _, u8;

        /// PLL failed to lock while attempting a TX/RX/CAD operation.
        /// 1 -> PLL did not lock
        /// 0 -> PLL did lock
        HopChannel, 7, _, pll_timeout, _, bool;

        /// CRC Information extracted from the received packet header
        /// (Explicit header mode only)
        /// 0 -> Header indicates CRC off
        /// 1 -> Header indicates CRC on
        HopChannel, 6, _, crc_on_payload, _, bool;

        /// Current value of frequency hopping channel in use.
        HopChannel, 5, 0, fhhs_present_channel, _, u8;

        /// Signal bandwidth. In the lower band (169MHz), signal bandwidths 250kHz and 500kHz are not supported
        ModemConfig1, 7, 4, bandwidth, set_bandwidth, BandwidthValue;

        /// Error coding rate. In implicit header mode should be set on receiver to determine expected coding rate.
        ModemConfig1, 3, 1, coding_rate, set_coding_rate, CodingRateValue;

        /// Configure implicit header mode
        ModemConfig1, 0, _, implicit_header_mode, set_implicit_header_mode, bool;

        /// SF rate (expressed as a base-2 logarithm)
        ModemConfig2, 7, 4, spreading_factor, set_spreading_factor, SpreadingFactorValue;

        /// Tx Continuous mode
        /// 0 -> normal mode, a single packet is sent
        ///  1 -> continuous mode, send multiple packets across the FIFO (used for spectral analysis)
        ModemConfig2, 3, _, tx_continuous_mode, set_tx_continuous_mode, bool;

        /// Enable CRC generation and check on payload. If CRC is needed, RxPayloadCrcOn should be set:
        /// - in Implicit header mode: on Tx and Rx side
        /// - in Explicit header mode: on the Tx side alone (recovered from the header in Rx side)
        ModemConfig2, 2, _, rx_payload_crc_on, set_rx_payload_crc_on, bool;

        /// Payload length in bytes. The register needs to be set in implicit
        /// header mode for the expected packet length. A 0 value is not permitted
        PayloadLength, 7, 0, payload_length, set_payload_length, u8;

        /// Maximum payload length; if header payload length exceeds value a
        /// header CRC error is generated. Allows filtering of packet with a bad size.
        MaxPayloadLength, 7, 0, payload_max_length, set_payload_max_length, u8;

        /// Symbol periods between frequency hops. (0 = disabled).
        /// 1st hop always happen after the 1st header symbol
        HopPeriod, 7, 0, freq_hopping_period, set_freq_hopping_period, u8;

        /// Current value of RX databuffer pointer (address of last byte written by Lora receiver)
        FifoRxByteAddr, 7, 0, fifo_rx_byte_addr_ptr, _, u8;

        /// Low data rate optimize. 1 -> Enabled; mandated for when the symbol length exceeds 16ms
        ModemConfig3, 3, _, low_data_rate_optimize, set_low_data_rate_optimize, bool;

        /// 0 -> LNA gain set by register LnaGain
        /// 1 -> LNA gain set by the internal AGC loop
        ModemConfig3, 2, _, agc_auto_on, set_agc_auto_on, bool;

        /// Wideband RSSI measurement used to locally generate a random number
        RssiWideband, 7, 0, rssi_wideband, _, u8;
        
        /// LoRa Detection Optimize
        DetectOptimize, 2, 0, detection_optimize, set_detection_optimize, DetectionOptimizeValue;

        /// Invert the LoRa I and Q signals
        InvertIQ, 6, _, invert_iq, set_invert_iq, bool;
        
        /// LoRa detection threshold
        // This line differs from the datasheet to avoid getting into trouble with using a non u8 value for representing an enum
        // in the macro system. Could be fixed with a from but that would probably make it panic.
        DetectionThreshold, 6, 0, detection_threshold, set_detection_threshold, DetectionThresholdValue;

        /// LoRa Sync Word. Value 0x34 is reserved for LoRaWAN networks
        SyncWord, 7, 0, sync_word, set_sync_word, u8;

        /// Version of the chip
        Version, 7, 0, version, _, u8;
    );

    sx127x_multiregister_interface!(
        /// Preamble length, = PreambleLength + 4.25 Symbols
        [PreambleMsb, PreambleLsb], preamble_length, set_preamble_length, u16;

        /// RF carrier frequency in integer. frf = (FX_OSC * Frf)/2^19
        [FrfLsb, FrfMid, FrfMsb], frequency_register, set_frequency_register, u32;

        /// Number of valid headers received since last transition into
        /// Rx mode. Header and packet counters are reseted in Sleep mode.
        [RxHeaderCntValueLsb, RxHeaderCntValueMsb], rx_header_count, _, u16;

        /// Number of valid packets received since last transition into 
        /// Rx mode. Header and packet counters are reseted in Sleep mode.
        [RxPacketCntValueLsb, RxPacketCntValueMsb], rx_packet_count, _, u16;

        /// Estimated frequency error from modem
        [FeiLsb, FeiMid, FeiMsb], estimated_frequency_error_register, _, u32;
    );

    /// Read the RF carrier frequency in kHz
    pub fn frequency(&mut self) -> Result<u32, Error<SpiError, PinError>> {
        let freq_int = self.frequency_register()? as u64;

        let freq = (freq_int* FX_OSC) >> 19;

        Ok(freq as u32)
    }

    /// Set the RF carrier frequency in kHz
    pub fn set_frequency(&mut self, frequency: &u32) -> Result<(), Error<SpiError, PinError>> {
        let freq : u64 = (((*frequency) as u64) << 19) / FX_OSC;

        self.set_frequency_register(freq as u32)
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