extern crate lorahatlib;
extern crate embedded_hal_mock;

use lorahatlib::SX1276;
use lorahatlib::{PaSelect,TransceiverMode, PaRampValue, LnaGainValue, BandwidthValue, CodingRateValue, SpreadingFactorValue};

use std::vec::Vec;

use embedded_hal_mock::pin::{Transaction as PinTransaction, Mock as PinMock, State as PinState};

use embedded_hal_mock::spi::{Mock as SpiMock, Transaction as SpiTransaction};

use embedded_hal_mock::delay::MockNoop;

fn get_sx1276_spi_chip_select_transactions() -> Vec<PinTransaction> {
    vec![
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ]
}

fn get_sx1276_create_transactions() -> (Vec<PinTransaction>, Vec<PinTransaction>, Vec<SpiTransaction>) {
    let reset_expectations = vec![
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High)
    ];
    
    let nss_expectations = get_sx1276_spi_chip_select_transactions();

    let spi_transactions = vec![
        SpiTransaction::transfer(vec![0x42,0x00],vec![0x00,0x12]),
    ];

    (reset_expectations, nss_expectations, spi_transactions)
}

fn create_sx1276(reset_expectations: &Vec<PinTransaction>, nss_expectations: &Vec<PinTransaction>, spi_expectations: &Vec<SpiTransaction>) -> SX1276<PinMock, SpiMock, MockNoop> {
    let reset_line = PinMock::new(reset_expectations);
    let nss_line =  PinMock::new(nss_expectations);
    let spi = SpiMock::new(spi_expectations);
    let delay = MockNoop::new();

    SX1276::new(spi, reset_line, nss_line, delay).unwrap()
}

#[test]
fn test_create_lora() {
    let (reset_expectations,nss_expectations,spi_transactions) = get_sx1276_create_transactions();

    let reset_line = PinMock::new(&reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);

    let spi = SpiMock::new(&spi_transactions);

    // No delays in the operation
    let delay = MockNoop::new();

    let _lora = SX1276::new(spi, reset_line, nss_line, delay).unwrap();
}

#[test]
fn test_create_wrong_lora_chip()
{
    // Get the original expectation and modify the SPI response for the wrong version
    let (reset_expectations,nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions[0] = SpiTransaction::transfer(vec![0x42,0x00],vec![0x00,0x13]);

    let reset_line = PinMock::new(&reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);


    let spi = SpiMock::new(&spi_transactions);

    // No delays in the operation
    let delay = MockNoop::new();

    assert_eq!(SX1276::new(spi, reset_line, nss_line, delay).err(), Some(lorahatlib::Error::InvalidVersion));
}

#[test]
fn test_set_frequency()
{
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x86,0xD9],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x87,0x06],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x88,0x66],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_frequency(&868100000).unwrap();

}

#[test]
fn test_get_frequency()
{
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x06,0x00],vec![0x00,0xD9]),
        SpiTransaction::transfer(vec![0x07,0x00],vec![0x00,0x06]),
        SpiTransaction::transfer(vec![0x08,0x00],vec![0x00,0x66]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    assert!(((sx1276.frequency().unwrap() as i32)-868100000).abs() < 30);
}

#[test]
fn test_pa_select() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x89,0x7F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x89,0x80],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_pa_select(PaSelect::RfoPin).unwrap();
    sx1276.set_pa_select(PaSelect::PaBoost).unwrap();
}

#[test]
fn test_max_power() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x89,0x8F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x89,0x50],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_max_power(&0).unwrap();
    sx1276.set_max_power(&5).unwrap();

    assert_eq!(sx1276.set_max_power(&10).err(),Some(lorahatlib::Error::InputOutOfRange));
}

#[test]
fn test_output_power() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x89,0xF0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x09,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x89,0x05],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_output_power(&0).unwrap();
    sx1276.set_output_power(&5).unwrap();

    assert_eq!(sx1276.set_output_power(&16).err(),Some(lorahatlib::Error::InputOutOfRange));
}

#[test]
fn test_lora_mode() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x81,0x7F],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_lora_mode(&true).unwrap();
    sx1276.set_lora_mode(&false).unwrap();
}

#[test]
fn test_transceiver_mode() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x81,0xF8],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x01],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x02],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x03],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x04],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x05],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x06],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x81,0x07],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_transceiver_mode(TransceiverMode::Sleep).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::Standby).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::FsTx).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::Tx).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::FsRx).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::Rx).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::RxSingle).unwrap();
    sx1276.set_transceiver_mode(TransceiverMode::CAD).unwrap();
}

#[test]
fn test_low_frequency_mode() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x81,0xFF],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x01,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x81,0xF7],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_low_frequency_mode(&true).unwrap();
    sx1276.set_low_frequency_mode(&false).unwrap();
}

#[test]
fn test_pa_ramp() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8A,0xF0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x01],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x02],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x03],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x04],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x05],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x06],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x07],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x08],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x09],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0A],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0B],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0C],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0D],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0E],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0A,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8A,0x0F],vec![0x00,0x00]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_pa_ramp(PaRampValue::Ramp3_4ms).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp2_0ms).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp1_0ms).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp500us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp250us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp125us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp100us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp62us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp50us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp40us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp31us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp25us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp20us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp15us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp12us).unwrap();
    sx1276.set_pa_ramp(PaRampValue::Ramp10us).unwrap();
}

#[test]
fn test_over_current_protection_on() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0B,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8B,0xDF],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0B,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8B,0x20],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_over_current_protection(&false).unwrap();
    sx1276.set_over_current_protection(&true).unwrap();
}

#[test]
fn test_ocp_trim() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0B,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8B,0xE0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0B,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8B,0x05],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_ocp_trim(&0).unwrap();
    sx1276.set_ocp_trim(&5).unwrap();

    assert_eq!(sx1276.set_output_power(&32).err(),Some(lorahatlib::Error::InputOutOfRange));
}

#[test]
fn test_lna_gain() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8C,0x3F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0x40],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0x60],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0xA0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0xC0],vec![0x00,0x00]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_lna_gain(LnaGainValue::G1).unwrap();
    sx1276.set_lna_gain(LnaGainValue::G2).unwrap();
    sx1276.set_lna_gain(LnaGainValue::G3).unwrap();
    sx1276.set_lna_gain(LnaGainValue::G4).unwrap();
    sx1276.set_lna_gain(LnaGainValue::G5).unwrap();
    sx1276.set_lna_gain(LnaGainValue::G6).unwrap();
}

#[test]
fn test_lna_boost_hf() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8C,0xFC],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0C,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x8C,0x03],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_lna_boost_hf(&false).unwrap();
    sx1276.set_lna_boost_hf(&true).unwrap();
}

#[test]
fn test_base_addresses() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x0D,0x00],vec![0x00,0x80]),
        SpiTransaction::transfer(vec![0x8D,0x01],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0E,0x00],vec![0x00,0x0F]),
        SpiTransaction::transfer(vec![0x8E,0x03],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x0F,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x8F,0x02],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x10,0x00],vec![0x00,0x11]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    assert_eq!(sx1276.fifo_addr_ptr().unwrap(),0x80);
    sx1276.set_fifo_addr_ptr(&0x01).unwrap();
    assert_eq!(sx1276.fifo_tx_base_addr().unwrap(),0x0F);
    sx1276.set_fifo_tx_base_addr(&0x03).unwrap();
    assert_eq!(sx1276.fifo_rx_base_addr().unwrap(),0xFF);
    sx1276.set_fifo_rx_base_addr(&0x02).unwrap();
    assert_eq!(sx1276.fifo_rx_current_addr().unwrap(),0x11);
}

#[test]
fn test_irq_flag_masks() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x40],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x20],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x10],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x08],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x04],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x02],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x91,0x01],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x01]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x02]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x04]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x08]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x10]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x20]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x40]),
        SpiTransaction::transfer(vec![0x11,0x00],vec![0x00,0x80]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_irq_rx_timeout_mask(&true).unwrap();
    sx1276.set_irq_rx_done_mask(&true).unwrap();
    sx1276.set_irq_payload_crc_error_mask(&true).unwrap();
    sx1276.set_irq_valid_header_mask(&true).unwrap();
    sx1276.set_irq_tx_done_mask(&true).unwrap();
    sx1276.set_irq_cad_done_mask(&true).unwrap();
    sx1276.set_irq_fhss_change_channel_mask(&true).unwrap();
    sx1276.set_irq_cad_detected_mask(&true).unwrap();
    assert_eq!(sx1276.irq_cad_detected_mask().unwrap(),true);
    assert_eq!(sx1276.irq_fhss_change_channel_mask().unwrap(),true);
    assert_eq!(sx1276.irq_cad_done_mask().unwrap(),true);
    assert_eq!(sx1276.irq_tx_done_mask().unwrap(),true);
    assert_eq!(sx1276.irq_valid_header_mask().unwrap(),true);
    assert_eq!(sx1276.irq_payload_crc_error_mask().unwrap(),true);
    assert_eq!(sx1276.irq_rx_done_mask().unwrap(),true);
    assert_eq!(sx1276.irq_rx_timeout_mask().unwrap(),true);
}

#[test]
fn test_irq_flag() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x40],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x20],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x10],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x08],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x04],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x02],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x92,0x01],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x01]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x02]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x04]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x08]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x10]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x20]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x40]),
        SpiTransaction::transfer(vec![0x12,0x00],vec![0x00,0x80]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.clear_irq_rx_timeout().unwrap();
    sx1276.clear_irq_rx_done().unwrap();
    sx1276.clear_irq_payload_crc_error().unwrap();
    sx1276.clear_irq_valid_header().unwrap();
    sx1276.clear_irq_tx_done().unwrap();
    sx1276.clear_irq_cad_done().unwrap();
    sx1276.clear_irq_fhss_change_channel().unwrap();
    sx1276.clear_irq_cad_detected().unwrap();
    assert_eq!(sx1276.irq_cad_detected().unwrap(),true);
    assert_eq!(sx1276.irq_fhss_change_channel().unwrap(),true);
    assert_eq!(sx1276.irq_cad_done().unwrap(),true);
    assert_eq!(sx1276.irq_tx_done().unwrap(),true);
    assert_eq!(sx1276.irq_valid_header().unwrap(),true);
    assert_eq!(sx1276.irq_payload_crc_error().unwrap(),true);
    assert_eq!(sx1276.irq_rx_done().unwrap(),true);
    assert_eq!(sx1276.irq_rx_timeout().unwrap(),true);
}

#[test]
fn test_rx_header_count() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x14,0x00],vec![0x00,0x80]),
        SpiTransaction::transfer(vec![0x15,0x00],vec![0x00,0x08]),
        SpiTransaction::transfer(vec![0x14,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x15,0x00],vec![0x00,0xFF]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    assert_eq!(sx1276.rx_header_count().unwrap(),0x8008);
    assert_eq!(sx1276.rx_header_count().unwrap(),0xFFFF);
}

#[test]
fn test_rx_packet_count() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x16,0x00],vec![0x00,0x80]),
        SpiTransaction::transfer(vec![0x17,0x00],vec![0x00,0x08]),
        SpiTransaction::transfer(vec![0x16,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x17,0x00],vec![0x00,0xFF]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    assert_eq!(sx1276.rx_packet_count().unwrap(),0x8008);
    assert_eq!(sx1276.rx_packet_count().unwrap(),0xFFFF);
}

#[test]
fn test_modem_status() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x18,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x18,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x18,0x00],vec![0x00,0b00010101]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    assert_eq!(sx1276.rx_coding_rate().unwrap(),0b111);

    let modem_status_all_true = sx1276.modem_status().unwrap();
    assert_eq!(*modem_status_all_true.modem_clear(),true);
    assert_eq!(*modem_status_all_true.header_info_valid(),true);
    assert_eq!(*modem_status_all_true.rx_ongoing(),true);
    assert_eq!(*modem_status_all_true.signal_synchronized(),true);
    assert_eq!(*modem_status_all_true.signal_detected(),true);

    let modem_status_mixed = sx1276.modem_status().unwrap();
    assert_eq!(*modem_status_mixed.modem_clear(),true);
    assert_eq!(*modem_status_mixed.header_info_valid(),false);
    assert_eq!(*modem_status_mixed.rx_ongoing(),true);
    assert_eq!(*modem_status_mixed.signal_synchronized(),false);
    assert_eq!(*modem_status_mixed.signal_detected(),true);
}

#[test]
fn test_bandwidth() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9D,0x0F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9D,0x1F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x20],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x30],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x40],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x50],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x60],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x70],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x90],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x10]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x20]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x30]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x40]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x50]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x60]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x70]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x80]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x90]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_bandwidth(BandwidthValue::Bw7_8kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw10_4kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw15_6kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw20_8kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw31_25kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw41_7kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw62_5kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw125kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw250kHz).unwrap();
    sx1276.set_bandwidth(BandwidthValue::Bw500kHz).unwrap();

    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw7_8kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw10_4kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw15_6kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw20_8kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw31_25kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw41_7kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw62_5kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw125kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw250kHz);
    assert_eq!(sx1276.bandwidth().unwrap(),BandwidthValue::Bw500kHz);
}

#[test]
fn test_coding_rate() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9D,0xF3],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9D,0xF5],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x06],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x08],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x08]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x06]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x04]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x02]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_coding_rate(CodingRateValue::Cr4_5).unwrap();
    sx1276.set_coding_rate(CodingRateValue::Cr4_6).unwrap();
    sx1276.set_coding_rate(CodingRateValue::Cr4_7).unwrap();
    sx1276.set_coding_rate(CodingRateValue::Cr4_8).unwrap();

    assert_eq!(sx1276.coding_rate().unwrap(),CodingRateValue::Cr4_8);
    assert_eq!(sx1276.coding_rate().unwrap(),CodingRateValue::Cr4_7);
    assert_eq!(sx1276.coding_rate().unwrap(),CodingRateValue::Cr4_6);
    assert_eq!(sx1276.coding_rate().unwrap(),CodingRateValue::Cr4_5);
}

#[test]
fn test_implicit_header_mode() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9D,0xFE],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1D,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9D,0x01],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_implicit_header_mode(&false).unwrap();
    sx1276.set_implicit_header_mode(&true).unwrap();
}

#[test]
fn test_spreading_factor() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9E,0x6F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9E,0x7F],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0x80],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0x90],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0xA0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0xB0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0xC0],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xC0]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xB0]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xA0]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x90]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x80]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x70]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x60]),
    ]);

    for _n in 0..spi_transactions.len() {
        nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    }

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_spreading_factor(SpreadingFactorValue::SF6).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF7).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF8).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF9).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF10).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF11).unwrap();
    sx1276.set_spreading_factor(SpreadingFactorValue::SF12).unwrap();

    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF12);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF11);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF10);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF9);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF8);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF7);
    assert_eq!(sx1276.spreading_factor().unwrap(),SpreadingFactorValue::SF6);
}

#[test]
fn test_tx_continous_mode() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9E,0xF7],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0x08],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_tx_continuous_mode(&false).unwrap();
    sx1276.set_tx_continuous_mode(&true).unwrap();
}

#[test]
fn test_rx_payload_crc() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0xFF]),
        SpiTransaction::transfer(vec![0x9E,0xFB],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x1E,0x00],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x9E,0x04],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_rx_payload_crc(&false).unwrap();
    sx1276.set_rx_payload_crc(&true).unwrap();
}


#[test]
fn test_payload_and_max_payload_length() {
    let (init_reset_expectations,mut nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());
    nss_expectations.append(&mut get_sx1276_spi_chip_select_transactions());

    spi_transactions.append(&mut vec![
        SpiTransaction::transfer(vec![0xA2,0x50],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0xA3,0xFF],vec![0x00,0x00]),
    ]);

    let mut sx1276 = create_sx1276(&init_reset_expectations, &nss_expectations, &spi_transactions);

    sx1276.set_payload_length(&0x50).unwrap();
    sx1276.set_payload_max_length(&0xFF).unwrap();
}