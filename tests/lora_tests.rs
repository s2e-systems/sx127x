extern crate lorahatlib;
extern crate embedded_hal_mock;

use lorahatlib::SX1276;
use lorahatlib::{PaSelect};

//use embedded_hal_mock::MockError;
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

#[test]
fn test_create_lora() {
    let (reset_expectations,nss_expectations,spi_transactions) = get_sx1276_create_transactions();

    let reset_line = PinMock::new(&reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);

    // DIO0 not used for now but needs to be available for the driver
    let dio0_line =  PinMock::new(&[]);

    let spi = SpiMock::new(&spi_transactions);

    // No delays in the operation
    let delay = MockNoop::new();

    let _lora = SX1276::new(spi, reset_line, nss_line, dio0_line, delay).unwrap();
}

#[test]
fn test_create_wrong_lora_chip()
{
    // Get the original expectation and modify the SPI response for the wrong version
    let (reset_expectations,nss_expectations,mut spi_transactions) = get_sx1276_create_transactions();

    spi_transactions[0] = SpiTransaction::transfer(vec![0x42,0x00],vec![0x00,0x13]);

    let reset_line = PinMock::new(&reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);

    // DIO0 not used for now but needs to be available for the driver
    let dio0_line =  PinMock::new(&[]);

    let spi = SpiMock::new(&spi_transactions);

    // No delays in the operation
    let delay = MockNoop::new();

    assert_eq!(SX1276::new(spi, reset_line, nss_line, dio0_line, delay).err(), Some(lorahatlib::Error::InvalidVersion));
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
        SpiTransaction::transfer(vec![0x87,0x26],vec![0x00,0x00]),
        SpiTransaction::transfer(vec![0x88,0x6B],vec![0x00,0x00]),
    ]);

    let reset_line = PinMock::new(&init_reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);
    let dio0_line =  PinMock::new(&[]);
    let spi = SpiMock::new(&spi_transactions);
    let delay = MockNoop::new();

    let mut sx1276 = SX1276::new(spi, reset_line, nss_line, dio0_line, delay).unwrap();

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
        SpiTransaction::transfer(vec![0x07,0x00],vec![0x00,0x26]),
        SpiTransaction::transfer(vec![0x08,0x00],vec![0x00,0x6B]),
    ]);

    let reset_line = PinMock::new(&init_reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);
    let dio0_line =  PinMock::new(&[]);
    let spi = SpiMock::new(&spi_transactions);
    let delay = MockNoop::new();

    let mut sx1276 = SX1276::new(spi, reset_line, nss_line, dio0_line, delay).unwrap();

    assert_eq!(sx1276.frequency().unwrap(),868099967);
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

    let reset_line = PinMock::new(&init_reset_expectations);
    let nss_line =  PinMock::new(&nss_expectations);
    let dio0_line =  PinMock::new(&[]);
    let spi = SpiMock::new(&spi_transactions);
    let delay = MockNoop::new();

    let mut sx1276 = SX1276::new(spi, reset_line, nss_line, dio0_line, delay).unwrap();

    sx1276.set_pa_select(PaSelect::RfoPin).unwrap();
    sx1276.set_pa_select(PaSelect::PaBoost).unwrap();
}

