#![allow(dead_code)]

#[derive(Clone, Copy)]
pub enum Register {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegOcp = 0x0b,
    RegLna = 0x0c,
    RegFifoAddrPtr = 0x0d,
    RegFifoTxBaseAddr = 0x0e,
    RegFifoRxBaseAddr = 0x0f,
    RegFifoRxCurrentAddr = 0x10,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegPktSnrValue = 0x19,
    RegPktRssiValue = 0x1a,
    RegModemConfig1 = 0x1d,
    RegModemConfig2 = 0x1e,
    RegPreambleMsb = 0x20,
    RegPreambleLsb = 0x21,
    RegPayloadLength = 0x22,
    RegModemConfig3 = 0x26,
    RegFreqErrorMsb = 0x28,
    RegFreqErrorMid = 0x29,
    RegFreqErrorLsb = 0x2a,
    RegRssiWideband = 0x2c,
    RegDetectionOptimize = 0x31,
    RegInvertiq = 0x33,
    RegDetectionThreshold = 0x37,
    RegSyncWord = 0x39,
    RegInvertiq2 = 0x3b,
    RegDioMapping1 = 0x40,
    RegVersion = 0x42,
    RegPaDac = 0x4d,
}
#[derive(Clone, Copy)]
pub enum PaConfig {
    PaBoost = 0x80,
    PaOutputRfoPin = 0,
}

#[derive(Clone, Copy)]
pub enum IRQ {
    IrqTxDoneMask = 0x08,
    IrqPayloadCrcErrorMask = 0x20,
    IrqRxDoneMask = 0x40,
}

impl Register {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

impl PaConfig {
    pub fn addr(self) -> u8 {
        self as u8
    }
}

impl IRQ {
    pub fn addr(self) -> u8 {
        self as u8
    }
}
