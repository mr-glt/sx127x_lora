#![allow(dead_code)]

#[derive(Clone, Copy)]
pub enum Register {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0a,
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
    RegHighBWOptimize1 = 0x36,
    RegInvertiq = 0x33,
    RegDetectionThreshold = 0x37,
    RegSyncWord = 0x39,
    RegHighBWOptimize2 = 0x3a,
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
    TxDone = 0x08,
    PayloadCrcError = 0x20,
    RxDone = 0x40,
}

impl Register {
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

impl PaConfig {
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

impl IRQ {
    pub const fn addr(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy)]
pub enum FskDataModulationShaping {
    None = 1,
    GaussianBt1d0 = 2,
    GaussianBt0d5 = 10,
    GaussianBt0d3 = 11,
}

#[derive(Clone, Copy)]
pub enum FskRampUpRamDown {
    #[allow(non_camel_case_types)]
    _3d4ms = 0b000,
    _2ms = 0b0001,
    _1ms = 0b0010,
    _500us = 0b0011,
    _250us = 0b0100,
    _125us = 0b0101,
    _100us = 0b0110,
    _62us = 0b0111,
    _50us = 0b1000,
    _40us = 0b1001,
    _31us = 0b1010,
    _25us = 0b1011,
    _20us = 0b1100,
    _15us = 0b1101,
    _12us = 0b1110,
    _10us = 0b1111,
}
