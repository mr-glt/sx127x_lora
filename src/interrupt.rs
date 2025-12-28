use crate::register::Register;

#[derive(Clone, Copy)]
pub enum Interrupt {
    RxDone = 0x0,
    TxDone = 0x40,
}

impl Interrupt {
    pub fn flag(self) -> u8 {
        match self {
            Interrupt::RxDone => 0x40,
            Interrupt::TxDone => 0x08
        }
    }

    pub fn mask(self) -> u8 {
        0xc0 // only supporting DIO0 for now
    }

    pub fn reg_addr(self) -> u8 {
        Register::RegDioMapping1.addr() // only supporting DIO0 for now
    }
}