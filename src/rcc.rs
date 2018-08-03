use stm32f469xx::{rcc, RCC};


pub trait RccExt {
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1{ _0: ()},
            ahb2: AHB2{ _0: ()},
            ahb3: AHB3{ _0: ()},
            apb1: APB1{ _0: ()},
            apb2: APB2{ _0: ()},
        }
    }
}


pub struct Rcc {
    pub ahb1: AHB1,
    pub ahb2: AHB2,
    pub ahb3: AHB3,
    pub apb1: APB1,
    pub apb2: APB2,

}

pub struct AHB1 {
    _0: (),
}

impl AHB1 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB1ENR {
        unsafe { &(*RCC::ptr()).ahb1enr }
    }
    
    
    pub(crate) fn rstr(&mut self) -> &rcc::AHB1RSTR {
        unsafe { &(*RCC::ptr()).ahb1rstr }
    }
}

pub struct AHB2 {
    _0: (),
}

pub struct AHB3 {
    _0: (),
}

pub struct APB1 {
    _0: (),
}

pub struct APB2 {
    _0: (),
}
