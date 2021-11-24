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

impl AHB2 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB2ENR {
        unsafe { &(*RCC::ptr()).ahb2enr }
    }
    
    pub(crate) fn rstr(&mut self) -> &rcc::AHB2RSTR {
        unsafe { &(*RCC::ptr()).ahb2rstr }
    }
}

pub struct AHB3 {
    _0: (),
}

impl AHB3 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        unsafe { &(*RCC::ptr()).ahb3enr }
    }
    
    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn enr(&mut self) -> &rcc::APB1ENR {
        unsafe { &(*RCC::ptr()).apb1enr }
    }
    
    pub(crate) fn rstr(&mut self) -> &rcc::APB1RSTR {
        unsafe { &(*RCC::ptr()).apb1rstr }
    }
}

pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        unsafe { &(*RCC::ptr()).apb2enr }
    }
    
    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}
