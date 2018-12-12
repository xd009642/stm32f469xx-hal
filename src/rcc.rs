use stm32f469xx::{rcc, RCC};
use time::Hertz;

pub trait RccExt {
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb1: AHB1 { _0: () },
            ahb2: AHB2 { _0: () },
            ahb3: AHB3 { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            cfgr: CFGR {
               hse: None,
               has_lse: false,
               hclk: None,
               pclk1: None,
               pclk2: None,
               sysclk: None,
               rtcclk: None,
               mco1: None,
               mco2: None,
            },
        }
    }
}

pub struct Rcc {
    pub ahb1: AHB1,
    pub ahb2: AHB2,
    pub ahb3: AHB3,
    pub apb1: APB1,
    pub apb2: APB2,
    pub cfgr: CFGR,
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

/// HSI in Hertz
const HSI: u32 = 16_000_000;
/// LSI in Hertz
const LSI: u32 = 32_000;
/// Default LSE frequency in Hertz
const LSE: u32 = 32_768;


/// RCC Clock configuration
/// By setting desired frequencies for each clock the function freeze then
/// sets these frequencies as best as possible. (Can't take into account
/// external clock sources)
pub struct CFGR {
    /// The frequency of the external high speed oscillator
    hse: Option<u32>,
    has_lse: bool,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    rtcclk: Option<u32>,
    mco1: Option<u32>,
    mco2: Option<u32>,
}

impl CFGR {
    pub fn hse<F>(mut self, freq: F) -> Self 
    where
        F: Into<Hertz>, 
    {
        let freq = freq.into().0;
        // Frequency range of HSE
        assert!(freq>=4_000_000 && freq <= 26_000_000);
        self.hse = Some(freq);
        self
    }

    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    pub fn rtcclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.rtcclk = Some(freq.into().0);
        self
    }

    pub fn mco1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.mco1 = Some(freq.into().0);
        self
    }

    pub fn mco2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.mco2 = Some(freq.into().0);
        self
    }

    /// Freezes the clock configuration making it effective
    /// It will attempt to match the requested frequencies as closely as 
    /// possible. Actual frequencies will be returned in the `Clocks` struct
    pub fn freeze(self) -> Clocks {
        // Prioritise SYSCLK 

        let rcc = unsafe { &*RCC::ptr() };
        rcc.cfgr.write(|w| unsafe {
            w.i2ssrc()
                .clear_bits()
        });
    }
}

pub struct Clocks {}
