use rcc::APB1;
use stm32f469xx::{WWDG, wwdg};




pub trait Watchdog {
    fn kick(&mut self);
}

pub trait WatchdogEnable {
    type Time;

    fn start<T>(&mut self, period: T) where T: Into<Self::Time>;
}


pub trait WwdgInterface {
    fn constrain(self, apb: &mut APB1) -> Wwdg;
}


impl WwdgInterface for WWDG {
    fn constrain(self, apb: &mut APB1) -> Wwdg {
        apb.enr().modify(|_, w| w.wwdgen().set_bit());
        apb.rstr().modify(|_, w| w.wwdgrst().set_bit());
        apb.rstr().modify(|_, w| w.wwdgrst().clear_bit());
        Wwdg {
            _0:(),
        }
    }
}

pub struct Wwdg {
    _0: ()
}


impl Wwdg {
    pub fn cr(&mut self) -> &wwdg::CR {
        unsafe { &(*WWDG::ptr()).cr }
    }
    
    pub fn cfr(&mut self) -> &wwdg::CFR {
        unsafe { &(*WWDG::ptr()).cfr }
    }

    pub fn sr(&mut self) -> &wwdg::SR {
        unsafe { &(*WWDG::ptr()).sr }
    }
}



impl WatchdogEnable for Wwdg {
    type Time = u8;
    fn start<T>(&mut self, period: T) where T: Into<Self::Time>  {
        let period:u8 = period.into();
        unsafe {
            self.cfr().modify(|r, w| w.bits((r.bits() & 0xFE7F) | ((period as u32)<<7)));
            self.cr().modify(|_, w| w.wdga().set_bit()); 
        }
    }

}

impl Watchdog for Wwdg {
    /// Warning with STM32 WWDG if the watchdog is kicked when time is greater
    /// than a window threshold (W) this will trigger a reset. Watchdog should be
    /// kicked at 0x3F < T < W.
    fn kick(&mut self) {
        // reset timer to maximum
        unsafe { self.cr().modify(|r, w| w.bits(0x7F)); }
    }
}
