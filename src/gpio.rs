use core::marker::PhantomData;

/// provides access to gpio pin
pub trait GpioInterface {
    type Parts;

    fn split(self) -> Self::Parts;
}

/// Line is pulled up
pub struct PullUp;
/// Line is pulled down
pub struct PullDown;
/// Line is floating
pub struct Floating;

/// Input mode
pub struct DigitalInput<PULL> {
    _pull: PhantomData<PULL>,
}

/// Analog input pin
pub struct AnalogInput;

/// Output mode
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Pin output state push-pull pull-up
pub struct PushPull;
/// Pin output state Open-drain
pub struct OpenDrain;

/// Alternate function 0 (type state)
pub struct AF0;
/// Alternate function 1 (type state)
pub struct AF1;
/// Alternate function 2 (type state)
pub struct AF2;
/// Alternate function 3 (type state)
pub struct AF3;
/// Alternate function 4 (type state)
pub struct AF4;
/// Alternate function 5 (type state)
pub struct AF5;
/// Alternate function 6 (type state)
pub struct AF6;
/// Alternate function 7 (type state)
pub struct AF7;
/// Alternate function 8 (type state)
pub struct AF8;
/// Alternate function 9 (type state)
pub struct AF9;
/// Alternate function 10 (type state)
pub struct AF10;
/// Alternate function 11 (type state)
pub struct AF11;
/// Alternate function 12 (type state)
pub struct AF12;
/// Alternate function 13 (type state)
pub struct AF13;
/// Alternate function 14 (type state)
pub struct AF14;
/// Alternate function 15 (type state)
pub struct AF15;

#[repr(u32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinMode {
    Input = 0,
    Output = 1,
    AlternateFunction = 2,
    Analog = 3,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinSpeed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

macro_rules! gpio_def {
    ($GPIO:ident, $gpio:ident, $gpio_ns:ident, $PX:ident, [
     $(($PXi:ident, $pxi:ident, $i:expr, $afr:ident),)+]) => {
        pub mod $gpio {
            use super::*;
            use stm32f469xx::{$GPIO, $gpio_ns};
            use core::marker::PhantomData;
            use hal::digital::OutputPin;
            #[cfg(feature = "unproven")]
            use hal::digital::InputPin;


            pub struct Parts {
                regs: Registers,
                $(
                /// Set to unconnected input as default.
                pub $pxi: $PXi<DigitalInput<Floating>>,
                )+
            }

            struct Registers {
                _0:()
            }

            impl Registers {
                pub(crate) fn regs(&mut self) -> &$gpio_ns::RegisterBlock {
                    unsafe { &(*$GPIO::ptr()) }
                }
            }

            impl GpioInterface for $GPIO {
                type Parts = Parts;

                fn split(self) -> Self::Parts {
                    Parts {
                        regs: Registers{_0:()},
                        $(
                            $pxi: $PXi{_mode: PhantomData},
                        )+
                    }
                }
            }

            /// Represents a pin where the GPIO registers have been locked.
            /// When locked the mode, otype, ospeed, pupd and afl registers for
            /// the pin may not be written to.
            pub struct LockedPin<PIN> {
                i: u8,
                pin: PIN,
            }

            impl<PIN> LockedPin<PIN> {
                /// Unlock the GPIO port
                pub fn unlock(self) -> PIN {
                    unsafe {
                        // Zero out lock bit
                        (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() & !(1<<16)));
                        // Set lock bit and change to pin locks
                        (*$GPIO::ptr()).lckr.modify(|r, w| w.bits((r.bits() & !(1<<self.i) ) | (1<<16)));
                        // Unset lock bit keeping change to pin locks
                        (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() & !(1<<16)));
                        // Set lock bit. Port is now unlocked
                        (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() | (1<<16)));
                    }
                    self.pin 
                }
            }
            
            impl<PIN> OutputPin for LockedPin<PIN> where PIN:OutputPin {
                /// Returns true if the output pin is high.
                fn is_high(&self) -> bool {
                    self.pin.is_high()
                }
                /// Returns true if the output pin is low.
                fn is_low(&self) -> bool {
                    self.pin.is_low()
                }
                /// Sets the pin output to high. If the pin is set high and low at the same time
                /// the high value will have precedence.
                fn set_high(&mut self) {
                    self.pin.set_high();
                }
                /// Sets the pin output to low. If the pin is set high and low at the same time
                /// the high value will have precedence.
                fn set_low(&mut self) {
                    self.pin.set_low();
                }
            }
            
            #[cfg(feature = "unproven")]
            impl<PIN> InputPin for LockedPin<PIN> where PIN:InputPin {
                fn is_high(&self) -> bool {
                    self.pin.is_high()
                }

                fn is_low(&self) -> bool {
                    self.pin.is_low()
                }
            }


            pub struct $PX<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> OutputPin for $PX<Output<MODE>> {
                /// Returns true if the output pin is high.
                fn is_high(&self) -> bool {
                    !self.is_low()
                }
                /// Returns true if the output pin is low.
                fn is_low(&self) -> bool {
                    unsafe {(*$GPIO::ptr()).odr.read().bits() & (1 << self.i) == 0 }
                }
                /// Sets the pin output to high. If the pin is set high and low at the same time
                /// the high value will have precedence.
                fn set_high(&mut self) {
                    unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1<<self.i)); }
                }
                /// Sets the pin output to low. If the pin is set high and low at the same time
                /// the high value will have precedence.
                fn set_low(&mut self) {
                    unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1<<(self.i + 16))); }
                }
            }

            #[cfg(feature = "unproven")]
            impl<MODE> InputPin for $PX<DigitalInput<MODE>> {
                fn is_high(&self) -> bool {
                    !self.is_low()
                }

                fn is_low(&self) -> bool {
                    unsafe { (*$GPIO::ptr()).idr.read().bits() & (1<<self.i) == 0 }
                }
            }

            impl<MODE> $PX<MODE> {
                /// Sets the port slew speed
                pub fn set_speed(&mut self, speed: PinSpeed) {
                    let offset = self.i * 2;
                    let speed = speed as u32;
                    unsafe {
                        (*$GPIO::ptr()).ospeedr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (speed<<offset)));
                    }
                }

            }

            $(
                pub struct $PXi<TYPE> {
                    _mode: PhantomData<TYPE>,
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    fn is_low(&self) -> bool {
                        unsafe {(*$GPIO::ptr()).odr.read().bits() & (1 << $i) == 0 }
                    }

                    fn set_high(&mut self) {
                        unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1<<$i)); }
                    }

                    fn set_low(&mut self) {
                        unsafe { (*$GPIO::ptr()).bsrr.write(|w| w.bits(1<<($i + 16))); }
                    }
                }

                #[cfg(feature = "unproven")]
                impl<MODE> InputPin for $PXi<DigitalInput<MODE>> {
                    fn is_high(&self) -> bool {
                        !self.is_low()
                    }

                    fn is_low(&self) -> bool {
                        unsafe { (*$GPIO::ptr()).idr.read().bits() & (1<<$i) == 0 }
                    }
                }

                impl $PXi<Output<OpenDrain>> {
                    /// Activates the internal pull-up resister for the opendrain output.
                    pub fn set_pullup(&mut self) {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).pupdr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (0b01 << offset)));
                        }
                    }

                    /// Activates the internal pull-down resister for the opendrain output.
                    pub fn set_pulldown(&mut self) {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).pupdr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (0b10 << offset)));
                        }
                    }
                }

                impl<TYPE> $PXi<TYPE> {
                    /// Lock the GPIO pin
                    pub fn lock(self) -> LockedPin<$PXi<TYPE>> {
                        unsafe {
                            // Zero out lock bit
                            (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() & !(1<<16)));
                            // Set lock bit and change to pin locks
                            (*$GPIO::ptr()).lckr.modify(|r, w| w.bits((r.bits() | (1<<$i) ) | (1<<16)));
                            // Unset lock bit keeping change to pin locks
                            (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() & !(1<<16)));
                            // Set lock bit. Port is now unlocked
                            (*$GPIO::ptr()).lckr.modify(|r, w| w.bits(r.bits() | (1<<16)));
                        }
                        LockedPin {
                            i: $i,
                            pin: self 
                        }
                    }

                    /// Downgrade type to more general type representing any pin on the bank.
                    pub fn downgrade(self) -> $PX<TYPE> {
                        $PX {
                            i: $i,
                            _mode: PhantomData,
                        }
                    }

                    /// Sets the port slew speed
                    pub fn set_speed(&mut self, speed: PinSpeed) {
                        let offset = $i * 2;
                        let speed = speed as u32;
                        unsafe {
                            (*$GPIO::ptr()).ospeedr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (speed<<offset)));
                        }
                    }

                    /// Turn pin into input pullup
                    pub fn into_input_pullup(self) -> $PXi<DigitalInput<PullUp>> {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).moder.modify(|r, w| w.bits(r.bits() & !(3<<offset)));
                            (*$GPIO::ptr()).pupdr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (0b01<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Turn pin into input pulldown
                    pub fn into_input_pulldown(self) -> $PXi<DigitalInput<PullDown>> {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).moder.modify(|r, w| w.bits(r.bits() & !(3<<offset)));
                            (*$GPIO::ptr()).pupdr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (0b10<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as a push-pull output
                    pub fn into_output_pushpull(self) -> $PXi<Output<PushPull>> {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).moder.modify(|r, w| w.bits((r.bits() & !(3<<offset))|(0b01<<offset)));
                            (*$GPIO::ptr()).otyper.modify(|r, w| w.bits(r.bits() & !(1<<$i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as an opendrain output
                    pub fn into_output_opendrain(self) -> $PXi<Output<OpenDrain>> {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).moder.modify(|r, w| w.bits((r.bits() & !(3<<offset))|(0b01<<offset)));
                            (*$GPIO::ptr()).otyper.modify(|r, w| w.bits((r.bits() & !(1<<$i)) | (0b1<<$i)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin as an analog input.
                    pub fn into_analog_pin(self) -> $PXi<AnalogInput> {
                        let offset = $i * 2;
                        unsafe {
                            (*$GPIO::ptr()).moder.modify(|r, w| w.bits((r.bits() & !(3<<offset))|(0b11<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af1(self) -> $PXi<AF1> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits(r.bits() &!(0xFF<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af2(self) -> $PXi<AF2> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (1<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af3(self) -> $PXi<AF3> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (2<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af4(self) -> $PXi<AF4> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (3<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af5(self) -> $PXi<AF5> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (4<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af6(self) -> $PXi<AF6> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (5<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af7(self) -> $PXi<AF7> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (6<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af8(self) -> $PXi<AF8> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (7<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af9(self) -> $PXi<AF9> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (8<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af10(self) -> $PXi<AF10> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (9<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af11(self) -> $PXi<AF11> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (10<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af12(self) -> $PXi<AF12> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (11<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af13(self) -> $PXi<AF13> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (12<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af14(self) -> $PXi<AF14> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (13<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_af15(self) -> $PXi<AF15> {
                        let offset = ($i%8) * 4;
                        unsafe {
                            (*$GPIO::ptr()).$afr.modify(|r, w| w.bits((r.bits() &!(0xFF<<offset)) | (14<<offset)));
                        }
                        $PXi { _mode: PhantomData }
                    }
                }
            )+
        }
    };
}

gpio_def!(
    GPIOA,
    gpioa,
    gpioa,
    PA,
    [
        (PA0, pa0, 0, afrl),
        (PA1, pa1, 1, afrl),
        (PA2, pa2, 2, afrl),
        (PA3, pa3, 3, afrl),
        (PA4, pa4, 4, afrl),
        (PA5, pa5, 5, afrl),
        (PA6, pa6, 6, afrl),
        (PA7, pa7, 7, afrl),
        (PA8, pa8, 8, afrh),
        (PA9, pa9, 9, afrh),
        (PA10, pa10, 10, afrh),
        (PA11, pa11, 11, afrh),
        (PA12, pa12, 12, afrh),
        (PA13, pa13, 13, afrh),
        (PA14, pa14, 14, afrh),
        (PA15, pa15, 15, afrh),
    ]
);

gpio_def!(
    GPIOB,
    gpiob,
    gpiob,
    PB,
    [
        (PB0, pb0, 0, afrl),
        (PB1, pb1, 1, afrl),
        (PB2, pb2, 2, afrl),
        (PB3, pb3, 3, afrl),
        (PB4, pb4, 4, afrl),
        (PB5, pb5, 5, afrl),
        (PB6, pb6, 6, afrl),
        (PB7, pb7, 7, afrl),
        (PB8, pb8, 8, afrh),
        (PB9, pb9, 9, afrh),
        (PB10, pb10, 10, afrh),
        (PB11, pb11, 11, afrh),
        (PB12, pb12, 12, afrh),
        (PB13, pb13, 13, afrh),
        (PB14, pb14, 14, afrh),
        (PB15, pb15, 15, afrh),
    ]
);

gpio_def!(
    GPIOC,
    gpioc,
    gpiok,
    PC,
    [
        (PC0, pc0, 0, afrl),
        (PC1, pc1, 1, afrl),
        (PC2, pc2, 2, afrl),
        (PC3, pc3, 3, afrl),
        (PC4, pc4, 4, afrl),
        (PC5, pc5, 5, afrl),
        (PC6, pc6, 6, afrl),
        (PC7, pc7, 7, afrl),
        (PC8, pc8, 8, afrh),
        (PC9, pc9, 9, afrh),
        (PC10, pc10, 10, afrh),
        (PC11, pc11, 11, afrh),
        (PC12, pc12, 12, afrh),
        (PC13, pc13, 13, afrh),
        (PC14, pc14, 14, afrh),
        (PC15, pc15, 15, afrh),
    ]
);

gpio_def!(
    GPIOD,
    gpiod,
    gpiok,
    PD,
    [
        (PD0, pd0, 0, afrl),
        (PD1, pd1, 1, afrl),
        (PD2, pd2, 2, afrl),
        (PD3, pd3, 3, afrl),
        (PD4, pd4, 4, afrl),
        (PD5, pd5, 5, afrl),
        (PD6, pd6, 6, afrl),
        (PD7, pd7, 7, afrl),
        (PD8, pd8, 8, afrh),
        (PD9, pd9, 9, afrh),
        (PD10, pd10, 10, afrh),
        (PD11, pd11, 11, afrh),
        (PD12, pd12, 12, afrh),
        (PD13, pd13, 13, afrh),
        (PD14, pd14, 14, afrh),
        (PD15, pd15, 15, afrh),
    ]
);

gpio_def!(
    GPIOE,
    gpioe,
    gpiok,
    PE,
    [
        (PE0, pe0, 0, afrl),
        (PE1, pe1, 1, afrl),
        (PE2, pe2, 2, afrl),
        (PE3, pe3, 3, afrl),
        (PE4, pe4, 4, afrl),
        (PE5, pe5, 5, afrl),
        (PE6, pe6, 6, afrl),
        (PE7, pe7, 7, afrl),
        (PE8, pe8, 8, afrh),
        (PE9, pe9, 9, afrh),
        (PE10, pe10, 10, afrh),
        (PE11, pe11, 11, afrh),
        (PE12, pe12, 12, afrh),
        (PE13, pe13, 13, afrh),
        (PE14, pe14, 14, afrh),
        (PE15, pe15, 15, afrh),
    ]
);

gpio_def!(
    GPIOF,
    gpiof,
    gpiok,
    PF,
    [
        (PF0, pf0, 0, afrl),
        (PF1, pf1, 1, afrl),
        (PF2, pf2, 2, afrl),
        (PF3, pf3, 3, afrl),
        (PF4, pf4, 4, afrl),
        (PF5, pf5, 5, afrl),
        (PF6, pf6, 6, afrl),
        (PF7, pf7, 7, afrl),
        (PF8, pf8, 8, afrh),
        (PF9, pf9, 9, afrh),
        (PF10, pf10, 10, afrh),
        (PF11, pf11, 11, afrh),
        (PF12, pf12, 12, afrh),
        (PF13, pf13, 13, afrh),
        (PF14, pf14, 14, afrh),
        (PF15, pf15, 15, afrh),
    ]
);

gpio_def!(
    GPIOG,
    gpiog,
    gpiok,
    PG,
    [
        (PG0, pg0, 0, afrl),
        (PG1, pg1, 1, afrl),
        (PG2, pg2, 2, afrl),
        (PG3, pg3, 3, afrl),
        (PG4, pg4, 4, afrl),
        (PG5, pg5, 5, afrl),
        (PG6, pg6, 6, afrl),
        (PG7, pg7, 7, afrl),
        (PG8, pg8, 8, afrh),
        (PG9, pg9, 9, afrh),
        (PG10, pg10, 10, afrh),
        (PG11, pg11, 11, afrh),
        (PG12, pg12, 12, afrh),
        (PG13, pg13, 13, afrh),
        (PG14, pg14, 14, afrh),
        (PG15, pg15, 15, afrh),
    ]
);

gpio_def!(
    GPIOH,
    gpioh,
    gpiok,
    PH,
    [
        (PH0, ph0, 0, afrl),
        (PH1, ph1, 1, afrl),
        (PH2, ph2, 2, afrl),
        (PH3, ph3, 3, afrl),
        (PH4, ph4, 4, afrl),
        (PH5, ph5, 5, afrl),
        (PH6, ph6, 6, afrl),
        (PH7, ph7, 7, afrl),
        (PH8, ph8, 8, afrh),
        (PH9, ph9, 9, afrh),
        (PH10, ph10, 10, afrh),
        (PH11, ph11, 11, afrh),
        (PH12, ph12, 12, afrh),
        (PH13, ph13, 13, afrh),
        (PH14, ph14, 14, afrh),
        (PH15, ph15, 15, afrh),
    ]
);

gpio_def!(
    GPIOI,
    gpioi,
    gpiok,
    PI,
    [
        (PI0, pi0, 0, afrl),
        (PI1, pi1, 1, afrl),
        (PI2, pi2, 2, afrl),
        (PI3, pi3, 3, afrl),
        (PI4, pi4, 4, afrl),
        (PI5, pi5, 5, afrl),
        (PI6, pi6, 6, afrl),
        (PI7, pi7, 7, afrl),
        (PI8, pi8, 8, afrh),
        (PI9, pi9, 9, afrh),
        (PI10, pi10, 10, afrh),
        (PI11, pi11, 11, afrh),
        (PI12, pi12, 12, afrh),
        (PI13, pi13, 13, afrh),
        (PI14, pi14, 14, afrh),
        (PI15, pi15, 15, afrh),
    ]
);

gpio_def!(
    GPIOJ,
    gpioj,
    gpiok,
    PJ,
    [
        (PJ0, pj0, 0, afrl),
        (PJ1, pj1, 1, afrl),
        (PJ2, pj2, 2, afrl),
        (PJ3, pj3, 3, afrl),
        (PJ4, pj4, 4, afrl),
        (PJ5, pj5, 5, afrl),
        (PJ6, pj6, 6, afrl),
        (PJ7, pj7, 7, afrl),
        (PJ8, pj8, 8, afrh),
        (PJ9, pj9, 9, afrh),
        (PJ10, pj10, 10, afrh),
        (PJ11, pj11, 11, afrh),
        (PJ12, pj12, 12, afrh),
        (PJ13, pj13, 13, afrh),
        (PJ14, pj14, 14, afrh),
        (PJ15, pj15, 15, afrh),
    ]
);

gpio_def!(
    GPIOK,
    gpiok,
    gpiok,
    PK,
    [
        (PK0, pk0, 0, afrl),
        (PK1, pk1, 1, afrl),
        (PK2, pk2, 2, afrl),
        (PK3, pk3, 3, afrl),
        (PK4, pk4, 4, afrl),
        (PK5, pk5, 5, afrl),
        (PK6, pk6, 6, afrl),
    ]
);
