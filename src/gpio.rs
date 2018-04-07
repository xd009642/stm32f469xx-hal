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
     $(($PXi:ident, $pxi:ident, $i:expr, $AFR:ident),)+]) => {
        pub mod $gpio {
            use super::*;
            use stm32f469xx::{$GPIO, $gpio_ns};
            use core::marker::PhantomData;
            use hal::digital::OutputPin;

            pub struct Parts {
                regs: Registers,
                $(
                /// Set to unconnected input as default.
                $pxi: $PXi<DigitalInput<Floating>>,
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
                    /// Downgrade type to more general type representing any pin on the bank.
                    pub fn downgrade(self) -> $PX<TYPE> {
                        $PX {
                            i: $i,
                            _mode: PhantomData,
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

                    /// Sets the port slew speed
                    pub fn set_speed(&mut self, speed: PinSpeed) {
                        let offset = $i * 2;
                        let speed = speed as u32;
                        unsafe {
                            (*$GPIO::ptr()).ospeedr.modify(|r, w| w.bits((r.bits() & !(3<<offset)) | (speed<<offset)));
                        }
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
        (PA0, pa0, 0, AFRL),
        (PA1, pa1, 1, AFRL),
        (PA2, pa2, 2, AFRL),
        (PA3, pa3, 3, AFRL),
        (PA4, pa4, 4, AFRL),
        (PA5, pa5, 5, AFRL),
        (PA6, pa6, 6, AFRL),
        (PA7, pa7, 7, AFRL),
        (PA8, pa8, 8, AFRH),
        (PA9, pa9, 9, AFRH),
        (PA10, pa10, 10, AFRH),
        (PA11, pa11, 11, AFRH),
        (PA12, pa12, 12, AFRH),
        (PA13, pa13, 13, AFRH),
        (PA14, pa14, 14, AFRH),
        (PA15, pa15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOB,
    gpiob,
    gpiob,
    PB,
    [
        (PB0, pb0, 0, AFRL),
        (PB1, pb1, 1, AFRL),
        (PB2, pb2, 2, AFRL),
        (PB3, pb3, 3, AFRL),
        (PB4, pb4, 4, AFRL),
        (PB5, pb5, 5, AFRL),
        (PB6, pb6, 6, AFRL),
        (PB7, pb7, 7, AFRL),
        (PB8, pb8, 8, AFRH),
        (PB9, pb9, 9, AFRH),
        (PB10, pb10, 10, AFRH),
        (PB11, pb11, 11, AFRH),
        (PB12, pb12, 12, AFRH),
        (PB13, pb13, 13, AFRH),
        (PB14, pb14, 14, AFRH),
        (PB15, pb15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOC,
    gpioc,
    gpiok,
    PC,
    [
        (PC0, pc0, 0, AFRL),
        (PC1, pc1, 1, AFRL),
        (PC2, pc2, 2, AFRL),
        (PC3, pc3, 3, AFRL),
        (PC4, pc4, 4, AFRL),
        (PC5, pc5, 5, AFRL),
        (PC6, pc6, 6, AFRL),
        (PC7, pc7, 7, AFRL),
        (PC8, pc8, 8, AFRH),
        (PC9, pc9, 9, AFRH),
        (PC10, pc10, 10, AFRH),
        (PC11, pc11, 11, AFRH),
        (PC12, pc12, 12, AFRH),
        (PC13, pc13, 13, AFRH),
        (PC14, pc14, 14, AFRH),
        (PC15, pc15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOD,
    gpiod,
    gpiok,
    PD,
    [
        (PD0, pd0, 0, AFRL),
        (PD1, pd1, 1, AFRL),
        (PD2, pd2, 2, AFRL),
        (PD3, pd3, 3, AFRL),
        (PD4, pd4, 4, AFRL),
        (PD5, pd5, 5, AFRL),
        (PD6, pd6, 6, AFRL),
        (PD7, pd7, 7, AFRL),
        (PD8, pd8, 8, AFRH),
        (PD9, pd9, 9, AFRH),
        (PD10, pd10, 10, AFRH),
        (PD11, pd11, 11, AFRH),
        (PD12, pd12, 12, AFRH),
        (PD13, pd13, 13, AFRH),
        (PD14, pd14, 14, AFRH),
        (PD15, pd15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOE,
    gpioe,
    gpiok,
    PE,
    [
        (PE0, pe0, 0, AFRL),
        (PE1, pe1, 1, AFRL),
        (PE2, pe2, 2, AFRL),
        (PE3, pe3, 3, AFRL),
        (PE4, pe4, 4, AFRL),
        (PE5, pe5, 5, AFRL),
        (PE6, pe6, 6, AFRL),
        (PE7, pe7, 7, AFRL),
        (PE8, pe8, 8, AFRH),
        (PE9, pe9, 9, AFRH),
        (PE10, pe10, 10, AFRH),
        (PE11, pe11, 11, AFRH),
        (PE12, pe12, 12, AFRH),
        (PE13, pe13, 13, AFRH),
        (PE14, pe14, 14, AFRH),
        (PE15, pe15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOF,
    gpiof,
    gpiok,
    PF,
    [
        (PF0, pf0, 0, AFRL),
        (PF1, pf1, 1, AFRL),
        (PF2, pf2, 2, AFRL),
        (PF3, pf3, 3, AFRL),
        (PF4, pf4, 4, AFRL),
        (PF5, pf5, 5, AFRL),
        (PF6, pf6, 6, AFRL),
        (PF7, pf7, 7, AFRL),
        (PF8, pf8, 8, AFRH),
        (PF9, pf9, 9, AFRH),
        (PF10, pf10, 10, AFRH),
        (PF11, pf11, 11, AFRH),
        (PF12, pf12, 12, AFRH),
        (PF13, pf13, 13, AFRH),
        (PF14, pf14, 14, AFRH),
        (PF15, pf15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOG,
    gpiog,
    gpiok,
    PG,
    [
        (PG0, pg0, 0, AFRL),
        (PG1, pg1, 1, AFRL),
        (PG2, pg2, 2, AFRL),
        (PG3, pg3, 3, AFRL),
        (PG4, pg4, 4, AFRL),
        (PG5, pg5, 5, AFRL),
        (PG6, pg6, 6, AFRL),
        (PG7, pg7, 7, AFRL),
        (PG8, pg8, 8, AFRH),
        (PG9, pg9, 9, AFRH),
        (PG10, pg10, 10, AFRH),
        (PG11, pg11, 11, AFRH),
        (PG12, pg12, 12, AFRH),
        (PG13, pg13, 13, AFRH),
        (PG14, pg14, 14, AFRH),
        (PG15, pg15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOH,
    gpioh,
    gpiok,
    PH,
    [
        (PH0, ph0, 0, AFRL),
        (PH1, ph1, 1, AFRL),
        (PH2, ph2, 2, AFRL),
        (PH3, ph3, 3, AFRL),
        (PH4, ph4, 4, AFRL),
        (PH5, ph5, 5, AFRL),
        (PH6, ph6, 6, AFRL),
        (PH7, ph7, 7, AFRL),
        (PH8, ph8, 8, AFRH),
        (PH9, ph9, 9, AFRH),
        (PH10, ph10, 10, AFRH),
        (PH11, ph11, 11, AFRH),
        (PH12, ph12, 12, AFRH),
        (PH13, ph13, 13, AFRH),
        (PH14, ph14, 14, AFRH),
        (PH15, ph15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOI,
    gpioi,
    gpiok,
    PI,
    [
        (PI0, pi0, 0, AFRL),
        (PI1, pi1, 1, AFRL),
        (PI2, pi2, 2, AFRL),
        (PI3, pi3, 3, AFRL),
        (PI4, pi4, 4, AFRL),
        (PI5, pi5, 5, AFRL),
        (PI6, pi6, 6, AFRL),
        (PI7, pi7, 7, AFRL),
        (PI8, pi8, 8, AFRH),
        (PI9, pi9, 9, AFRH),
        (PI10, pi10, 10, AFRH),
        (PI11, pi11, 11, AFRH),
        (PI12, pi12, 12, AFRH),
        (PI13, pi13, 13, AFRH),
        (PI14, pi14, 14, AFRH),
        (PI15, pi15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOJ,
    gpioj,
    gpiok,
    PJ,
    [
        (PJ0, pj0, 0, AFRL),
        (PJ1, pj1, 1, AFRL),
        (PJ2, pj2, 2, AFRL),
        (PJ3, pj3, 3, AFRL),
        (PJ4, pj4, 4, AFRL),
        (PJ5, pj5, 5, AFRL),
        (PJ6, pj6, 6, AFRL),
        (PJ7, pj7, 7, AFRL),
        (PJ8, pj8, 8, AFRH),
        (PJ9, pj9, 9, AFRH),
        (PJ10, pj10, 10, AFRH),
        (PJ11, pj11, 11, AFRH),
        (PJ12, pj12, 12, AFRH),
        (PJ13, pj13, 13, AFRH),
        (PJ14, pj14, 14, AFRH),
        (PJ15, pj15, 15, AFRH),
    ]
);

gpio_def!(
    GPIOK,
    gpiok,
    gpiok,
    PK,
    [
        (PK0, pk0, 0, AFRL),
        (PK1, pk1, 1, AFRL),
        (PK2, pk2, 2, AFRL),
        (PK3, pk3, 3, AFRL),
        (PK4, pk4, 4, AFRL),
        (PK5, pk5, 5, AFRL),
        (PK6, pk6, 6, AFRL),
    ]
);
