use core::marker::PhantomData;
use hal::serial;
use nb;
use stm32f469xx::{USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8}
use gpio::*;

/// Error enum for USART messages
pub enum Error {
    /// Overrun flag is set when data is ready to be transferred into the
    /// data register but the current data there hasn't been read yet.
    Overrun,
    /// Flagged when noise is detected on a received frame
    Noise,
    /// Set when de-synchronisation, excessive noise or a break character is
    /// detected
    Framing,
    /// Set when a parity error is detected on the data.
    Parity,
}

/// Tx pin trait - not to be implemented
pub unsafe trait TxPin<SERIAL> {}
/// Rx pin trait - not to be implemented
pub unsafe trait RxPin<SERIAL> {}

unsafe impl TxPin<USART1> for PA9<AF7> {}
unsafe impl TxPin<USART1> for PB6<AF7> {}
unsafe impl TxPin<USART2> for PA2<AF7> {}
unsafe impl TxPin<USART2> for PD5<AF7> {}
unsafe impl TxPin<USART3> for PB10<AF7> {}
unsafe impl TxPin<USART3> for PC10<AF7> {}
unsafe impl TxPin<USART3> for PD8<AF7> {}
unsafe impl TxPin<USART6> for PC6<AF8>{}
unsafe impl TxPin<USART6> for PG14<AF8>{}

unsafe impl RxPin<USART1> for PA10<AF7> {}
unsafe impl RxPin<USART1> for PB11<AF7> {}
unsafe impl RxPin<USART2> for PA3<AF7> {}
unsafe impl RxPin<USART2> for PD6<AF7> {}
unsafe impl RxPin<USART3> for PB13<AF7> {}
unsafe impl RxPin<USART3> for PC11<AF7> {}
unsafe impl RxPin<USART3> for PD9<AF7> {}
unsafe impl RxPin<USART6> for PC7<AF8> {}
unsafe impl RxPin<USART6> for PG9<AF8> {}

unsafe impl TxPin<UART4> for PA0<AF8> {}
unsafe impl TxPin<UART4> for PC10<AF8> {}
unsafe impl TxPin<UART5> for PC12<AF8> {}
unsafe impl TxPin<UART7> for PE8<AF8> {}
unsafe impl TxPin<UART7> for PF7<AF8> {}
unsafe impl TxPin<UART8> for PE1<AF8> {}

unsafe impl RxPin<UART4> for PA1<AF8> {}
unsafe impl RxPin<UART4> for PC11<AF8> {}
unsafe impl RxPin<UART5> for PD2<AF8> {}
unsafe impl RxPin<UART7> for PE7<AF8> {}
unsafe impl RxPin<UART7> for PF6<AF8> {}
unsafe impl RxPin<UART8> for PE0<AF8> {}

macro_rules! uart {
    ($UART: ident) => {
        impl serial::Read<u8> for RxPin<$UART> {
            type Error = Error;
            fn read(&mut self) -> nb::Result<u8, Error> {
                unimplemented!(); 
            }
        }

        impl serial::Write<u8> for TxPin<$UART> {
            type Error = Error;

            fn write(&mut self, word: u8) -> nb::Result<(), Error> {
                unimplemented!();
            }

            fn flush(&mut self) -> nb::Result<(), Error> {
                unimplemented!();
            }
        }
    }
}

macro_rules! usart {
    ($USART: ident) => {
        impl serial::Read<u8> for RxPin<$USART> {
            type Error = Error;
            fn read(&mut self) -> nb::Result<u8, Error> {
                unimplemented!(); 
            }
        }

        impl serial::Write<u8> for TxPin<$USART> {
            type Error = Error;

            fn write(&mut self, word: u8) -> nb::Result<(), Error> {
                unimplemented!();
            }

            fn flush(&mut self) -> nb::Result<(), Error> {
                unimplemented!();
            }
        }
    }
}

uart!(UART4);
uart!(UART5);
uart!(UART7);
uart!(UART8);

usart!(USART1);
usart!(USART2);
usart!(USART3);
usart!(USART6);
