use core::marker::PhantomData;
use hal::serial;
use stm32f469xx::{USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8}
use gpio::*;

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
