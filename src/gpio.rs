use stm32f469xx::*;
use core::marker::PhantomData;

/// Line is pulled up
struct PullUp;
/// Line is pulled down
struct PullDown;
/// Line is floating
struct Floating;
/// Line has both pull up and pull down resistors
struct PullSideways;

struct Input<MODE, PULL> {
    _mode: PhantomData<MODE>,
    _pull: PhantomData<PULL>,
}

/// Pin input state digital
pub struct DigitalInput;
/// Pin input state analog
pub struct AnalogInput;

struct Output<MODE, PULL> {
    _mode: PhantomData<MODE>,
    _pull: PhantomData<PULL>,
}

/// Pin output state push-pull pull-up
pub struct PushPullOut;
/// Pin output state Open-drain
pub struct OpenDrainOut;

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

