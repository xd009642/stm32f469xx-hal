

#[derive(Clone, Copy, Debug)]
pub struct Hertz(pub u32);


impl From<u32> for Hertz {
    fn from(i: u32) -> Self {
        Hertz(i)
    }
}
