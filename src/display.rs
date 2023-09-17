use cortex_m::delay::Delay;
use embedded_hal::blocking::{delay::DelayMs, i2c};

pub(crate) struct Display<T>
where
    T: i2c::Write,
{
    i2c: T,
    address: u8,
}

impl<T> Display<T>
where
    T: i2c::Write,
{
    pub fn new(address: u8, i2c: T) -> Display<T> {
        Display { i2c, address }
    }

    // fn init(&mut self){
    //     self.i2c.write(self.address, )
    // }

    fn write_word(&mut self, data: u8) -> Result<(), <T as i2c::Write>::Error> {
        let mut temp = data;
        self.i2c.write(self.address, &[temp | 0x08])
    }

    fn send_command(
        &mut self,
        command: u8,
        delay: &mut Delay,
    ) -> Result<(), <T as i2c::Write>::Error> {
        let mut buf = command & 0xF0;
        buf |= 0x04;
        self.write_word(buf)?;
        delay.delay_ms(5);
        buf &= 0xFB;
        self.write_word(buf)?;

        buf = (command & 0x0F) << 4;
        buf |= 0x04;
        self.write_word(buf)?;
        delay.delay_ms(5);
        buf &= 0xFB;
        self.write_word(buf)
    }

    fn send_data(&mut self, data: u8, delay: &mut Delay) -> Result<(), <T as i2c::Write>::Error> {
        let mut buf = data & 0xF0;
        buf |= 0x05;
        self.write_word(buf)?;
        delay.delay_ms(5);
        buf &= 0xFB;
        self.write_word(buf)?;

        buf = (data & 0x0F) << 4;
        buf |= 0x05;
        self.write_word(buf)?;
        delay.delay_ms(5);
        buf &= 0xFB;
        self.write_word(buf)
    }

    pub fn clear(&mut self, delay: &mut Delay) -> Result<(), <T as i2c::Write>::Error> {
        self.send_command(0x01, delay)
    }

    pub fn openlight(&mut self) -> Result<(), <T as i2c::Write>::Error> {
        self.i2c.write(self.address, &[0x08])
    }

    pub fn write(
        &mut self,
        mut x: u8,
        mut y: u8,
        word: &str,
        delay: &mut Delay,
    ) -> Result<(), <T as i2c::Write>::Error> {
        if x > 15 {
            x = 15;
        }

        if y > 1 {
            y = 1;
        }
        let addr = 0x80 + (0x40 * y) + x;
        let mut last = self.send_command(addr, delay);

        for letter in word.chars() {
            last = self.send_data(letter as u8, delay);
        }
        last
    }

    pub fn init(&mut self, delay: &mut Delay) -> Result<(), <T as i2c::Write>::Error> {
        self.send_command(0x33, delay)?;
        delay.delay_ms(5);
        self.send_command(0x32, delay)?;
        delay.delay_ms(5);
        self.send_command(0x28, delay)?;
        delay.delay_ms(5);
        self.send_command(0x0C, delay)?;
        delay.delay_ms(5);
        self.send_command(0x01, delay)?;
        self.i2c.write(self.address, &[0x08])
    }
}
