use encoder::Encoder;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::delay::{self};
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::prelude::*;
use l293x::L293x;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    println!("Configuring output channel");

    let peripherals = Peripherals::take()?;

    // Motors
    // create driver
    let driver1 = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &config::TimerConfig::new().frequency(5.kHz().into()),
    )?;
    let driver2 = LedcTimerDriver::new(
        peripherals.ledc.timer1,
        &config::TimerConfig::new().frequency(5.kHz().into()),
    )?;
    let driver3 = LedcTimerDriver::new(
        peripherals.ledc.timer2,
        &config::TimerConfig::new().frequency(5.kHz().into()),
    )?;
    let driver4 = LedcTimerDriver::new(
        peripherals.ledc.timer3,
        &config::TimerConfig::new().frequency(5.kHz().into()),
    )?;

    let mut m1_forward =
        LedcDriver::new(peripherals.ledc.channel0, driver1, peripherals.pins.gpio40)?;

    let mut m1_reverse =
        LedcDriver::new(peripherals.ledc.channel1, driver2, peripherals.pins.gpio41)?;

    let mut m2_forward =
        LedcDriver::new(peripherals.ledc.channel2, driver3, peripherals.pins.gpio4)?;
    let mut m2_reverse =
        LedcDriver::new(peripherals.ledc.channel3, driver4, peripherals.pins.gpio5)?;
    let m1_enable = peripherals.pins.gpio12;
    let m2_enable = peripherals.pins.gpio42;

    println!("Starting duty-cycle loop");

    let mut motors = L293x::new(
        m1_forward, m1_reverse, m2_forward, m2_reverse, m1_enable, m2_enable,
    );

    // Encoder
    let mut pin_a = peripherals.pins.gpio8;
    let mut pin_b = peripherals.pins.gpio9;
    println!("setup encoder");
    let encoder = Encoder::new(peripherals.pcnt0, &mut pin_a, &mut pin_b)?;

    let mut last_value = 0i32;
    motors.set_y2_duty_cycle_fully_off()?;
    motors.set_y4_duty_cycle_fully_off()?;
    motors.set_y1_duty_cycle_fully_on()?;
    motors.set_y3_duty_cycle_percent(30)?;

    loop {
        let value = encoder.get_value()?;
        if value != last_value {
            println!("value: {value}");
            last_value = value;
        }
        FreeRtos::delay_ms(100u32);
    }

    /*
    loop {
        println!("inside the loop");

        delay::Ets::delay_ms(100);
        // Full speed forward
        motors.set_y2_duty_cycle_fully_off()?;
        motors.set_y4_duty_cycle_fully_off()?;
        motors.set_y1_duty_cycle_fully_on()?;
        motors.set_y3_duty_cycle_fully_on()?;
        delay::Ets::delay_ms(500);

        motors.set_y1_duty_cycle_fully_off()?;
        motors.set_y3_duty_cycle_fully_off()?;
        motors.set_y2_duty_cycle_fully_on()?;
        motors.set_y4_duty_cycle_fully_on()?;
    }
    */
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
fn main() {
    use esp_idf_hal::delay::FreeRtos;
    println!("pcnt peripheral not supported on this device!");
    loop {
        FreeRtos::delay_ms(100u32);
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
// esp-idf encoder implementation using v4 pcnt api
mod encoder {
    use std::cmp::min;
    use std::sync::atomic::AtomicI32;
    use std::sync::atomic::Ordering;
    use std::sync::Arc;

    use esp_idf_hal::gpio::AnyInputPin;
    use esp_idf_hal::gpio::InputPin;
    use esp_idf_hal::pcnt::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_sys::EspError;

    const LOW_LIMIT: i16 = -100;
    const HIGH_LIMIT: i16 = 100;

    pub struct Encoder<'d> {
        unit: PcntDriver<'d>,
        approx_value: Arc<AtomicI32>,
    }

    impl<'d> Encoder<'d> {
        pub fn new<PCNT: Pcnt>(
            pcnt: impl Peripheral<P = PCNT> + 'd,
            pin_a: impl Peripheral<P = impl InputPin> + 'd,
            pin_b: impl Peripheral<P = impl InputPin> + 'd,
        ) -> Result<Self, EspError> {
            let mut unit = PcntDriver::new(
                pcnt,
                Some(pin_a),
                Some(pin_b),
                Option::<AnyInputPin>::None,
                Option::<AnyInputPin>::None,
            )?;
            // The Idea here is that each "channel" is a pair of pins that are connected to the
            // encoder. Since CLK and DT will determine which rising/falling edge determines the
            // count, we will have a channel each for forward and backward movement.
            unit.channel_config(
                PcntChannel::Channel0,
                PinIndex::Pin0,
                PinIndex::Pin1,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Decrement,
                    neg_mode: PcntCountMode::Increment,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;
            unit.channel_config(
                PcntChannel::Channel1,
                PinIndex::Pin1,
                PinIndex::Pin0,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Increment,
                    neg_mode: PcntCountMode::Decrement,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;

            unit.set_filter_value(min(10 * 80, 1023))?;
            unit.filter_enable()?;

            let approx_value = Arc::new(AtomicI32::new(0));
            // unsafe interrupt code to catch the upper and lower limits from the encoder
            // and track the overflow in `value: Arc<AtomicI32>` - I plan to use this for
            // a wheeled robot's odomerty
            unsafe {
                let approx_value = approx_value.clone();
                unit.subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                    }
                })?;
            }
            unit.event_enable(PcntEvent::HighLimit)?;
            unit.event_enable(PcntEvent::LowLimit)?;
            unit.counter_pause()?;
            unit.counter_clear()?;
            unit.counter_resume()?;

            Ok(Self { unit, approx_value })
        }

        pub fn get_value(&self) -> Result<i32, EspError> {
            let value =
                self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i32;
            Ok(value)
        }
    }
}
