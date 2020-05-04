//! [A0 EXTI0^v input with internall pullup] Switch d (pull down once in each main period if closed)
//! [A1 EXTI1^v input with internall pullup] Switch c (pull down once in each main period if closed)
//! [A2 EXTI2^v input with internall pullup] Switch a (pull down once in each main period if closed)
//! [A3 EXTI3^v input with internall pullup] Switch b (pull down once in each main period if closed)
//!
//! [A4 EXTI4^v input with internall pullup] Motion alarm on A4 (pull down)
//! [A5 EXTI5^v input with internall pullup] Open alarm on A5 (pull down)
//!
//! A6, A7 not used, connected to the ground
//!
//! Optional piezzo speaker on A8
//!
//! Solid state relay connected to A9 drives the ssr_lamp_a
//! Solid state relay connected to A10 drives the ssr_lamp_b
//!
//! CAN (RX, TX) on A11, A12
//!
//! [A15 EXTI15^v input with internall pullup] Read the NEC IR remote commands
//!
//! Photoresistor on B0 (ADC8)
//!
//! B1 not connected
//! B3 not used, connected to the ground
//!
//! DS18B20 1-wire temperature sensors connected to B4 GPIO
//! JTAG is removed from B3, B4 to make it work
//!
//! B5 not used, connected to the ground
//!
//! [EXTI6..9??] Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
//!
//! PT8211 DAC (BCK, DIN, WS) on B10, B11, B12
//!
//! RGB led on PB13, PB14, PB15 as push pull output
//!
//! C13 on board LED
//!
//! C14, C15 used on the bluepill board for 32768Hz xtal
//!

//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

//#[cfg(feature = "semihosting-debug")]
use cortex_m_semihosting::hprintln;

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;

use embedded_hal::{
    digital::v2::InputPin,
    digital::v2::OutputPin,
    watchdog::{Watchdog, WatchdogEnable},
};

use stm32f1xx_hal::{
    adc,
    can::*,
    delay::Delay,
    gpio::{ExtiPin, *},
    prelude::*,
    rtc,
    serial::{Config, Serial},
    timer::Timer,
    watchdog::IndependentWatchdog,
};

use onewire::{ds18x20::*, temperature::Temperature, *};

use room_pill::{
    ac_sense::AcSense,
    ac_switch::*,
    dac::*,
    ir,
    ir::NecReceiver,
    ir_remote::*,
    messenger::{Messenger, ID_MOVEMENT, ID_OPEN, ID_TEMPERATURE},
    rgb::{Colors, RgbLed},
    timing::{SysTicks, Ticker, TimeExt},
};

#[entry]
fn main() -> ! {
    door_unit_main();
}

fn door_unit_main() -> ! {
    let device = stm32f1xx_hal::pac::Peripherals::take().unwrap();
    let mut watchdog = IndependentWatchdog::new(device.IWDG);
    watchdog.start(stm32f1xx_hal::time::U32Ext::ms(2_000u32));

    //10 period at 50Hz, 12 period at 60Hz
    let ac_test_period = TimeExt::us(200_000);
    let one_sec = TimeExt::us(1_000_000);
    let sound_period = TimeExt::us(1_000_000u32 / 8_000u32); //room_pill::timing::Duration<u32, SysTicks> = 3_000u32.into();

    let mut flash = device.FLASH.constrain();

    // flash.acr.prftbe().enabled();//?? Configure Flash prefetch - Prefetch buffer is not available on value line devices
    // scb().set_priority_grouping(NVIC_PRIORITYGROUP_4);

    let mut rcc = device.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .adcclk(9.mhz()) //ADC clock: PCLK2 / 8. User specified value is be approximated using supported prescaler values 2/4/6/8.
        .freeze(&mut flash.acr);
    watchdog.feed();

    let mut core = cortex_m::Peripherals::take().unwrap();
    #[cfg(feature = "itm-debug")]
    let stim = &mut core.ITM.stim[0];

    let mut pwr = device.PWR;
    let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
    // real time clock
    let _rtc = rtc::Rtc::rtc(device.RTC, &mut backup_domain);

    // A/D converter
    let mut adc1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
    watchdog.feed();

    let mut afio = device.AFIO.constrain(&mut rcc.apb2);

    // Configure pins:
    let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

    // Disables the JTAG to free up pb3, pb4 and pa15 for normal use
    let (pa15, _pb3_itm_swo, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
    watchdog.feed();

    //--------- Lights control releated
    // Switces on A0, A1, A2, A3 (pull down once in each main period if closed)
    let mut switch_d = AcSwitch::new(gpioa.pa0.into_pull_up_input(&mut gpioa.crl), ac_test_period);
    let mut switch_c = AcSwitch::new(gpioa.pa1.into_pull_up_input(&mut gpioa.crl), ac_test_period);
    let mut switch_a = AcSwitch::new(gpioa.pa2.into_pull_up_input(&mut gpioa.crl), ac_test_period);
    let mut switch_b = AcSwitch::new(gpioa.pa3.into_pull_up_input(&mut gpioa.crl), ac_test_period);
    // Solid state relay connected to A9 drives the lamp_b
    let mut ssr_lamp_b = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
    ssr_lamp_b.set_low().unwrap();

    // Solid state relay connected to A10 drives the lamp_a
    let mut ssr_lamp_a = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
    ssr_lamp_a.set_low().unwrap();

    // Photoresistor on B0 (ADC8)
    let mut adc8_photo_resistor = gpiob.pb0.into_analog(&mut gpiob.crl);

    // -------- Alarm related
    // Motion alarm on A4 (pull down)
    let mut motion_alarm = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);
    // Open alarm on A5 (pull down)
    let mut open_alarm = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);

    // CAN (RX, TX) on A11, A12
    let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

    // USB is needed here because it can not be used at the same time as CAN since they share memory:
    let mut messenger = Messenger::new(Can::can1(
        device.CAN1,
        (cantx, canrx),
        &mut afio.mapr,
        &mut rcc.apb1,
        device.USB,
    ));

    // -------- Heating control related:
    watchdog.feed();

    // DS18B20 1-wire temperature sensors connected to B4 GPIO
    let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);
    let delay = Delay::new(core.SYST, clocks);
    let mut one_wire = OneWirePort::new(onewire_io, delay).unwrap();

    // -------- Generic user interface related
    // Read the NEC IR remote commands on A15 GPIO as input with internal pullup
    let ir_receiver = pa15.into_pull_up_input(&mut gpioa.crh);
    let mut receiver = ir::IrReceiver::new();

    // PT8211 DAC (BCK, DIN, WS) on B10, B11, B12
    let mut dac = room_pill::dac::Pt8211::new(
        gpiob.pb10.into_push_pull_output(&mut gpiob.crh), //use as SCL?
        gpiob.pb11.into_push_pull_output(&mut gpiob.crh), //use as SDA?
        gpiob.pb12.into_push_pull_output(&mut gpiob.crh), //word select (left / right^)
    );

    // RGB led on PB13, PB14, PB15 as push pull output
    let mut rgb = RgbLed::new(
        gpiob.pb13.into_open_drain_output(&mut gpiob.crh),
        gpiob.pb15.into_open_drain_output(&mut gpiob.crh),
        gpiob.pb14.into_open_drain_output(&mut gpiob.crh),
    );
    rgb.color(Colors::Black).unwrap();

    // C13 on board LED^
    let mut led = gpioc.pc13.into_open_drain_output(&mut gpioc.crh);
    led.set_high().unwrap();

    //USART1
    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;
    let serial = Serial::usart1(
        device.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(2_000_000.bps()),
        clocks,
        &mut rcc.apb2,
    );
    let (mut tx, rx) = serial.split();

    // Optional piezzo speaker on A8 (open drain output)
    // TODO into_alternate_open_drain
    let _piezzo_pin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    // let mut piezzo =
    //     Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2).pwm(piezzo_pin, &mut afio.mapr, 1.khz()); //pwm::<Tim1NoRemap, _, _, _>
    // piezzo.set_duty(Channel::C1, piezzo.get_max_duty() / 2);
    // piezzo.disable(Channel::C1);

    // A6, A7 not used, connected to the ground
    let _a6 = gpioa.pa6.into_pull_down_input(&mut gpioa.crl);
    let _a7 = gpioa.pa7.into_pull_down_input(&mut gpioa.crl);
    // B1 not connected
    let _b1 = gpiob.pb1.into_pull_down_input(&mut gpiob.crl);

    // B3 not used, connected to the ground
    #[cfg(not(feature = "itm-debug"))]
    let _b3 = _pb3_itm_swo.into_pull_down_input(&mut gpiob.crl);
    #[cfg(feature = "itm-debug")]
    let _b3 = _pb3_itm_swo.into_push_pull_output(&mut gpiob.crl);
    // B5 not used, connected to the ground
    let _b5 = gpiob.pb5.into_pull_down_input(&mut gpiob.crl);

    // Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
    // let _ssr_0 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
    // let _ssr_1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    let _ssr_2 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let _ssr_3 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
    // C14, C15 used on the bluepill board for 32768Hz xtal
    watchdog.feed();

    const MAX_THERMOMETER_COUNT: usize = 2; //max number of thermometers

    //store the addresses of temp sensors, start measurement on each:
    let mut roms = [[0u8; 8]; MAX_THERMOMETER_COUNT];
    let mut count = 0;

    let mut it = RomIterator::new(0);
    loop {
        watchdog.feed();

        match one_wire.iterate_next(true, &mut it) {
            Ok(None) => {
                break; //no or no more devices found -> stop
            }

            Ok(Some(rom)) => {
                if let Some(_device_type) = detect_18x20_devices(rom[0]) {
                    //writeln!(tx, "rom: {:?}", &rom).unwrap();

                    //TODO use this address as unique id on the CAN bus!
                    roms[count] = *rom;
                    //messenger.transmit(ID_TEMPERATURE, Payload::new(rom));
                    count = count + 1;
                    let _ = one_wire.start_temperature_measurement(&rom);
                    if count >= MAX_THERMOMETER_COUNT {
                        break;
                    }
                }
                continue;
            }

            Err(_e) => {
                rgb.color(Colors::White).unwrap();
                break;
            }
        }
    }
    //not mutable anymore
    let roms = roms;
    let count = count;
    let mut temperatures = [Option::<Temperature>::None; MAX_THERMOMETER_COUNT];

    let mut lux = Option::<u32>::None;

    // -------- Config finished
    watchdog.feed();
    let tick = Ticker::new(core.DWT, core.DCB, clocks);
    //let tick = MonoTimer::new(core.DWT, clocks); //core.DCB,

    //todo beep(piezzo, 440, 100);

    let mut last_time = tick.now();
    let mut last_big_time = last_time;
    let mut last_sound_time = last_time;

    let mut sound = 0u8;

    rgb.color(Colors::Black).unwrap();

    //Unique ID: 50ff6c065177535424281587 [40, 97, 100, 18, 46, 79, 94, 252]
    //hprintln!("Unique ID: {} {:?}", stm32_device_signature::device_id_hex(), roms[0]).unwrap();

    //main update loop
    loop {
        watchdog.feed();

        // calculate the time since last execution:
        let now = tick.now();
        let delta = tick.to_us(now - last_time); //in case of 72MHz sysclock this works if less than 59sec passed between two calls
        last_time = now;
       
        //update the IR receiver statemachine:
        let ir_cmd = receiver.receive(ir_receiver.is_low().unwrap(), now, |last| {
            tick.to_us(now - last).into()
        });

        // if tick.to_us(tick.now() - last_sound_time) > sound_period {
        //     last_sound_time = now;
        //     sound = sound & 127;
        //     let a = (SINUS[sound as usize] as i16) * 256;
        //     dac.stereo(a as u16, a as u16).unwrap();            
        //     sound += 7; //appprox 440hz = 7*8000/128                        
        // }

        match ir_cmd {
            Ok(ir::NecContent::Repeat) => {}
            Ok(ir::NecContent::Data(data)) => {
                let ir_command = translate(data);
            }
            _ => {}
        }

        switch_a.update(delta).unwrap();
        switch_b.update(delta).unwrap();
        switch_c.update(delta).unwrap();
        switch_d.update(delta).unwrap();

        //messenger.receive_log(&mut tx);
        if let Some((filter_match_index, time_stamp, frame)) = messenger.try_receive() {
            // hprintln!(
            //     "Door f:{} t:{} i:{} d:{}",
            //     filter_match_index,
            //     time_stamp,
            //     frame.id().standard(),
            //     frame.data().data_as_u64()
            // ).unwrap();
        }

        // do not execute the followings too often: (temperature conversion time of the sensors is a lower limit)
        let big_delta = tick.to_us(now - last_big_time);
        if big_delta < one_sec {
            continue;
        }
        last_big_time = now;
        //rgb.color(Colors::Green).unwrap();
        //messenger.transmit(ID_MOVEMENT, Payload::new(&roms[0]));

        if motion_alarm.is_high() == Ok(true) {
            //TODO send can message on rising edges
            //rgb.color(Colors::Purple).unwrap(); //todo remove?
            //let _ = messenger.transmit(ID_MOVEMENT, Payload::new(&roms[0]));
        }

        if open_alarm.is_high() == Ok(true) {
            //TODO send can message on rising edges
            //rgb.color(Colors::Cyan).unwrap(); //todo remove?
            //messenger.transmit(ID_OPEN, Payload::new(&roms[0]));
        }

        //read sensors and restart temperature measurement
        for i in 0..count {
            temperatures[i] = match one_wire.read_temperature_measurement_result(&roms[i]) {
                Ok(temperature) => {
                    //TODO
                    // let mut buffer = [0u8; 8];
                    // temperature.whole_degrees().numtoa(10, &mut buffer);
                    // // buffer[4] = b'.';
                    // // temperature.fraction_degrees().numtoa(10, &mut buffer);
                    // messenger.transmit(
                    //  	ID_OPEN,
                    // 	Payload::new(&buffer),
                    // );

                    Some(temperature)
                }
                Err(_code) => None,
            };

            let _ = one_wire.start_temperature_measurement(&roms[i]);
        }

        //TODO send can message
        //TODO receive can light control
        //TODO receive can sound ?
        //TODO receive can ring control ?
        //TODO measure light and send can message in case of change
        let light: u32 = adc1.read(&mut adc8_photo_resistor).unwrap();
        if let Some(l) = lux {
            if l != light {
                lux = Some(light);
                //TODO send can message
            }
        } else {
            lux = Some(light);
        }

        led.toggle().unwrap();
    }
}

static SINUS: [i8; 128] = [
    0, 6, 12, 19, 25, 31, 37, 43, 49, 54, 60, 65, 71, 76, 81, 85, 90, 94, 98, 102, 106, 109, 112,
    115, 117, 120, 122, 123, 125, 126, 126, 127, 127, 127, 126, 126, 125, 123, 122, 120, 117, 115,
    112, 109, 106, 102, 98, 94, 90, 85, 81, 76, 71, 65, 60, 54, 49, 43, 37, 31, 25, 19, 12, 6, 0,
    -6, -12, -19, -25, -31, -37, -43, -49, -54, -60, -65, -71, -76, -81, -85, -90, -94, -98, -102,
    -106, -109, -112, -115, -117, -120, -122, -123, -125, -126, -126, -127, -127, -127, -126, -126,
    -125, -123, -122, -120, -117, -115, -112, -109, -106, -102, -98, -94, -90, -85, -81, -76, -71,
    -65, -60, -54, -49, -43, -37, -31, -25, -19, -12, -6,
];
