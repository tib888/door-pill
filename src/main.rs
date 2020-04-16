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

use stm32f1xx_hal::{
    can::*, 
    delay::Delay, 
    gpio::{ExtiPin, *}, 
    prelude::*, 
    rtc, 
    watchdog::IndependentWatchdog
};
use embedded_hal::{
    digital::v2::InputPin,
    watchdog::{Watchdog, WatchdogEnable},
};
use heapless::{
    consts::*,
    i,
    spsc::{Consumer, Producer, Queue},
};
use onewire::*;
use room_pill::{
    ir,
    ir::NecReceiver,
    ir_remote::*,
    rgb::{Colors, RgbLed},
};
use rtfm::{
    app,
    cyccnt::{Duration, Instant, U32Ext},
};


#[cfg(not(feature = "itm-debug"))]
use panic_halt as _;
#[cfg(feature = "itm-debug")]
use panic_itm as _;

const MHZ: u32 = 72;

#[derive(Copy, Clone, PartialEq)]
pub enum OnOff {
    Off,
    On,
}

pub struct AcSwitch2<PIN>
where
    PIN: InputPin,
{
    pin: PIN,
    ac_period: Duration,
    rised_at: Option<Instant>,
}

impl<PIN> AcSwitch2<PIN>
where
    PIN: InputPin,
{
    pub fn new(ac_period: Duration, pin: PIN) -> AcSwitch2<PIN> {
        AcSwitch2 {
            pin: pin,
            ac_period: ac_period,
            rised_at: Option::None,
        }
    }

    pub fn update_state(&mut self, now: Instant) -> OnOff {
        if let Ok(true) = self.pin.is_high() {
            self.rised_at = Some(now);
        }
        if let Some(rised) = self.rised_at {
            if now.duration_since(rised) <= self.ac_period {
                return OnOff::On;
            };
        };
        OnOff::Off
    }
}

#[app(device = stm32f1xx_hal::pac, monotonic = rtfm::cyccnt::CYCCNT, peripherals = true)]
const APP: () = {
    struct Resources {
        ir_producer: Producer<'static, ir::NecContent, U4>,
        ir_consumer: Consumer<'static, ir::NecContent, U4>,
        WATCHDOG: IndependentWatchdog,
        SWITCH_A: AcSwitch2<stm32f1xx_hal::gpio::gpioa::PA2<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        SWITCH_B: AcSwitch2<stm32f1xx_hal::gpio::gpioa::PA3<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        SWITCH_C: AcSwitch2<stm32f1xx_hal::gpio::gpioa::PA1<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        SWITCH_D: AcSwitch2<stm32f1xx_hal::gpio::gpioa::PA0<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>>,
        IR_RECEIVER: stm32f1xx_hal::gpio::gpioa::PA15<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::PullUp>>,
        NEC_RECEIVER: ir::IrReceiver<Instant>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // all interrupts are disabled here
        // let _: Instant = start;
        // let _: rtfm::Peripherals = core;
        // let _: stm32f103xx::Peripherals = device;
        // let _: init::Schedule = schedule;
        // let _: init::Spawn = spawn;

        let device = cx.device;

        let mut watchdog = IndependentWatchdog::new(device.IWDG);
        watchdog.start(2_000u32.ms());

        let mut flash = device.FLASH.constrain();

        // flash.acr.prftbe().enabled();//?? Configure Flash prefetch - Prefetch buffer is not available on value line devices
        // scb().set_priority_grouping(NVIC_PRIORITYGROUP_4);

        let mut rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(MHZ.mhz())
            .hclk(MHZ.mhz())
            .pclk1(36.mhz())
            .pclk2(MHZ.mhz())
            .adcclk(12.mhz())
            .freeze(&mut flash.acr);

        watchdog.feed();

        // real time clock
        let mut pwr = device.PWR;
        let mut backup_domain = rcc.bkp.constrain(device.BKP, &mut rcc.apb1, &mut pwr);
        let _rtc = rtc::Rtc::rtc(device.RTC, &mut backup_domain);
        watchdog.feed();

        // Configure pins:
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        // Disables the JTAG to free up pb3, pb4 and pa15 for normal use
        let (pa15, _pb3_itm_swo, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Switces on A0, A1, A2, A3 (pull down once in each main period if closed)
        let ac_period = (clocks.sysclk().0 / 50u32).cycles();

        let mut pa0 = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);
        pa0.make_interrupt_source(&mut afio);
        pa0.trigger_on_edge(&device.EXTI, Edge::RISING_FALLING);
        let switch_d = AcSwitch2::new(ac_period, pa0);

        let mut pa1 = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);
        pa1.make_interrupt_source(&mut afio);
        pa1.trigger_on_edge(&device.EXTI, Edge::RISING_FALLING);
        let switch_c = AcSwitch2::new(ac_period, pa1);

        let mut pa2 = gpioa.pa2.into_pull_up_input(&mut gpioa.crl);
        pa2.make_interrupt_source(&mut afio);
        pa2.trigger_on_edge(&device.EXTI, Edge::RISING_FALLING);
        let switch_a = AcSwitch2::new(ac_period, pa2);

        let mut pa3 = gpioa.pa3.into_pull_up_input(&mut gpioa.crl);
        pa3.make_interrupt_source(&mut afio);
        pa3.trigger_on_edge(&device.EXTI, Edge::RISING_FALLING);
        let switch_b = AcSwitch2::new(ac_period, pa3);

        // Motion alarm on A4 (pull down)
        let _motion_alarm = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);

        // Open alarm on A5 (pull down)
        let _open_alarm = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);

        // A6, A7 not used, connected to the ground
        let _a6 = gpioa.pa6.into_pull_down_input(&mut gpioa.crl);
        let _a7 = gpioa.pa7.into_pull_down_input(&mut gpioa.crl);

        // Optional piezzo speaker on A8
        let _piezzo = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);

        // Solid state relay connected to A9 drives the lamp_b
        let _ssr_lamp_b = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

        // Solid state relay connected to A10 drives the lamp_a
        let _ssr_lamp_a = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

        // CAN (RX, TX) on A11, A12
        let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        // USB is needed here because it can not be used at the same time as CAN since they share memory:
        let _can = Can::can1(
            device.CAN1,
            (cantx, canrx),
            &mut afio.mapr,
            &mut rcc.apb1,
            device.USB,
        );

        // Read the NEC IR remote commands on A15 GPIO as input with internal pullup
        let ir_receiver = pa15.into_pull_up_input(&mut gpioa.crh);

        // Photoresistor on B0 (ADC8)
        let _photoresistor = gpiob.pb0.into_floating_input(&mut gpiob.crl);

        // B1 not connected
        let _b1 = gpiob.pb1.into_pull_down_input(&mut gpiob.crl);

        // B3 not used, connected to the ground
        #[cfg(not(feature = "itm-debug"))]
        let _b3 = _pb3_itm_swo.into_pull_down_input(&mut gpiob.crl);
        #[cfg(feature = "itm-debug")]
        let _b3 = _pb3_itm_swo.into_push_pull_output(&mut gpiob.crl);

        // DS18B20 1-wire temperature sensors connected to B4 GPIO
        let onewire_io = pb4.into_open_drain_output(&mut gpiob.crl);

        // B5 not used, connected to the ground
        let _b5 = gpiob.pb5.into_pull_down_input(&mut gpiob.crl);

        // Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
        let _ssr_0 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
        let _ssr_1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
        let _ssr_2 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
        let _ssr_3 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

        // PT8211 DAC (BCK, DIN, WS) on B10, B11, B12
        let _dac = room_pill::dac::Pt8211::new(
            gpiob.pb10.into_push_pull_output(&mut gpiob.crh), //use as SCL?
            gpiob.pb11.into_push_pull_output(&mut gpiob.crh), //use as SDA?
            gpiob.pb12.into_push_pull_output(&mut gpiob.crh), //word select (left / right^)
        );

        // RGB led on PB13, PB14, PB15 as push pull output
        let mut rgb = RgbLed::new(
            gpiob.pb13.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb14.into_push_pull_output(&mut gpiob.crh),
            gpiob.pb15.into_push_pull_output(&mut gpiob.crh),
        );
        rgb.color(Colors::Black);

        // C13 on board LED^
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        //led.set_high().unwrap();

        // C14, C15 used on the bluepill board for 32768Hz xtal
        watchdog.feed();

        let mut delay = Delay::new(cx.core.SYST, clocks);
        let mut _one_wire = OneWirePort::new(onewire_io, delay);

        let receiver = ir::IrReceiver::<Instant>::new();
        
        //setup external interrupts EXTI0,1,2,3,4,EXTI9_5,EXTI15_10
                     
        static mut IR_QUEUE: Queue<ir::NecContent, U4> = Queue(i::Queue::new());
        
        unsafe {
            let (ir_producer, ir_consumer) = IR_QUEUE.split();
            
            // Initialization of late resources
            init::LateResources {
                ir_producer: ir_producer,
                ir_consumer: ir_consumer,
                WATCHDOG: watchdog,            
                SWITCH_A: switch_a,
                SWITCH_B: switch_b,
                SWITCH_C: switch_c,
                SWITCH_D: switch_d,
                NEC_RECEIVER: receiver,
                IR_RECEIVER: ir_receiver,
            }
        }
    }

    #[idle(resources = [WATCHDOG, ir_consumer])] 
    fn idle(c: idle::Context) -> ! {
        // the interrupts are enabled here
        loop {
            c.resources.WATCHDOG.feed();

            if let Some(ir_cmd) = c.resources.ir_consumer.dequeue() {
                match ir_cmd {
                    ir::NecContent::Repeat => {}
                    ir::NecContent::Data(data) => {
                        let command = translate(data);
                        // write!(hstdout, "{:x}={:?} ", data, command).unwrap();
                        // model.ir_remote_command(command, &MENU);
                        // model.refresh_display(&mut display, &mut backlight);
                    }
                }
            }
        }
    }

    #[task(
        binds = EXTI0,
        priority = 1,
        resources = [SWITCH_D],
    )]
    fn exti0(c: exti0::Context) {
        let state = c.resources.SWITCH_D.update_state(Instant::now()); 
        //TODO on state change notify the main loop...
    }

    #[task(
        binds = EXTI1,
        priority = 1,
        resources = [SWITCH_C],
    )]
    fn exti1(c: exti1::Context) {
        let state = c.resources.SWITCH_C.update_state(Instant::now());
        //TODO on state change notify the main loop...
    }

    #[task(
        binds = EXTI2,
        priority = 1,
        resources = [SWITCH_A],
    )]
    fn exti2(c: exti2::Context) {
        let state = c.resources.SWITCH_A.update_state(Instant::now());
        //TODO on state change notify the main loop...
        //and toggle lamp A?
    }

    #[task(
        binds = EXTI3,
        priority = 1,
        resources = [SWITCH_B],
    )]
    fn exti3(c: exti3::Context) {
        let state = c.resources.SWITCH_B.update_state(Instant::now());
        //TODO on state change notify the main loop...
        //and toggle lamp B?
    }

    #[task(
        binds = EXTI15_10,
        priority = 2,
        resources = [NEC_RECEIVER, IR_RECEIVER, ir_producer],   
        // schedule = [foo],
        // spawn = [foo, bar],
    )]
    fn exti15_10(c: exti15_10::Context) {
        // let _: Instant = start;
        // let _: resources::IR_FIFO_P = resources.IR_FIFO_P;
        // let _: EXTI0::Schedule = schedule;
        // let _: EXTI0::Spawn = spawn;
        let now = Instant::now();

        let ir_cmd = c.resources.NEC_RECEIVER.receive(
            c.resources.IR_RECEIVER.is_low().unwrap(),
            now,
            |last| now.duration_since(last).as_cycles() / MHZ,
        ); //in case of 72MHz, 72 clock = 1 microsecond

        // update the IR receiver statemachine:
        if let Ok(ir_cmd) = ir_cmd {
            c.resources.ir_producer.enqueue(ir_cmd);
        }
    }

    //EXTI line 16 is connected to the PVD output
    //EXTI line 17 is connected to the RTC Alarm event
    //EXTI line 18 is connected to the USB Wakeup event

    // #[exception(schedule = [foo], spawn = [foo])]
    // fn SVCall() {
    //     let _: Instant = start;
    //     let _: SVCall::Schedule = schedule;
    //     let _: SVCall::Spawn = spawn;
    // }

    // #[task(priority = 2, resources = [SHARED], schedule = [foo], spawn = [foo])]
    // fn foo() {
    //     let _: Instant = scheduled;
    //     let _: Exclusive<u32> = resources.SHARED;
    //     let _: foo::Resources = resources;
    //     let _: foo::Schedule = schedule;
    //     let _: foo::Spawn = spawn;
    // }
    // extern "C" {
    //     fn UART1();
    // }
};
