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

#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate panic_halt;
#[macro_use]
extern crate cortex_m_semihosting as sh;
extern crate embedded_hal;
extern crate ir;
extern crate nb;
extern crate onewire;
extern crate room_pill;
extern crate rtfm;
extern crate stm32f103xx_hal as hal;

use crate::hal::{
    can::*, delay::Delay, gpio::*, prelude::*, rtc, stm32f103xx, watchdog::IndependentWatchdog,
};
use embedded_hal::{
    digital::InputPin,
    watchdog::{Watchdog, WatchdogEnable},
};
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};
use ir::NecReceiver;
use onewire::*;
use room_pill::{
    ir_remote::*,
    rgb::{Colors, RgbLed},
};
use rtfm::{app, Duration, Instant, U32Ext};
// use stm32f103xx::Interrupt;
// use sh::hio;
// use core::fmt::Write;

#[derive(Copy, Clone)]
pub struct MyInstant(rtfm::Instant);

impl ir::Instant for MyInstant {
    fn elapsed_us_till(&self, now: &Self) -> u32 {
        0 //TODO now.0.duration_since(self.0).cycles() / 72;
    }
}

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
        if self.pin.is_high() {
            self.rised_at = Some(now);
        };

        if let Some(rised) = self.rised_at {
            if now.duration_since(rised) <= self.ac_period {
                return OnOff::On;
            };
        };
        OnOff::Off
    }
}

#[app(device = stm32f103xx)]
const APP: () = {
    // Late resources
    static mut IR_FIFO_P: Producer<'static, ir::NecContent, U4> = ();
    static mut IR_FIFO_C: Consumer<'static, ir::NecContent, U4> = ();
    static mut WATCHDOG: IndependentWatchdog = ();
    static mut SWITCH_A: AcSwitch2<hal::gpio::gpioa::PA2<hal::gpio::Input<hal::gpio::PullUp>>> = ();
    static mut SWITCH_B: AcSwitch2<hal::gpio::gpioa::PA3<hal::gpio::Input<hal::gpio::PullUp>>> = ();
    static mut SWITCH_C: AcSwitch2<hal::gpio::gpioa::PA1<hal::gpio::Input<hal::gpio::PullUp>>> = ();
    static mut SWITCH_D: AcSwitch2<hal::gpio::gpioa::PA0<hal::gpio::Input<hal::gpio::PullUp>>> = ();
    static mut IR_RECEIVER: hal::gpio::gpioa::PA15<hal::gpio::Input<hal::gpio::PullUp>> = ();
    static mut NEC_RECEIVER: ir::IrReceiver<MyInstant> = ();

    #[init]
    // #[init(schedule = [foo], spawn = [foo])]
    fn init() {
        // let now = Instant::now();

        // all interrupts are disabled here
        // let _: Instant = start;
        // let _: rtfm::Peripherals = core;
        // let _: stm32f103xx::Peripherals = device;
        // let _: init::Schedule = schedule;
        // let _: init::Spawn = spawn;

        let mut watchdog = IndependentWatchdog::new(device.IWDG);
        watchdog.start(2_000_000u32.us());

        let mut flash = device.FLASH.constrain();

        // flash.acr.prftbe().enabled();//?? Configure Flash prefetch - Prefetch buffer is not available on value line devices
        // scb().set_priority_grouping(NVIC_PRIORITYGROUP_4);

        let mut rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            //.sysclk(72.mhz())
            //.hclk(72.mhz())
            //.pclk1(36.mhz())
            //.pclk2(72.mhz())
            //.adcclk(12.mhz())
            .freeze(&mut flash.acr);

        watchdog.feed();

        // real time clock
        let rtc = rtc::Rtc::new(device.RTC, &mut rcc.apb1, &mut device.PWR);
        watchdog.feed();

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        // Disables the JTAG to free up pb3, pb4 and pa15 for normal use
        afio.mapr.disable_jtag();

        // Configure pins:
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

        // Switces on A0, A1, A2, A3 (pull down once in each main period if closed)
        let ac_period = (clocks.sysclk().0 / 50u32).cycles();
        let mut switch_d = AcSwitch2::new(ac_period, gpioa.pa0.into_pull_up_input(&mut gpioa.crl));
        let mut switch_c = AcSwitch2::new(ac_period, gpioa.pa1.into_pull_up_input(&mut gpioa.crl));
        let mut switch_a = AcSwitch2::new(ac_period, gpioa.pa2.into_pull_up_input(&mut gpioa.crl));
        let mut switch_b = AcSwitch2::new(ac_period, gpioa.pa3.into_pull_up_input(&mut gpioa.crl));

        // Motion alarm on A4 (pull down)
        let motion_alarm = gpioa.pa4.into_pull_up_input(&mut gpioa.crl);

        // Open alarm on A5 (pull down)
        let open_alarm = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);

        // A6, A7 not used, connected to the ground
        let _a6 = gpioa.pa6.into_pull_down_input(&mut gpioa.crl);
        let _a7 = gpioa.pa7.into_pull_down_input(&mut gpioa.crl);

        // Optional piezzo speaker on A8
        let _piezzo = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);

        // Solid state relay connected to A9 drives the lamp_b
        let mut ssr_lamp_b = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);

        // Solid state relay connected to A10 drives the lamp_a
        let mut ssr_lamp_a = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);

        // CAN (RX, TX) on A11, A12
        let canrx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let cantx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        // USB is needed here because it can not be used at the same time as CAN since they share memory:
        let mut can = Can::can1(
            device.CAN,
            (cantx, canrx),
            &mut afio.mapr,
            &mut rcc.apb1,
            device.USB,
        );

        // Read the NEC IR remote commands on A15 GPIO as input with internal pullup
        let ir_receiver = gpioa.pa15.into_pull_up_input(&mut gpioa.crh);

        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // Photoresistor on B0 (ADC8)
        let photoresistor = gpiob.pb0.into_floating_input(&mut gpiob.crl);

        // B1 not connected
        let _b1 = gpiob.pb1.into_pull_down_input(&mut gpiob.crl);

        // B3 not used, connected to the ground
        let _b3 = gpiob.pb3.into_pull_down_input(&mut gpiob.crl);

        // DS18B20 1-wire temperature sensors connected to B4 GPIO
        let onewire_io = gpiob.pb4.into_open_drain_output(&mut gpiob.crl);

        // B5 not used, connected to the ground
        let _b5 = gpiob.pb5.into_pull_down_input(&mut gpiob.crl);

        // Solid state relay or arbitrary unit can be connected to B6, B7, B8, B9
        let _ssr_0 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
        let _ssr_1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
        let _ssr_2 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
        let _ssr_3 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);

        // PT8211 DAC (BCK, DIN, WS) on B10, B11, B12
        let dac = room_pill::dac::Pt8211::new(
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

        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        // C13 on board LED^
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        // C14, C15 used on the bluepill board for 32768Hz xtal
        watchdog.feed();

        let cp = cortex_m::Peripherals::take().unwrap();

        let mut delay = Delay::new(cp.SYST, clocks);
        let mut one_wire = OneWirePort::new(onewire_io, delay);

        let mut receiver = ir::IrReceiver::<MyInstant>::new();

        // Schedule `bar` to run 4e6 cycles in the future
        // schedule.foo(now + 4_000_000.cycles()).unwrap();

        // NOTE: we use `Option` here to work around the lack of
        // a stable `const` constructor
        static mut Q: Option<Queue<ir::NecContent, U4>> = None;
        Q = Some(Queue::new());
        let (p, c) = Q.as_mut().unwrap().split();

        //setup external interrupts EXTI0,1,2,3,4,EXTI9_5,EXTI15_10

        // A0 = switch_d, A1 = switch_c, A2 = switch_a, A3 = switch_b,
        // A4 = Motion,
        // A5 = Open,
        // A15 = ir_remote
        // -> trigger on rising edge
        device
            .EXTI
            .rtsr
            .write(|w| w.bits(0b_0000_0000_0000_0000_1000_0000_0011_1111));

        // A0 = switch_d, A1 = switch_c, A2 = switch_a, A3 = switch_b,
        // A4 = Motion,
        // A5 = Open,
        // A15 = ir_remote
        // -> trigger on rising edge
        device
            .EXTI
            .ftsr
            .write(|w| w.bits(0b_0000_0000_0000_0000_1000_0000_0011_1111));

        //device.EXTI.imr // interrupt mask register - handled by rtfm when finished with init()?
        //device.EXTI.emr // event mask - not needed
        //device.EXTI.swier // software interrupt request - not needed
        //device.EXTI.pr // pending request - handled by rtfm when exiting from irq?

        // Initialization of late resources
        WATCHDOG = watchdog;
        IR_FIFO_P = p;
        IR_FIFO_C = c;
        SWITCH_A = switch_a;
        SWITCH_B = switch_b;
        SWITCH_C = switch_c;
        SWITCH_D = switch_d;
        NEC_RECEIVER = receiver;
        IR_RECEIVER = ir_receiver;
    }

    #[idle(resources = [WATCHDOG, IR_FIFO_C])]
    fn idle() -> ! {
        // the interrupts are enabled here
        loop {
            resources.WATCHDOG.feed();

            if let Some(ir_cmd) = resources.IR_FIFO_C.dequeue() {
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

    #[interrupt(
        priority = 1,
        resources = [SWITCH_D],
    )]
    fn EXTI0() {
        let state = resources.SWITCH_D.update_state(start);
        //TODO on state change notify the main loop...
    }

    #[interrupt(
        priority = 1,
        resources = [SWITCH_C],
    )]
    fn EXTI1() {
        let state = resources.SWITCH_C.update_state(start);
        //TODO on state change notify the main loop...
    }

    #[interrupt(
        priority = 1,
        resources = [SWITCH_A],
    )]
    fn EXTI2() {
        let state = resources.SWITCH_A.update_state(start);
        //TODO on state change notify the main loop...
        //and toggle lamp A?
    }

    #[interrupt(
        priority = 1,
        resources = [SWITCH_B],
    )]
    fn EXTI3() {
        let state = resources.SWITCH_B.update_state(start);
        //TODO on state change notify the main loop...
        //and toggle lamp B?
    }

    #[interrupt(
        priority = 2,
        resources = [IR_FIFO_P, NEC_RECEIVER, IR_RECEIVER],
        // schedule = [foo],
        // spawn = [foo, bar],
    )]
    fn EXTI15_10() {
        // let _: Instant = start;
        // let _: resources::IR_FIFO_P = resources.IR_FIFO_P;
        // let _: EXTI0::Schedule = schedule;
        // let _: EXTI0::Spawn = spawn;

        // update the IR receiver statemachine:
        if let Ok(ir_cmd) = resources
            .NEC_RECEIVER
            .receive(MyInstant(start), resources.IR_RECEIVER.is_low())
        {
            resources.IR_FIFO_P.enqueue(ir_cmd);
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
