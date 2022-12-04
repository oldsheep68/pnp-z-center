//
// Pinnig for MOBO REV3 (Mai21) with STM32F103 and additional modifications necessary
// Z_DIR:     PB1
// Z_STEP:    PE7
// Z_EN:      PE9
// Z_CS_UART: PE8
// Z_Limit:   PD14
// A_EN =>    PA3 wired to Z:MS2_CS_UART (0 Ohm resistor) [driver pin 12]
//               needs to be 3.3V for A49xx and treestate for TMC2208/2209

// BLE Connections
// Servo1     PB6
// Servo2     PB7

// A_Limit:   PC9

// MOSFET1:   PE2
// MOSFET2:   PE3
// MOSFET3:   PE4
// MOSFET4:   PE5


//! CDC-ACM serial port example using polling in a busy loop.
//! Target board: any STM32F103
#![no_std]
#![no_main]

use panic_halt as _;

use rtt_target::{rprintln, rprint, rtt_init_print};
// use cortex_m_semihosting::{ hprintln}; // debug,

use core::mem::MaybeUninit;
use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};

//use cortex_m::peripheral::{DWT};  // für cycling counter 32bit

//use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f1xx_hal::usb::{Peripheral, UsbBus};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    timer::{CounterUs, Event, Timer},
    //time::ms,
    //timer::{Channel, Tim1NoRemap},
}; // , pwm::*
//use usbd_serial::SerialPort;
use usbd_serial::{SerialPort}; // , USB_CLASS_CDC

//use stm32f4::stm32f411::TIM1;
use stm32f1::stm32f103::TIM1;
use usb_device::prelude::*;

use stm32f1xx_hal as hal;
use crate::hal::{
    pac::interrupt,
    // delay::Delay,
    gpio::{
           // gpioa::PA3,
           gpiob::{PB1}, // PB3, PB4, , PB6, PB7
           //gpiod::PD15,
           gpioe::{PE7, PE9, }, // PE2, PE3, PE8,
           Edge, Input, PullUp, Output, PushPull},
};

use stm32f1xx_hal::gpio::ExtiPin;

// extern crate stm32f4;
// use stm32f4::interrupt;

// use embedded_hal::PwmPin;
// use stm32f1xx_hal::dwt::DwtExt;

extern crate gcode;
extern crate arrayvec;
use arrayvec::ArrayVec;
use gcode::{Mnemonic}; // Callbacks, Span,

use core::ops::DerefMut;

use heapless::spsc::Queue;
use heapless::String;

use ramp_maker::{MotionProfile as _, Trapezoidal};
use fixed::{ types::extra::U32, FixedI64}; // types::extra::U16, types::extra::U20, FixedI32,

//static mut EP_MEMORY: [u32; 1024] = [0; 1024];
const W_OFFSET:u32 = 203;   // set HOME Offset for W axis
const READ_SWITCH:u32 = 489;
const SET_SERVO_POSITION:u32 = 280;
const READ_FW_VERSION:u32 = 115;
const READ_POSITION:u32 = 114;
const SET_MM_MODE:u32 = 21;
const G0_POSITION:u32 = 0;
const G28_HOME:u32 = 28;

const INVERS_MOTOR_DIR:bool = false;   // false fits with inverted motor cable (other not tested)

// servo duty-cycles for BLTouch (with 200Hz frequency = 5ms)
// 90° = 1473us  = 0.2946  19307:u16 (push pin up)
// 10° = 647us   = 0.1294   8480:u16 (pusch down)
// 60° = 1162us  = 0.2324  15231:u16 (alarm release)
// 120° = 1782us = 0.3564  23357:u16 (self test)
// 160° = 2194us = 0.4388  28757:u16 ( alarm releas and push pin up)
const PUSH_PIN_UP:u16           = 17100;
const PUSH_PIN_DOWN:u16         = 7670;
const RELEASE_ALARM:u16         = 13500;
const SELF_TEST:u16             = 21357;
const RELEASE_ALARM_PUSH_UP:u16 = 26400;

//ble_event: &mut stm32f1xx_hal::gpio::Pin<Input<PullUp>, CRH, 'C', 13>
//expected struct `stm32f1xx_hal::gpio::Pin<Input<PullUp>, CRH, 'C', 13>`

//static BLE_EVENT: Mutex<RefCell<Option<&mut PC13<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static STATE: Mutex<Cell<BleEventState>> = Mutex::new(Cell::new(BleEventState::Waiting));

// Set up global state. It's all mutexed up for concurrency safety.
// static ELAPSED_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0u32));
static G_TIMER_TIM2: Mutex<RefCell<Option<CounterUs<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioc::PC9<Input<PullUp>>> =
    MaybeUninit::uninit();
static mut INT_Z_ENDSTOP_PIN  : MaybeUninit<stm32f1xx_hal::gpio::gpiod::PD14<Input<PullUp>>> =
        MaybeUninit::uninit();

#[derive(Clone, Copy)]
enum BleEventState {
    Triggered,
    Waiting,
}

// Stepper recources
type Steper0Step = PE7<Output<PushPull>>;
type Steper0En = PE9<Output<PushPull>>;
type Steper0Dir = PB1<Output<PushPull>>;
// static G_STEPER0_STEP: Mutex<RefCell<Option<Seper0Step>>> = Mutex::new(RefCell::new(None));
// static G_STEPER0_DIR: Mutex<RefCell<Option<Seper0Dir>>> = Mutex::new(RefCell::new(None));

fn copy_static2u8(from: &[u8], to: &mut [u8]){
    for (i, el) in from.iter().enumerate() {
        to[i] = *el;
    }
}

#[derive(PartialEq)]
enum StepperMoveState {
    IDLE,
    SendOk,
    G28Homing,
    MOVE,
}
// #[derive(Clone, Copy)]
struct Zctrl {
    current_psition: i32,
    target_position: i32,
    ble_trigger_position: i32,
    step_dir: bool,
    //distance: i32,
    // max_spped: i32,
    // max_accel: i32,
    max_velocity: FixedI64::<U32>,
    target_accel: FixedI64::<U32>,
    profile: Trapezoidal<FixedI64::<U32>>,
    finished: bool,
    end_stop_irq: bool,
    steper_z_step: Steper0Step,
    steper_z_dir: Steper0Dir,
    steper_z_en: Steper0En,
    // elapsed: bool,
    counts_next: FixedI64::<U32>,  // used to count between steps pin
    move_state: StepperMoveState,
}

static G_Z_CTRL: Mutex<RefCell<Option<Zctrl>>> = Mutex::new(RefCell::new(None));


#[entry]
fn main() -> ! {
    let ok_answer = b"\nok\n";
    let firmware_answer = b"\nFIRMWARE V1.0.0\nok\n\r";   // M115
    // let firmware_answer = b"\nFIRMWARE V1.0.0\n";   // M115
    let z_pos_answer =    b"\nZ:0.00            \n";
    let trigger_answer = b"\n1\nok\n\r";

    let mut ok_answer_buff: [u8;4] = [0;4];
    let mut firmware_answer_buf: [u8;22] = [32;22];
    let mut z_pos_answer_buf: [u8;20] = [32;20];
    let mut trigger_answer_buf: [u8;7] = [32;7];
    // for (i, el) in ok_answer.iter().enumerate() {
    //     ok_answer_buff[i] = *el;
    // }
    copy_static2u8(ok_answer, &mut ok_answer_buff);
    copy_static2u8(firmware_answer, &mut firmware_answer_buf);
    copy_static2u8(z_pos_answer, &mut z_pos_answer_buf);
    copy_static2u8(trigger_answer, &mut trigger_answer_buf);


    let mut dp = pac::Peripherals::take().unwrap();
    let mut afio = dp.AFIO.constrain();
    let cp =  cortex_m::peripheral::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    rtt_init_print!();
    rprintln!("Hello from main!");

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .pclk2(48.MHz())
        //.require_pll48clk()
        .freeze(&mut flash.acr);

    // Create a delay abstraction based on DWT cycle counter
    // let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    // let mut delay = dwt.delay();
    let mut delay = cp.SYST.delay(&clocks);

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioe = dp.GPIOE.split();



    // pwm control pins
    let tim1_c1 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    let tim1_c2 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pins = (tim1_c1, tim1_c2);
    // ch1 ist pwm output for bltouch
    // let mut pwm = dp
    //     .TIM1
    //     .pwm_hz::<Tim1NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz(), &clocks);
    // pwm.enable(Channel::C1);
    let pwm = Timer::new(dp.TIM1, &clocks).pwm_hz(pins, &mut afio.mapr, 200u32.Hz());

    let _max = pwm.get_max_duty();

    let (mut ch1, _ch2) = pwm.split();

    // let channels = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());

    // let pwm = Timer::new(dp.TIM1, &clocks).pwm(channels, 200u32.hz());
    // let (mut ch1, _ch2) = pwm;


    // Create input interrupt for endswitch Z (Z_Limit:   PD14)
    let mut gpiod = dp.GPIOD.split();
    let z_limit_event = unsafe {&mut *INT_Z_ENDSTOP_PIN.as_mut_ptr() };
    *z_limit_event = gpiod.pd14.into_pull_up_input(&mut gpiod.crh);
    z_limit_event.make_interrupt_source(&mut afio);
    z_limit_event.trigger_on_edge(&mut dp.EXTI, Edge::Falling); //Edge::Falling);
    z_limit_event.enable_interrupt(&mut dp.EXTI);

    let mut w_offset:f32 = 0.0;

    rprintln!("z_limit_event inital state : {}", z_limit_event.is_high());

    // Create interrupt for ble touch event on A_Limit input (A_Limit:   PC9)
    let mut gpioc = dp.GPIOC.split();
    let ble_event = unsafe {&mut *INT_PIN.as_mut_ptr() };
    *ble_event = gpioc.pc9.into_pull_up_input(&mut gpioc.crh);
    ble_event.make_interrupt_source(&mut afio);
    ble_event.trigger_on_edge(&mut dp.EXTI, Edge::Rising); //Edge::Falling);
    ble_event.enable_interrupt(&mut dp.EXTI);


    ch1.set_duty(PUSH_PIN_UP); // 1473us (90°)
    ch1.enable();

    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    cortex_m::asm::delay(clocks.sysclk().raw() / 100);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };

    let usb_bus = UsbBus::new(usb);
    // let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("bltouchctrl")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    rprintln!("Hello write_buffer!");

    // Enable interrupts
    pac::NVIC::unpend(hal::pac::Interrupt::EXTI15_10);
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::EXTI15_10);
    };

    // Enable interrupts
    pac::NVIC::unpend(hal::pac::Interrupt::EXTI9_5);
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::EXTI9_5);
    };

    let mut rb: Queue<u8, 256> = Queue::new();
    let mut cmd_s: String<256> = String::new();

    //==================================================================
    // stepper specific stuff
    // Z_DIR:     PB1
    // Z_STEP:    PE7
    // Z_EN:      PE9
    // Z_CS_UART: PE8
    // Z_Limit:   PD15

    // A_Limit:   PC9
    //==================================================================
    rprintln!("Hello before stepper stuff");
    // stepper control pins
    let mut steper0_step = gpioe.pe7.into_push_pull_output(&mut gpioe.crl);
    let mut steper0_dir = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
    let mut steper0_en = gpioe.pe9.into_push_pull_output(&mut gpioe.crh);

    steper0_step.set_low();
    steper0_dir.set_low();
    steper0_en.set_high();  // high is stepper disabled


    // Let's use floating point numbers here to keep the example simple.
    // RampMaker also supports fixed-point numbers though.
    let steps_per_mm = 80;   // constant from mechanics
    let mm_per_second = 100; // definable max speed

    let mm_per_second2 = 200;
    let accel_in_steps_per_second2: u32 = mm_per_second2 * steps_per_mm;

    let steps_per_secoond: u32 = mm_per_second * steps_per_mm;
    let target_accel = FixedI64::<U32>::from_num(accel_in_steps_per_second2); //1000; // meters per second^2
    let max_velocity = FixedI64::<U32>::from_num(steps_per_secoond); //1500; // meters per second

    let mut profile = ramp_maker::Trapezoidal::new(target_accel);

    let num_steps = 0;
    profile.enter_position_mode(max_velocity, num_steps);

    rprintln!("Hello before TIM2");
     // Create a 1ms periodic interrupt from TIM2
    // Set up a timer expiring after 1s
    // let mut timer2 = dp.TIM2.counter_ms(&clocks);
    // timer2.start(1.secs()).unwrap();
    let mut timer2 = dp.TIM2.counter_us(&clocks);
    // rprintln!("Hello before enabel TIM2-1");
    timer2.start(50.micros()).unwrap();  // 1secs(); 1.millis(); 1.micros(); 1.nanos(); 1.minutes(); 1.hours()
    //timer2.listen(Event::C1);
    timer2.listen(Event::Update);
    rprintln!("Hello before enabel TIM2");
    // Enable interrupts
    pac::NVIC::unpend(hal::pac::Interrupt::TIM2);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }

    let z_ctrl = Zctrl {
        current_psition: 0,
        target_position: 0,
        ble_trigger_position: 0,
        step_dir: true,  // default run to positiv z value
        //distance: i32,
        // max_spped: 10,
        // max_accel: 10,
        max_velocity,
        target_accel,
        profile,
        finished: true,
        end_stop_irq: false,
        steper_z_step: steper0_step,
        steper_z_dir: steper0_dir,
        steper_z_en: steper0_en,
        // elapsed: true,
        counts_next: FixedI64::<U32>::from_num(0),
        move_state: StepperMoveState::IDLE,
    };

    cortex_m::interrupt::free(|cs| *G_Z_CTRL.borrow(cs).borrow_mut() = Some(z_ctrl));


    rprintln!("Hello before loop");

    free(|cs| {
            G_TIMER_TIM2.borrow(cs).replace(Some(timer2));
            // BUTTON.borrow(cs).replace(Some(board_btn));
        });

    loop {
        // every thing below here is called in every loop
        free(|cs| {
            if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                //rprintln!("z_ctrl.finshed {} {:?}", z_ctrl.finished, (z_ctrl.move_state == StepperMoveState::G28Homing));
                match z_ctrl.move_state {
                    StepperMoveState::G28Homing => {

                        if z_ctrl.finished {
                            if z_limit_event.is_high() {
                                rprint!("#");
                                z_ctrl.target_position = z_ctrl.current_psition - (5 * 80) as i32;
                            } else {
                                rprint!(".");
                                z_ctrl.target_position = z_ctrl.current_psition + (5 * 80) as i32;
                            }
                            start_move_motor(z_ctrl);
                        }
                    }
                    StepperMoveState::MOVE => {
                        if z_ctrl.finished {
                            z_ctrl.move_state = StepperMoveState::SendOk;
                        }
                    },
                    StepperMoveState::SendOk => {
                        write_buffer(&ok_answer_buff, 4, &mut serial);
                        z_ctrl.move_state = StepperMoveState::IDLE;
                    },
                    StepperMoveState::IDLE => {},
                }
            }
        });

        // ###################################################################################
        // Start command parsing
        if !usb_dev.poll(&mut [&mut serial]) {
            if rb.len() == 0 {
                continue;
            }

        }

        let mut buf = [0u8; 64];
        let mut drop : bool = false;
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                for c in buf[0..count].iter() {
                    rprintln!("received: {:#?}", *c);
                    // if *c == 10 {
                    //     break;
                    // }
                    if *c  == 59 {
                        drop = true;
                        rprintln!("drop = true: {:#?}", *c);
                    }

                    // rprintln!("enque: {:#?}", *c);
                    if drop == false {
                        let  res = rb.enqueue(*c).is_ok();
                        match res {
                            true  => rprintln!("enque: {:#?}", *c),
                            false => {rprintln!("error enque: {:#?}", *c);
                                break;},
                        }
                        // rb.enqueue(*c).unwrap();
                        if rb.is_full() {
                            rprintln!("######## que is full ########");
                            break;
                        }
                    }
                    if *c  == 10 {
                        drop = false;
                        rprintln!("drop = false: {:#?}", *c)
                    }

                }
            }
            _ => {}
        }


        // M119 Read Endstop States // not supported
        // M402 Stow Probe   // not supported
        // M401 Deploy Probe   // not supported
        // M280 P0 Sxx   Servo Command
        // M289 read switch state of probe
        // M226 P<pin> S<state>   // not supported
        // M203 W<offset>  sets W offset in relation to Z
        // M114 (read position, not yet implemented)
        // G0 Z10      move motor to position 10mm (absolut move)
        // G0 Z10 F10  move with a deticated speed setting (exampe to Position 10mm with max speed 10mm/sec)
        let q_len = rb.len();
        // rprintln!("q_len start Zeitpunkt {:#?}", q_len);
        if q_len > 0 {
            let mut dropping : bool = false;
            for _i in [0..q_len] {
                // rprintln!("_i: {:#?}", _i);
                let c = rb.dequeue();
                let mut chr: u8 = 10;
                match c {
                    Some(charakter) => chr = charakter,
                    None => rprintln!("######## deque error ########"),
                }
                if dropping & (chr != 10) {
                    continue;
                } else if chr == 10 {
                    dropping = false;
                }
                // rprintln!("que len(): {:#?}", rb.len());
                // if (c.unwrap() != 10) & (c.unwrap() != 13){
                if (chr != 10) & (chr != 13) {
                    if chr != 59 {
                        rprintln!("dequed c: {:#?} {:#?}", c.unwrap(), u8tostr(c.unwrap()));
                        let _res = cmd_s.push_str(u8tostr(c.unwrap()));
                        rprintln!("cmd_part: {:#?} q-len: {:#?}", cmd_s.as_str(), rb.len());
                    } else {
                        rprintln!("######## dropping false ########");
                        dropping = true;
                    }
                } else {
                    rprintln!("cmd_final: {:#?}", cmd_s.as_str());
                    let gcmds: ArrayVec::<_, 32> = gcode::parse(cmd_s.as_str()).collect();
                    for c in gcmds.iter(){
                        rprintln!("{:?}", c);
                    }
                    cmd_s.clear();

                    if gcmds.len() > 0 {
                        let gcmd = &gcmds[0];
                        //rprintln!("len:    {}", gcmds.len());
                        rprintln!("mnemonic:    {}", gcmd.mnemonic());
                        rprintln!("Major:    {}", gcmd.major_number());
                        rprintln!("minor:    {}", gcmd.minor_number());
                        rprintln!("W:    {:#?}",gcmd.value_for('W'));
                        rprintln!("X:    {:#?}",gcmd.value_for('X'));
                        rprintln!("Y:    {:#?}",gcmd.value_for('Y'));
                        rprintln!("Z:    {:#?}",gcmd.value_for('Z'));
                        rprintln!("P:    {:#?}",gcmd.value_for('P'));
                        rprintln!("S:    {:#?}",gcmd.value_for('S'));
                        rprintln!("x:    {:#?}",gcmd.value_for('x'));
                        rprintln!("y:    {:#?}",gcmd.value_for('y'));
                        rprintln!("z:    {:#?}",gcmd.value_for('z'));

                        let mnem   = gcmd.mnemonic();
                        let major  = gcmd.major_number();
                        let _minor  = gcmd.minor_number();
                        let mut _p_val:f32  = 0.0;
                        let mut _s_val:f32  = 0.0;

                        // delay introduced to hopefully remove lost answers on openpnp python scripts
                        delay.delay_ms(20_u32);

                        if mnem == Mnemonic::Miscellaneous {
                            rprintln!("######## M-Command detected ########");
                            match major {
                                W_OFFSET => {
                                    match gcmd.value_for('W'){
                                        Some(w_o) => w_offset = w_o,
                                        None => {},
                                    }
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                },
                                READ_SWITCH => {  // M489
                                    free(|cs| {
                                        let state = STATE.borrow(cs).get();
                                        match state {
                                            BleEventState::Triggered => {
                                                rprintln!("Triggered");
                                                STATE.borrow(cs).replace(BleEventState::Waiting);
                                                //buf[1] = 49; // write 1 for event
                                                trigger_answer_buf[1] = b"1"[0];
                                            },
                                            BleEventState::Waiting => {
                                                rprintln!("Waiting");
                                                //buf[1] = 48; // write 0 for no event
                                                trigger_answer_buf[1] = b"0"[0];
                                            }
                                        }
                                        // trigger_answer_buf
                                        write_buffer(&trigger_answer_buf, 7, &mut serial);
                                        let _write_offset = 0;
                                    });

                                },
                                SET_SERVO_POSITION => {  // M280
                                    // let buf: [u8;4] = [10, 111, 107, 10,];
                                    match gcmd.value_for('P') {
                                        Some(val) => _p_val = val,
                                        None => continue, // p_val = 0.0,
                                    }
                                    match gcmd.value_for('S') {
                                        Some(val) => _s_val = val,
                                        None => continue, // s_val = 0.0,
                                    }
                                    // setServo(&mut ch1, s_val);
                                    let s_val_i32 = _s_val as i32;
                                    match s_val_i32 {
                                        10 =>  {
                                                    ch1.set_duty(RELEASE_ALARM_PUSH_UP);
                                                    // wait till potential alarm is release
                                                    delay.delay_ms(200_u32);
                                                    // set automatic in wait for touch event mode
                                                    ch1.set_duty(PUSH_PIN_DOWN)
                                                    },
                                        60 =>  ch1.set_duty(RELEASE_ALARM),  // state, where waitin for touch event
                                        90 =>  ch1.set_duty(PUSH_PIN_UP),
                                        120 => ch1.set_duty(SELF_TEST),
                                        160 => ch1.set_duty(RELEASE_ALARM_PUSH_UP),
                                        _ => {},
                                    }
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                },
                                READ_FW_VERSION => {  //M115
                                    write_buffer(&firmware_answer_buf, 20, &mut serial);
                                }
                                // READ_POSITION M114
                                READ_POSITION => {  // M114 z_pos_answer_buf
                                    free(|cs|
                                        if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                                            let z_position = (z_ctrl.current_psition as f32)/ 80.0;
                                            rprintln!("M114 position {:?}", z_position);
                                            let mut buffer = ryu::Buffer::new();
                                            let z_pos_str = buffer.format(z_position);
                                            let z_len_pos_str = z_pos_str.len();
                                            let mut pos_answer_buf: [u8;20] = [0;20];
                                            for (i,c) in z_pos_str.chars().enumerate(){
                                                pos_answer_buf[i] = c as u8;
                                            }
                                            write_buffer(b"\nZ:", 3, &mut serial);
                                            write_buffer(&pos_answer_buf, z_len_pos_str, &mut serial);
                                            write_buffer(b" \nW:", 4, &mut serial);
                                            let w_position = -z_position - w_offset;
                                            let w_pos_str = buffer.format(w_position);
                                            let w_len_pos_str = w_pos_str.len();
                                            for (i,c) in w_pos_str.chars().enumerate(){
                                                pos_answer_buf[i] = c as u8;
                                            }
                                            write_buffer(&pos_answer_buf, w_len_pos_str, &mut serial);
                                        });
                                    write_buffer(b"\n", 1, &mut serial);
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                }
                                _ => {
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                },
                            }

                        }
                        if mnem == Mnemonic::General {
                            rprintln!("######## G-Command detected ########");
                            match major {
                                SET_MM_MODE => {  // G21
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                }
                                G28_HOME => {  // G28
                                    rprintln!("G28 Start homing");
                                    // set a low speed for homing
                                    free(|cs|
                                        if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                                            // set a low speed for homing
                                            z_ctrl.max_velocity = FixedI64::<U32>::from_num((5 as u32) * steps_per_mm);
                                            z_ctrl.move_state = StepperMoveState::G28Homing;
                                        });
                                }
                                G0_POSITION => {  // G0
                                    rprintln!("G0");
                                    //let z_pos =
                                    match gcmd.value_for('Z') {
                                        Some(z_pos) => {
                                            rprintln!("G0 z_pos {:?}mm {:?} steps", z_pos, (z_pos * 80.0) as i32);
                                            free(|cs|
                                                if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                                                    if z_ctrl.finished == true {
                                                        if let Some(z_max_velocity) = gcmd.value_for('F') {
                                                            // max speed is set in mm/sec
                                                            z_ctrl.max_velocity = FixedI64::<U32>::from_num((z_max_velocity as u32) * steps_per_mm);
                                                        }
                                                        //if z_pos < core::i32::MAX as u32 {
                                                        rprintln!("G0 {:?}mm {:?} steps", z_pos, (z_pos * 80.0) as i32);
                                                        z_ctrl.target_position = (z_pos * 80.0) as i32;
                                                        z_ctrl.move_state = StepperMoveState::MOVE;
                                                        start_move_motor(z_ctrl);
                                                        //}
                                                    }
                                            });
                                        },
                                        None => {},
                                    };
                                    //let w_pos =
                                    match gcmd.value_for('W') {
                                        Some(w_pos) => {
                                            rprintln!("G0 w_pos {:?}mm {:?} steps", w_pos, (w_pos * 80.0) as i32);
                                            //let w_offset = FixedI64::<U32>::from_num(0);
                                            //let w_offset = 0.0;
                                            free(|cs|
                                                if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                                                    if z_ctrl.finished == true {
                                                        if let Some(z_max_velocity) = gcmd.value_for('F') {
                                                            // max speed is set in mm/sec
                                                            z_ctrl.max_velocity = FixedI64::<U32>::from_num((z_max_velocity as u32) * steps_per_mm);
                                                        }
                                                        rprintln!("G0 {:?}mm {:?} steps", w_pos, (w_pos * 80.0) as i32);
                                                        z_ctrl.target_position = ((-w_pos * 80.0) as i32) + ((-w_offset * 80.0) as i32);
                                                        z_ctrl.move_state = StepperMoveState::MOVE;
                                                        start_move_motor(z_ctrl);
                                                    }
                                            });
                                        },
                                        None => continue, // p_val = 0.0,
                                    };
                                }
                                _ => {
                                    write_buffer(&ok_answer_buff, 4, &mut serial);
                                },
                            }
                        } else {
                            write_buffer(&ok_answer_buff, 4, &mut serial);
                        }


                    }
                }
            }

        }
    }
}


fn start_move_motor(z_ctrl:&mut Zctrl) {
    rprintln!("start_move_motor");
    z_ctrl.finished = false;
    if z_ctrl.target_position < z_ctrl.current_psition {
        z_ctrl.step_dir = INVERS_MOTOR_DIR;
        // calc an set steps to move
        z_ctrl.profile.enter_position_mode(z_ctrl.max_velocity, (z_ctrl.current_psition - z_ctrl.target_position) as u32);
    } else {
        z_ctrl.step_dir = !INVERS_MOTOR_DIR;
        // calc an set steps to move
        z_ctrl.profile.enter_position_mode(z_ctrl.max_velocity, (z_ctrl.target_position - z_ctrl.current_psition) as u32);
    }
}

fn write_buffer(buf: &[u8], data_len: usize, serial: &mut SerialPort<UsbBus<Peripheral>>) {
    rprintln!("buf.len(): {}", buf.len());
    //for el in buf
    rprintln!("Hello write_buffer!");

    let mut write_offset = 0;
    while write_offset < data_len {
        match serial.write(&buf[write_offset..data_len]) {
            Ok(len) if len > 0 => {
                       write_offset += len;
                   }
            _ => {}
        }
    }
}


#[interrupt]
fn TIM2() {
    free(|cs| {
        // static mut elapsed: bool = true;
        if let Some(ref mut tim2) = G_TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            tim2.clear_interrupt(Event::Update);
        }
        if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() { //}.deref_mut(){
            if z_ctrl.finished == false {
                // activate stepper
                z_ctrl.steper_z_en.set_low();

                // if next delay is elapsed, set high steper pin
                if z_ctrl.counts_next < FixedI64::<U32>::from_num(1) {

                    z_ctrl.steper_z_step.set_high();
                    if z_ctrl.step_dir {
                        // run to positiv direction
                        z_ctrl.steper_z_dir.set_high();
                        z_ctrl.current_psition += 1;
                    } else {
                        // run to negativ direction
                        z_ctrl.steper_z_dir.set_low();
                        z_ctrl.current_psition -= 1;
                    }
                    //z_ctrl.steper_z_step.toggle();
                    // get next delay and set z_control countdown for next pin toggle
                    if let Some(delay) = z_ctrl.profile.next_delay(){
                        z_ctrl.counts_next = (delay * 10000) + z_ctrl.counts_next;
                        if z_ctrl.counts_next <= FixedI64::<U32>::from_num(1) {
                            z_ctrl.counts_next = FixedI64::<U32>::from_num(2); // needs minimum to be 2, to set step pin low again
                            rprintln!("z_ctrl.counts_next = 1 set to 2");
                        }
                    } else {
                        // TODO is both necessary?
                        z_ctrl.finished = true;
                        z_ctrl.steper_z_en.set_high();
                        if z_ctrl.current_psition != z_ctrl.target_position {
                            rprintln!("Error after move, current and target positions differs c{:?} t{:?}",
                                            z_ctrl.current_psition, z_ctrl.target_position);
                        }
                        else {
                            z_ctrl.finished = true;
                            rprintln!("Move, finished normal c{:?} t{:?}",
                                            z_ctrl.current_psition, z_ctrl.target_position);
                        }
                    }
                } else {
                    // count down till next toggle is ready of stper pin
                    z_ctrl.counts_next = z_ctrl.counts_next - FixedI64::<U32>::from_num(1);
                    z_ctrl.steper_z_step.set_low();
                    if z_ctrl.current_psition == z_ctrl.target_position {
                        z_ctrl.finished = true;
                        rprintln!("Emove, finished early c{:?} t{:?}",
                                        z_ctrl.current_psition, z_ctrl.target_position);
                    }
                }
            }
        }
    });
}

// Endstop interrupt
#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        rprint!(":");
        let end_stop_event = unsafe {&mut *INT_Z_ENDSTOP_PIN.as_mut_ptr() };
        if end_stop_event.check_interrupt() {
            end_stop_event.clear_interrupt_pending_bit();
            rprintln!("EXTI9_5 called");
        }
        if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
            if z_ctrl.finished == false {
                z_ctrl.finished = true;
                z_ctrl.move_state = StepperMoveState::SendOk;
                // z_ctrl.steper_z_en.set_high();
                z_ctrl.current_psition = 0;
                z_ctrl.target_position = 0;
                z_ctrl.end_stop_irq = true;
            }
        }

    });
}

// blTouch interrupt, when triggered (edge high to low)
#[interrupt]
fn EXTI9_5() {
    rprintln!("EXTI15_10 called");
    free(|cs| {
        let ble_event = unsafe {&mut *INT_PIN.as_mut_ptr() };
            ble_event.clear_interrupt_pending_bit();

            unsafe {
                (*<TIM1>::ptr()).ccr1.write(|w| w.bits(PUSH_PIN_UP.into()));
            }

            let state = STATE.borrow(cs).get();
            // Run the state machine in an ISR - probably not something you want to do in most
            // cases but this one only sets the trigger state, when there is a trigger event
            // TODO and save the hight position
            match state {
                BleEventState::Waiting => {
                    STATE.borrow(cs).replace(BleEventState::Triggered);
                }
                BleEventState::Triggered => {
                    if let Some(ref mut z_ctrl) = *G_Z_CTRL.borrow(cs).borrow_mut() {
                        z_ctrl.finished = true;
                        z_ctrl.target_position = z_ctrl.current_psition;
                        z_ctrl.ble_trigger_position = z_ctrl.current_psition;
                    }
                }
            }
        //}
    });
}

// note: expected mutable reference `&mut stm32f4xx_hal::pwm::PwmChannel<TIM1, stm32f4xx_hal::pwm::C1>`
// found reference `&stm32f4xx_hal::pwm::PwmChannel<TIM, CHANNEL>`

// fn setServo<TIM1, C1>(mut ch : &PwmChannel<TIM1, C1>, s: f32) where PwmChannel<TIM1, C1>: embedded_hal::PwmPin {
//     rprintln!("setServo called:    S{}", s);
//     // servo duty-cycles for BLTouch (with 200Hz frequency = 5ms)
//     // 90° = 1473us  = 0.2946  19307:u16 (push pin up)
//     // 10° = 647us   = 0.1294   8480:u16 (pusch down)
//     // 60° = 1162us  = 0.2324  15231:u16 (alarm release)
//     // 120° = 1782us = 0.3564  23357:u16 (self test)
//     // 160° = 2194us = 0.4388  28757:u16 ( alarm releas and push pin up)
//     if s < 12.0 {
//         // ch.set_duty(PUSH_PIN_UP);
//         // <PwmChannel<TIM1, C1>>::set_duty(&mut ch, PUSH_PIN_UP);
//         let max_duty = <PwmChannel<TIM1, C1> as embedded_hal::PwmPin>::get_max_duty(&ch);
//         <PwmChannel<TIM1, C1> as PwmPin>::set_duty(&mut ch, max_duty);

//     }

// }

fn u8tostr(c: u8) -> &'static str {
    match c {
        70 => r#"F"#,
        71 => r#"G"#,
        77 => r#"M"#,
        83 => r#"S"#,
        80 => r#"P"#,
        59 => r#";"#,
        48 => r#"0"#,
        49 => r#"1"#,
        50 => r#"2"#,
        51 => r#"3"#,
        52 => r#"4"#,
        53 => r#"5"#,
        54 => r#"6"#,
        55 => r#"7"#,
        56 => r#"8"#,
        57 => r#"9"#,
        32 => r#" "#,
        13 => "\r",
        10 => "\n",
        46 => ".",
        87 => r#"W"#,
        88 => r#"X"#,
        89 => r#"Y"#,
        90 => r#"Z"#,
        120 => r#"x"#,
        121 => r#"y"#,
        122 => r#"z"#,
        61 => r#"="#,
        45 => r#"-"#,
        _ => r#""#,
    }
}
