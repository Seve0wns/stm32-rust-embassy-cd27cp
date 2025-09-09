#![no_std]
#![no_main]

use cortex_m_rt::pre_init;
use embassy_stm32::{peripherals, Peri};
use embassy_sync::pubsub::PubSubChannel;
use core::arch::asm;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_stm32::gpio::{Output, Pull, Level, Speed, OutputType};
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::timer::simple_pwm::{PwmPin,SimplePwm};
use embassy_stm32::time::hz;
use embassy_stm32::exti::ExtiInput;
use embassy_time::{Duration, Timer};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use heapless::String;
use core::fmt::Write;
use embassy_stm32::adc::Adc;
use {defmt_rtt as _, panic_probe as _};

static DUTY_CYCLE:AtomicU8=AtomicU8::new(0);
static SPIN_CLOCKWISE:AtomicBool=AtomicBool::new(true);

static BUTTON_CHANNEL:Channel<CriticalSectionRawMutex,&'static str,4>=Channel::new();
static DC_CHANNEL:PubSubChannel<CriticalSectionRawMutex,u8,4,2,1>=PubSubChannel::new();
static SD_CHANNEL:PubSubChannel<CriticalSectionRawMutex,&'static str,4,2,1>=PubSubChannel::new();

macro_rules! get {
    ($a:expr) => {
        $a.load(Ordering::Relaxed)
    };
}
macro_rules! post {
    ($a:expr,$b:expr) => {
        $a.store($b,Ordering::Relaxed)
    };
}
fn get_number(str: &[u8]) -> i8 {
    info!("get_number received {}",str);
    // Check for empty input
    if str.is_empty() {
        return -1;
    }
    // Trim leading/trailing whitespace
    let start = str.iter().position(|&b| b != b' ' && b != b'\t').unwrap_or(str.len());
    if start >= str.len() {
        return -1; // Only whitespace
    }
    
    let end = str.iter().rposition(|&b| b != b' ' && b != b'\t' && b != b'\r' && b != b'\n')
    .map(|i| i + 1)
    .unwrap_or(str.len());

if start >= end {
    return -1;
    }
    
    let trimmed = &str[start..end];
    
    if let Ok(number_str) = core::str::from_utf8(trimmed) {
        if let Ok(duty_value) = number_str.parse::<u8>() {
            if duty_value <= 100 {
                return duty_value as i8;
            }
        }
    }
    return -1;
}
fn process_command(line:&[u8],out:&mut String<512>)->i8{
    out.clear();
    let pos=line.iter().position(|&b| b==b' ').unwrap_or(line.len());
    let command=&line[..pos];
    if command==b"dutycycle"{
        if (pos+1)<line.len(){
        let dc=get_number(&line[pos+1..]);
        info!("duty value: {}",dc);
        if dc!=-1{
                return dc;
            }
            }
            out.push_str("invalid duty cycle value\r\n").ok();
    }
    else if command==b"status"{
        core::write!(out,"spin: {}\r\nduty cycle: {}%\r\n",if get!(SPIN_CLOCKWISE) {"clockwise"} else {"counterclockwise"},get!(DUTY_CYCLE   )).unwrap();
    }
    else if command==b"help"{
        out.push_str("====== Command List ======\r\ndutycycle <number> - Change the duty cycle of the motor's PWM\r\nstatus - Show the current spin direction and duty cycle of the motor\r\nhelp - Show this help guide\r\ntasks - Show list of tasks installed\r\n").ok();
    }
    else if command==b"tasks"{
        info!("entered in tasks command");
        out.push_str("====== Task List ======\r\nbutton_task - Deals with the press of the button\r\npwm_task1 - Manages the pwm for the clock spin of the motor\r\npwm_task2 - Manages the pwm for the counterclock spin of the motor\r\nuart_task - Sends and receives serial messages via UART\r\n").ok();
    }
    else{
        out.push_str("Unknown command\r\n").ok();
    }
    return -1;
}
// Declare async tasks
#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    //Task to deal with button press, changes the spin direction of the motor
    let publisher=SD_CHANNEL.publisher().unwrap();
    loop {
        button.wait_for_rising_edge().await;
        button.wait_for_falling_edge().await;
        BUTTON_CHANNEL.send("Button pressed!\n\r").await;
        publisher.publish("change").await;
        let current=get!(SPIN_CLOCKWISE);
        post!(SPIN_CLOCKWISE,!current);
    }
}

//Declare async tasks
#[embassy_executor::task]
async fn pwm_task1(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM3>) {
    //Task to generate the pwm to move the motor close-wise, 1khz on TIM3 CH1, PA6
    let mut sd_handler=SD_CHANNEL.subscriber().unwrap();
    let mut dc_handler=DC_CHANNEL.subscriber().unwrap();
    let mut is_active=true;

    pwm.set_frequency(hz(1000));
    
    let mut ch1=pwm.ch1();
    ch1.set_duty_cycle_percent(0);
    
    // Enable PWM
    ch1.enable();
    
    loop {
        match embassy_futures::select::select(
            sd_handler.next_message(),
            dc_handler.next_message()
        ).await {
            embassy_futures::select::Either::First(_) => {
        is_active=!is_active;
        if is_active{            
            Timer::after_millis(1000).await;
            ch1.set_duty_cycle_percent(get!(DUTY_CYCLE));
        }
        else{
            ch1.set_duty_cycle_fully_off();
        }
    }
    embassy_futures::select::Either::Second(dc) => {
        if let embassy_sync::pubsub::WaitResult::Message(duty_cycle) = dc {
            if is_active {
                info!("setting duty cycle to {}",duty_cycle);
                ch1.set_duty_cycle_percent(duty_cycle);
            }
        }
    }
    }
}
}

//Declare async tasks
#[embassy_executor::task]
async fn pwm_task2(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM5>) {
    //Task to generate the pwm to move the motor counterclock-wise, 1khz on TIM5 CH1, PA0     
    let mut sd_handler=SD_CHANNEL.subscriber().unwrap();
    let mut dc_handler=DC_CHANNEL.subscriber().unwrap();
    let mut is_active=false;  
    pwm.set_frequency(hz(1000));
    let mut ch1=pwm.ch1();
    ch1.set_duty_cycle_percent(0);
    
    // Enable PWM
    ch1.enable();
    
    loop {
        match embassy_futures::select::select(
            sd_handler.next_message(),
            dc_handler.next_message()
        ).await {
            embassy_futures::select::Either::First(_) => {
        is_active=!is_active;
        if is_active{            
            Timer::after_millis(1000).await;
            ch1.set_duty_cycle_percent(get!(DUTY_CYCLE));
        }
        else{
            ch1.set_duty_cycle_fully_off();
        }
    }
    embassy_futures::select::Either::Second(dc) => {
        if let embassy_sync::pubsub::WaitResult::Message(duty_cycle) = dc {
            if is_active {
                info!("setting duty cycle to {}",duty_cycle);
                ch1.set_duty_cycle_percent(duty_cycle);
            }
        }
    }
    }
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn uart_task(mut lpuart: Uart<'static, embassy_stm32::mode::Async>) {
    lpuart.write("UART started, type help to see command list...\r\n".as_bytes()).await.unwrap();
    let mut read_buffer = [0u8; 1];     // Single byte for reading
    let mut line_buffer = [0u8; 128];   // Line accumulation buffer
    let mut line_pos = 0;               // Current position in line buffer
    
    let publisher=DC_CHANNEL.publisher().unwrap();
    let mut output:String<512>=String::new();
    let header=b"<control>:";
    lpuart.write(header).await.unwrap();
    let mut rt:i8;
    
    // Loop to handle both UART echo and button messages
    loop {
        // Use select to handle both UART input and button messages
        match embassy_futures::select::select(
            lpuart.read(&mut read_buffer),
            BUTTON_CHANNEL.receive()
        ).await {
            embassy_futures::select::Either::First(_) => {
                // Handle UART input
                let byte = read_buffer[0];
                
                // Check for Enter key (carriage return \r or newline \n)
                if byte == b'\r' || byte == b'\n' {
                    output.clear();
                    // Enter was pressed - process the complete line
                    if line_pos > 0 {
                        // Echo newline
                        lpuart.write(b"\r\n").await.unwrap();
                        
                        // Process the complete line here
                        let line = &line_buffer[..line_pos];
                        rt=process_command(line, &mut output);
                        if rt!=-1{
                            post!(DUTY_CYCLE,rt as u8);
                            publisher.publish(rt as u8).await;
                        }
                        if !output.is_empty(){
                            lpuart.write(output.as_bytes()).await.unwrap();
                        }
                        lpuart.write(header).await.unwrap();
                                                
                        // Reset line buffer for next line
                        line_pos = 0;
                    } else {
                        // Just echo newline if empty line
                        lpuart.write(b"\r\n").await.unwrap();
                    }
                } else if byte == 8 || byte == 127 { // Backspace or DEL
                    if line_pos > 0 {
                        line_pos -= 1;
                        // Echo backspace sequence to terminal
                        lpuart.write(b"\x08 \x08").await.unwrap();
                    }
                } else if byte >= 32 && byte <= 126 { // Printable ASCII characters
                    // Add character to line buffer if there's space
                    if line_pos < line_buffer.len() {
                        line_buffer[line_pos] = byte;
                        line_pos += 1;
                        
                        // Echo the character
                        lpuart.write(&read_buffer).await.unwrap();
                    }
                }
                // Ignore other control characters
            }
            embassy_futures::select::Either::Second(msg) => {
                // Handle button messages
                lpuart.write(msg.as_bytes()).await.unwrap();
                lpuart.write(b"\r\n").await.unwrap();
                lpuart.write(header).await.unwrap();
                line_pos=0;
            }
        }
    }
}

//adc task
#[embassy_executor::task]
async fn adc_task(mut adc:Adc<'static,peripherals::ADC1>,mut pin:Peri<'static,peripherals::PC0>){
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES144);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS12);

    loop{
        let raw=adc.blocking_read(&mut pin);
        let v=(raw as u32*3300)/4095;
        info!("PC1: {}",v);
        Timer::after(Duration::from_hz(60000)).await;
    }
}
bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

#[pre_init]
unsafe fn before_main() {
    unsafe {
        asm!{
            "ldr r0, =__sccmdata
            ldr r1, =__eccmdata
            ldr r2, =__siccmdata
            0:
            cmp r1, r0
            beq 1f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 0b
            1:"
        }

        asm!{
            "ldr r0, =__sdata2
            ldr r1, =__edata2
            ldr r2, =__sidata2
            2:
            cmp r1, r0
            beq 3f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 2b
            3:"
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });

        config.rcc.pll_src=PllSource::HSE;

        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV7),
            divr: None,
        });

        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
    }

    let p: embassy_stm32::Peripherals = embassy_stm32::init(config);

    //Creation of the button task
    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);
    spawner.spawn(button_task(button)).unwrap();

    //Creation of the UART task
    let mut usart_config = usart::Config::default();
    usart_config.baudrate = 115_200;
    let uart = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA2_CH7, p.DMA2_CH5, usart_config).unwrap();
spawner.spawn(uart_task(uart)).unwrap();

    //Creation of the first PWM task
    let pwm_channel=PwmPin::new(p.PA6,OutputType::PushPull);
    let pwm1 = SimplePwm::new(p.TIM3,
        Some(pwm_channel),
        None,
        None,
        None,
        hz(1000),
        Default::default());
    spawner.spawn(pwm_task1(pwm1)).unwrap();

    //Creation of the second PWM task
    let pwm_channel2=PwmPin::new(p.PA0,OutputType::PushPull);
    let pwm2 = SimplePwm::new(p.TIM5,
     Some(pwm_channel2),
     None,
     None,
     None,
     hz(1000),
     Default::default());
    spawner.spawn(pwm_task2(pwm2)).unwrap();

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    //Creation of adc task
    spawner.spawn(adc_task(Adc::new(p.ADC1),p.PC0.into())).unwrap();

    loop {
        led.set_high();
        Timer::after_millis(100).await;

        led.set_low();
        Timer::after_millis(100).await;
    }
}