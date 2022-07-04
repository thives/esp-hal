#![no_std]
#![no_main]

use core::sync::atomic::{AtomicI32, Ordering};

use esp32_hal::{
    clock::ClockControl,
    pac::{Peripherals, TIMG1},
    prelude::*,
    CpuControl,
    RtcCntl,
    Timer,
};
use esp_println::println;
use nb::block;
use panic_halt as _;
use xtensa_lx::mutex::Mutex;
use xtensa_lx_rt::entry;

#[entry]
fn main() -> ! {
    _main();
}

fn _main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut timer0 = Timer::new(peripherals.TIMG0, clocks.apb_clock);
    let mut timer1 = Timer::new(peripherals.TIMG1, clocks.apb_clock);
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    timer0.disable();
    timer1.disable();
    rtc_cntl.set_wdt_global_enable(false);

    timer0.start(1u64.secs());
    timer1.start(500u64.millis());

    let counter = xtensa_lx::mutex::SpinLockMutex::new(AtomicI32::new(0));

    let mut cpu_control = CpuControl::new(system.cpu_control);
    let mut cpu1_fnctn = || {
        cpu1_task(&mut timer1, &counter);
    };
    let _guard = cpu_control.start_app_core(&mut cpu1_fnctn).unwrap();

    loop {
        block!(timer0.wait()).unwrap();

        let count = (&counter).lock(|counter| counter.load(Ordering::Relaxed));
        println!("Hello World - Core 0! Counter is {}", count);
    }
}

fn cpu1_task(timer: &mut Timer<TIMG1>, counter: &xtensa_lx::mutex::SpinLockMutex<AtomicI32>) -> ! {
    println!("Hello World - Core 1!");
    loop {
        block!(timer.wait()).unwrap();

        (&*counter).lock(|counter| {
            counter.store(
                counter.load(Ordering::Relaxed).wrapping_add(1),
                Ordering::Relaxed,
            );
        });
    }
}
