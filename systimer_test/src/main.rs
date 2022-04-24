#![no_std]
#![no_main]

use core::{cell::RefCell, fmt::Write};

use bare_metal::Mutex;
pub use esp32c3::{Interrupt, INTERRUPT_CORE0, SYSTIMER};
use esp32c3_hal::{
    pac::{self, Peripherals, RTC_CNTL, TIMG0, TIMG1, UART0},
    prelude::*,
    RtcCntl, Serial, Timer,
};
use esp_hal_common::{
    interrupt::{self, CpuInterrupt, InterruptKind},
    Cpu,
};
use panic_halt as _;
use riscv_rt::entry;

mod ESP32C3_Interrupts {
    pub use super::*;
    pub fn enable(interrupt_number: isize, cpu_interrupt_number: CpuInterrupt) {
        unsafe {
            let cpu_interrupt_number = cpu_interrupt_number as isize;
            let intr = &*INTERRUPT_CORE0::ptr();
            let intr_map_base = intr.mac_intr_map.as_ptr();
            intr_map_base
                .offset(interrupt_number)
                .write_volatile(cpu_interrupt_number as u32);

            // enable interrupt
            intr.cpu_int_enable
                .modify(|r, w| w.bits((1 << cpu_interrupt_number) | r.bits()));
        }
    }
    pub fn disable(interrupt: isize) {
        unsafe {
            let interrupt_number = interrupt as isize;
            let intr = &*INTERRUPT_CORE0::ptr();
            let intr_map_base = intr.mac_intr_map.as_ptr();
            intr_map_base.offset(interrupt_number).write_volatile(0);
        }
    }
    pub fn set_kind(which: CpuInterrupt, kind: InterruptKind) {
        unsafe {
            let intr = &*INTERRUPT_CORE0::ptr();
            let cpu_interrupt_number = which as isize;

            let interrupt_type = match kind {
                InterruptKind::Level => 0,
                InterruptKind::Edge => 1,
            };
            intr.cpu_int_type.modify(|r, w| {
                w.bits(
                    r.bits() & !(1 << cpu_interrupt_number)
                        | (interrupt_type << cpu_interrupt_number),
                )
            });
        }
    }
    pub fn set_priority(which: CpuInterrupt, priority: u32) {
        unsafe {
            let intr = &*INTERRUPT_CORE0::ptr();
            let cpu_interrupt_number = which as isize;
            let intr_prio_base = intr.cpu_int_pri_0.as_ptr();

            intr_prio_base
                .offset(cpu_interrupt_number as isize)
                .write_volatile(priority as u32);
        }
    }
    pub fn clear(which: CpuInterrupt) {
        unsafe {
            let cpu_interrupt_number = which as isize;
            let intr = &*INTERRUPT_CORE0::ptr();
            intr.cpu_int_clear
                .write(|w| w.bits(1 << cpu_interrupt_number));
        }
    }
    //TODO implement testing for these functions
    pub fn is_enabled(cpu_interrupt_number: CpuInterrupt) -> bool {
        unsafe {
            let intr = &*INTERRUPT_CORE0::ptr();
            let b = intr.cpu_int_enable.read().bits();
            let ans = b & (1 << cpu_interrupt_number as isize);
            ans != 0
        }
    }
    pub fn is_pending(cpu_interrupt_number: CpuInterrupt) -> bool {
        unsafe {
            let intr = &*INTERRUPT_CORE0::ptr();
            let b = intr.cpu_int_eip_status.read().bits();
            let ans = b & (1 << cpu_interrupt_number as isize);
            ans != 0
        }
    }
    pub fn get_priority(which: CpuInterrupt) -> u32 {
        unsafe {
            let intr = &*INTERRUPT_CORE0::ptr();
            let cpu_interrupt_number = which as isize;
            let intr_prio_base = intr.cpu_int_pri_0.as_ptr();

            let x = intr_prio_base
                .offset(cpu_interrupt_number as isize)
                .read_volatile();
            return x;
        }
    }
}

static mut SERIAL: Mutex<RefCell<Option<Serial<UART0>>>> = Mutex::new(RefCell::new(None));
static mut TIMER0: Mutex<RefCell<Option<Timer<TIMG0>>>> = Mutex::new(RefCell::new(None));

#[inline(always)]
fn get_time_test() -> u64 {
    unsafe {
        let systimer = &*SYSTIMER::ptr();
        systimer
            .unit0_op
            .write(|w| w.timer_unit0_update().set_bit());

        while !systimer
            .unit0_op
            .read()
            .timer_unit0_value_valid()
            .bit_is_set()
        {}

        let value_lo = systimer.unit0_value_lo.read().bits();
        let value_hi = systimer.unit0_value_hi.read().bits();

        ((value_hi as u64) << 32) | value_lo as u64
    }
}

const CLK_FREQ_HZ: u64 = 16_000_000;
unsafe fn set_one_time_alarm(delay_micros: u64, serial: &mut Serial<UART0>) {
    let micros_to_delay = delay_micros; //(delay_micros * CLK_FREQ_HZ) / 1_000_000;
    let systimer = &*SYSTIMER::ptr();

    systimer.target0_conf.write(|w| {
        // /1. Set SYSTIMER_TARGETx_TIMER_UNIT_SEL to select the counter (UNIT0 or UNIT1) used for COMPx.
        // w.target0_timer_unit_sel().set_bit().
        let w1 = w.target0_timer_unit_sel().clear_bit();
        //3. Clear SYSTIMER_TARGETx_PERIOD_MODE to enable target mode.
        let w2 = w1.target0_period_mode().clear_bit();
        w2
    });
    writeln!(serial, "step 1").ok();
    //2. Read current count value, see Section 10.5.1. This value will be used to calculate the alarm value (t) in Step
    let current_time = get_time_test();
    let target_time = current_time + micros_to_delay;
    writeln!(serial, "2.").ok();
    //4. Set an alarm value (t), and fill its lower 32 bits to SYSTIMER_TIMER_TARGETx_LO, and the higher 20 bits to hi
    writeln!(serial, "current time: {}", &current_time);
    writeln!(serial, "target time:  {}", &target_time);
    systimer
        .target0_hi
        .write(|w| w.timer_target0_hi().bits((target_time >> 32) as u32));
    systimer.target0_lo.write(|w| {
        w.timer_target0_lo()
            .bits((target_time & 0xFFFF_FFFF) as u32)
    });
    writeln!(serial, "4.").ok();
    //5. Set SYSTIMER_TIMER_COMPx_LOAD to synchronize the alarm value (t) to COMPx, i.e. load the alarm
    systimer
        .comp0_load
        .write(|w| w.timer_comp0_load().set_bit());
    writeln!(serial, "5.").ok();
    //6. Set SYSTIMER_TARGETx_WORK_EN to enable the selected COMPx. COMPx starts comparing the count
    systimer.conf.write(|w| {
        w.target0_work_en()
            .set_bit()
            .timer_unit0_core0_stall_en()
            .clear_bit()
    });
    writeln!(serial, "6.").ok();
    // 7. Set SYSTIMER_TARGETx_INT_ENA to enable timer interrupt. When Unitn counts to the alarm value (t), a
    // SYSTIMER_TARGETx_INT interrupt is triggered
    systimer.int_ena.write(|w| w.target0_int_ena().set_bit());
    writeln!(serial, "7.").ok();
}

unsafe fn unset_systimer1_interrupt_source(){
    let systimer = &*SYSTIMER::ptr();
    //stop comparing values to interrupt so no new interrupt is triggerred
    systimer.int_clr.write(|w| w.target0_int_clr().set_bit());
    // systimer.conf.write(|w| w.target0_work_en().clear_bit());

}

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut timer0 = Timer::new(peripherals.TIMG0);

    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();

    ESP32C3_Interrupts::enable(Interrupt::TG0_T0_LEVEL as isize, CpuInterrupt::Interrupt1);

    ESP32C3_Interrupts::set_priority(
        CpuInterrupt::Interrupt1,
        interrupt::Priority::Priority1 as u32,
    );
    ESP32C3_Interrupts::set_kind(CpuInterrupt::Interrupt1, InterruptKind::Level);

    ESP32C3_Interrupts::enable(
        Interrupt::SYSTIMER_TARGET0 as isize,
        CpuInterrupt::Interrupt5,
    );
    ESP32C3_Interrupts::set_priority(
        CpuInterrupt::Interrupt5,
        interrupt::Priority::Priority3 as u32,
    );
    ESP32C3_Interrupts::set_kind(CpuInterrupt::Interrupt5, InterruptKind::Level);
    timer0.start(30_000_000u64);
    timer0.listen();

    // timer1.start(20_000_000u64);
    // timer1.listen();

    riscv::interrupt::free(|_cs| unsafe {
        SERIAL.get_mut().replace(Some(serial0));
        TIMER0.get_mut().replace(Some(timer0));
        // TIMER1.get_mut().replace(Some(timer1));
    });

    unsafe {
        riscv::interrupt::enable();
        riscv::interrupt::free(|cs| unsafe {
            let mut serial = SERIAL.borrow(*cs).borrow_mut();
            let mut serial = serial.as_mut().unwrap();
            set_one_time_alarm(30_000_000u64, &mut serial);
            writeln!(serial, "looping...").ok();
        })
    }


    loop {
        unsafe {
            riscv::interrupt::free(|cs| unsafe {
                let mut serial = SERIAL.borrow(*cs).borrow_mut();
                let mut serial = serial.as_mut().unwrap();
                writeln!(serial, "looping...").ok();
            });
            riscv::asm::wfi();
        }
        riscv::interrupt::free(|cs| unsafe {
            let time = get_time_test();
            let mut serial = SERIAL.borrow(*cs).borrow_mut();
            let serial = serial.as_mut().unwrap();
            writeln!(serial, "Time: {}", &time).ok();
        });
    }
}

#[no_mangle]
pub fn interrupt1() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        let time = get_time_test();
        writeln!(serial, "Time: {}", &time).ok();

        let mut timer0 = TIMER0.borrow(*cs).borrow_mut();
        let timer0 = timer0.as_mut().unwrap();

        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt1);
        timer0.clear_interrupt();

        timer0.start(30_000_000u64);
    });
}

#[no_mangle]
pub fn interrupt5() {
    riscv::interrupt::free(|cs| unsafe {
        let mut serial = SERIAL.borrow(*cs).borrow_mut();
        let serial = serial.as_mut().unwrap();
        writeln!(serial, "Interrupt 5").ok();

        // let mut timer1 = TIMER1.borrow(*cs).borrow_mut();
        // let timer1 = timer1.as_mut().unwrap();
        unset_systimer1_interrupt_source();
        interrupt::clear(Cpu::ProCpu, interrupt::CpuInterrupt::Interrupt5);
        
        // timer1.clear_interrupt();

        // timer1.start(20_000_000u64);
    });
}
