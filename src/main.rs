//! Horse
//!
//! Input: move detection -> d7 (interrupt on change)
//! Output: d5 -> MOSFET (PWM via Timer0)
//! Interrupt: on change of d7

/*
* Timer0: 5,6
* Timer1: 9,10
* Timer2: 3,11
*
* PWM pins: 3, 5, 6, 9, 10, 11
*
* Port A: a0..a5
* Port B: d8..d13
* Port C: d0..d7
*/

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::sync::atomic::{AtomicBool, Ordering};

use arduino_hal::{delay_ms, Peripherals, simple_pwm::*};
use avr_device::interrupt;
use panic_halt as _;

static mut FLAG: u8 = 0;
static MOVED: AtomicBool = AtomicBool::new(true);
static CHECKING: AtomicBool = AtomicBool::new(false);

#[arduino_hal::entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // TC0 controls PWM of d5 pin. 
    // Arduino will be connected to MOSFET chip via d5 pin.
    //
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale1024);
    let mut mosfet = pins.d5.into_output().into_pwm(&timer0);
    mosfet.enable();

    // Adrduino will be connected to move detection chip via d7 pin.
    // Any move detection causes an interrupt.
    // 
    dp.EXINT.pcicr.write(|w|
        // enable PCINT2 (Port D) pin change interrupt
        unsafe { w.bits(0b100) }
    );
    dp.EXINT.pcmsk2.write(|w|
        // enable pin change interrupt on PCINT23 (d7)
        w.bits(1 << 7)
    );

    // Enable interrupts globally.
    // SAFETY: we are not inside a critical section.
    unsafe { interrupt::enable() };

    loop {
        if MOVED.load(Ordering::Acquire) {
            MOVED.store(false, Ordering::Release);

            for val in (0..=255).rev().step_by(16) {
                mosfet.set_duty(val);
                delay_ms(10);
            }

            for _ in 0..2 {
                delay_ms(1_000);
                let pattern = (0..=255).step_by(32).chain((0..=255).rev().step_by(64));
                for val in pattern.clone() {
                    mosfet.set_duty(val);
                    delay_ms(10);
                }
                delay_ms(200);
                for val in pattern {
                    mosfet.set_duty(val);
                    delay_ms(10);
                }
            }

            // wait for _10 secs before next iteration
            delay_ms(10_000);
            CHECKING.store(true, Ordering::SeqCst);
        }
    }
}

#[interrupt(atmega328p)]
fn PCINT2() {
    if CHECKING.load(Ordering::SeqCst) {
        // Prevent from MOVE is being set twice.
        // SAFETY: access to FLAG happens only inside the current function.
        unsafe {
            FLAG = 1 - FLAG;
            if FLAG == 0 {
                return;
            }
        }

        MOVED.store(true, Ordering::SeqCst);
    }
}
