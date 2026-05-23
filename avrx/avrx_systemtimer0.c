#include <avr/io.h>			// This defers to avr/io.h for GCC
#include <avr/interrupt.h>		// This defers to avr/signal.h for GCC
#include "avrx/Avrx.h"
#include "avrx_systemtimer0.h"

#define CPUCLK F_CPU     // CPU xtal
#define TICKRATE 1000       // AvrX timer queue Tick rate


// Peripheral initialization

#define TCNT0_INIT (0xFF-CPUCLK/256/TICKRATE)

// NB: Clock divisor bits are different between classic and mega103 chips!
// NB: IAR chooses to use different CPU identifiers for their C compiler
// NB: GCC changes the IO bit names just for fun...

#if defined (__AVR_ATmega103__) || defined (__ATmega103__)
#	define TMC8_CK256 ((1<<CS02) | (1<<CS01))
#elif defined (__AVR_ATmega128__) || defined (__ATmega128__)
#	define TMC8_CK256 ((1<<CS2) | (1<<CS1))
#else	// Most other AVR processors
#	define TMC8_CK256 (1<<CS02)
#endif

/*
 Timer 0 Overflow Interrupt Handler

 Prototypical Interrupt handler:
 . Switch to kernel context
 . handle interrupt
 . switch back to interrupted context.
 */
//#pragma optimize=z 4
///AVRX_SIGINT(SIG_OVERFLOW0)
#ifdef OS_AVRX
AVRX_SIGINT(RTC_OVF_vect)
{
    IntProlog();                // Switch to kernel stack/context
    AvrXTimerHandler();         // Call Time queue manager
    Epilog();                   // Return to tasks
}


void AvrXInitSystemTimer0(void)                 // Main runs under the AvrX Stack
{
	while(RTC.STATUS & RTC_SYNCBUSY_bm);
	
	RTC.PER = 1024;
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC_INTCTRL = RTC_OVFINTLVL_LO_gc;
}

#endif