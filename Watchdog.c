#include "Watchdog.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>

// Initialize Watchdog Timer with given timeout period
void watchdog_init(uint8_t timeout_period) 
{
    // Disable interrupts globally
    cli();

    // Wait until previous watchdog configuration is cleared
    while (WDT.STATUS & WDT_SYNCBUSY_bm);

    // Set new timeout period but don't enable it yet
	// Unlock and configure
	CCP = CCP_IOREG_gc;
    WDT.CTRL = (timeout_period & WDT_PER_gm);

    // Enable interrupts globally
    sei();
}

// Enable Watchdog (with already set timeout)
void watchdog_enable(void) 
{
    cli();
    while (WDT.STATUS & WDT_SYNCBUSY_bm);

    // OR the ENABLE bit to start watchdog
	// Unlock and configure
	CCP = CCP_IOREG_gc;
    WDT.CTRL |= WDT_ENABLE_bm;

    sei();
}

// Disable Watchdog (only if fuse WDLOCK is not set!)
void watchdog_disable(void) 
{
    cli();
    while (WDT.STATUS & WDT_SYNCBUSY_bm);

	// Unlock and configure
	CCP = CCP_IOREG_gc;
    WDT.CTRL = 0;

    sei();
}

// Reset (feed/kick) the watchdog
void watchdog_reset(void) 
{
    asm volatile("wdr");
}

