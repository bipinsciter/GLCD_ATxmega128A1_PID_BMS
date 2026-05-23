#include <stdio.h>
#include <stdlib.h>

// Initialize Watchdog Timer with given timeout period
void watchdog_init(uint8_t timeout_period);

// Enable Watchdog (with already set timeout)
void watchdog_enable(void);

// Disable Watchdog (only if fuse WDLOCK is not set!)
void watchdog_disable(void);

// Reset (feed/kick) the watchdog
void watchdog_reset(void);
