
#include <avr\io.h>
#include <avr\pgmspace.h>
#include <stdio.h>
#include "os_api.h"
#include "avr\eeprom.h"


char * strtrim( char * str )
{
   uint8_t i;
   for ( i=0; str[i] == ' '; i++);
   return &str[i];
}


#ifdef OS_AVRX

#include "avrx_systemtimer0.h"

Mutex EEPromMutex;

// Initialize AvrX
void AvrXInit()
{
   AvrXSetKernelStack(0);

   AvrXInitSystemTimer0();   // Configure and start system timer
   /* Needed for EEPROM access */
   AvrXSetSemaphore(&EEPromMutex);
}

void AvrXWriteEEPromWord(unsigned int * address, unsigned int value)
{
   OSSemaTakeEver( EEPromMutex);
   eeprom_write_word(address, value);
   OSSemaGive( EEPromMutex );
}

unsigned char AvrXReadEEProm( unsigned char * address )
{
	unsigned char data;

	OSSemaTakeEver( EEPromMutex);
	data = eeprom_read_byte( address );
	OSSemaGive( EEPromMutex );
	return data;
}

unsigned int AvrXReadEEPromWord( unsigned int * address )
{
	unsigned int data;

	OSSemaTakeEver( EEPromMutex);
	data = eeprom_read_word( address );
	OSSemaGive( EEPromMutex );
	return data;
}

#else

#include <avr/eeprom.h>

xSemaphoreHandle EEPromMutex;


void FreeRTOSInit()
{
    EEPromMutex = xSemaphoreCreateMutex();
    xSemaphoreGive( EEPromMutex );
}

unsigned char FreeRTOSReadEEPromByte( unsigned char * address )
{
   unsigned char data;

   OSSemaTakeEver( EEPromMutex );
   data = eeprom_read_byte( address );
   OSSemaGive( EEPromMutex );
   return data;
}

unsigned int FreeRTOSReadEEPromWord( unsigned int * address )
{
   unsigned int data;

   OSSemaTakeEver( EEPromMutex );
   data = eeprom_read_word( address );
   OSSemaGive( EEPromMutex );
   return data;
}


void FreeRTOSWriteEEPromWord(unsigned int * address, unsigned int value)
{
   OSSemaTakeEver( EEPromMutex );
   eeprom_busy_wait();
   eeprom_write_word(address, value);
   eeprom_busy_wait();
   OSSemaGive( EEPromMutex );
}

void FreeRTOSReadEEPromBlock(uint8_t * address, uint8_t *value, uint8_t size)
{
	OSSemaTakeEver( EEPromMutex );
	eeprom_read_block( value, address, size );
	OSSemaGive( EEPromMutex );
}


void FreeRTOSWriteEEPromBlock(uint8_t * address, uint8_t *value, uint8_t size)
{
	OSSemaTakeEver( EEPromMutex );
	eeprom_busy_wait();
	eeprom_write_block(value, address, size);
	OSSemaGive( EEPromMutex );
}


extern void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
	//PORTA_OUTSET = _BV(5);
}


#endif

