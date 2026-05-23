/*
	AvrXBufferedSerial.c

	Sample code for fully buffered interrupt driven serial I/O for the
	AVR processor.  Uses the AvrXFifo facility.

	Author: Larry Barello (larry@barello.net)

	Revision History:
	09-13-2005	- Initial version

*/

//------------------------------------------------------------------------------
#ifdef OS_AVRX

#include <avr/io.h>
#include <avr/interrupt.h>
#define _AVRXSERIALIO_C_
#include "AvrXSerialIo.h"
#include "UserInterface.h"

#if USART_CHANNELS & CHANNEL_0

AVRX_DECL_FIFO(Rx0Buf, RX0_BUFSZ);
AVRX_DECL_FIFO(Tx0Buf, TX0_BUFSZ);

int put_c0(char c)	// Non blocking output
{
	int retc;
	retc = AvrXPutFifo(Tx0Buf, c);
	USARTC0_CTRLA |= (USART_DREINTLVL_HI_gc);
	return retc;
}

int put_char0( char c)	// Blocking output
{
	AvrXWaitPutFifo(Tx0Buf, c);
	USARTC0_CTRLA |= (USART_DREINTLVL_HI_gc);
	return 0;
}

int get_c0(void)	// Non blocking, return status outside of char range
{
	int retc = AvrXPullFifo(Rx0Buf);
	return retc;
}

int get_char0(void)	// Blocks waiting for something
{
	return AvrXWaitPullFifo(Rx0Buf);
}
// We don't care if the buffer is full.  Just signal we got one.
// The task may attempt one extra time to get data out of an empyt
// buffer (getc(), but so what.  Eventually it will block waiting
// for another character to be received.

///#if defined(SIG_UART_RECV) && !defined(SIG_UART0_RECV)
///#  define SIG_UART0_RECV SIG_UART_RECV		// This covers old single UART chips
///#  define SIG_UART0_DATA SIG_UART_DATA
///#endif

AVRX_SIGINT(USARTC0_RXC_vect)
{
    IntProlog();
    USARTC0_CTRLA &= ~(USART_RXCINTLVL_gm);	// Disable Rx interrupt
    sei();					// Allow other interrupt activity to occur
	AvrXPutFifo(Rx0Buf, USARTC0_DATA);// This resets the Rx Interrupt
	USARTC0_CTRLA |= (USART_RXCINTLVL_HI_gc);	// Re-enable.
	Epilog();
}

AVRX_SIGINT(USARTC0_DRE_vect)
{
    IntProlog();
    USARTC0_CTRLA &= ~(USART_DREINTLVL_gm);			// Disable UDRE interrupt
    sei();							// Allow other stuff to happen
	int c = AvrXPullFifo(Tx0Buf);	// Return -1 if empty (enables interrupts)
	if (c >= 0)						// Tricky tight code: only checking sign
	{
		//if( disableTx_0 ) USARTC1_CTRLA |= (USART_TXCINTLVL_HI_gc);
		//if (enableTx_0)	(*enableTx_0)();
		
		USARTC0_DATA = c;
		USARTC0_CTRLA |= (USART_DREINTLVL_HI_gc);
	}
	Epilog();
}

AVRX_SIGINT(USARTC0_TXC_vect)
{
	IntProlog();
	//(*disableTx_0)();
	USARTC0_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UDRE interrupt
	Epilog();
}

#endif	// USART_CHANNELS & CHANNEL_0
//------------------------------------------------------------------------
#if USART_CHANNELS & CHANNEL_1

AVRX_DECL_FIFO(Rx1Buf, RX1_BUFSZ);
AVRX_DECL_FIFO(Tx1Buf, TX1_BUFSZ);

//static void (*enableTx1)();
//static void (*disableTx1)();


int put_c1(char c)	// Non blocking output
{
	int retc;
	retc = AvrXPutFifo(Tx1Buf, c);
	USARTC1_CTRLA |= (USART_DREINTLVL_HI_gc);
	return retc;
}

int put_char1( char c)	// Blocking output
{
	AvrXWaitPutFifo(Tx1Buf, c);
	USARTC1_CTRLA |= (USART_DREINTLVL_HI_gc);
	return 0;
}

int get_c1(void)	// Non blocking, return status outside of char range
{
	int retc = AvrXPullFifo(Rx1Buf);
	return retc;
}

int get_char1(void)	// Blocks waiting for something
{
	return AvrXWaitPullFifo(Rx1Buf);
}
// We don't care if the buffer is full.  Just signal we got one.
// The task may attempt one extra time to get data out of an empyt
// buffer (getc(), but so what.  Eventually it will block waiting
// for another character to be received.

AVRX_SIGINT(USARTC1_RXC_vect)
{
    IntProlog();
	USARTC1_CTRLA &= ~(USART_RXCINTLVL_gm);	// Disable Rx interrupt
    sei();					// Allow other interrupt activity to occur
	AvrXPutFifo(Rx1Buf, USARTC1_DATA);// This resets the Rx Interrupt
	USARTC1_CTRLA |= (USART_RXCINTLVL_HI_gc);	// Re-enable.
	Epilog();
}

AVRX_SIGINT(USARTC1_DRE_vect)
{
    IntProlog();
    USARTC1_CTRLA &= ~(USART_DREINTLVL_gm);				// Disable UDRE interrupt
    sei();							// Allow other stuff to happen
	int c = AvrXPullFifo(Tx1Buf);	// Return -1 if empty (enables interrupts)
	if (c >= 0)						// Tricky tight code: only checking sign
	{
        //if( disableTx_1 ) USARTC1_CTRLA |= (USART_TXCINTLVL_HI_gc);
		//if (enableTx_1)  (*enableTx_1)();
		
		USARTC1_DATA = c;
		USARTC1_CTRLA |= (USART_DREINTLVL_HI_gc);
	}
	Epilog();
}

AVRX_SIGINT(USARTC1_TXC_vect)
{
    IntProlog();
    //(*disableTx_1)();
   USARTC1_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UDRE interrupt
	Epilog();
}

#endif	// USART_CHANNELS & CHANNEL_1

#endif /* OS_AVRX */