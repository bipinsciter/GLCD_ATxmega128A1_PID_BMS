#include <avr/io.h>
#include <avr/interrupt.h>
#include "UserInterface.h"
#include "FreeRTOSBufferedSerial.h"
#include "FreeRTOS.h"
#include "SerialInterface.h"

#ifndef OS_AVRX

void put_char(USART_t * usart, char c)	// Blocking output
{
	while(!(usart->STATUS & USART_DREIF_bm));
	usart->DATA=c;
	
	while(!(usart->STATUS & USART_TXCIF_bm));
	usart->STATUS |= USART_DREIF_bm;
}

//=============================================================================================================
ISR(USARTC0_RXC_vect)
{	
	signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    ucChar = USARTC0_DATA;

    xQueueSendFromISR( vUSARTC0_RxQueue, &ucChar, &xHigherPriorityTaskWoken );

	//portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	
    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        //portYIELD_FROM_ISR();
    }
}

ISR(USARTC0_DRE_vect)
{
	signed char cChar, cTaskWoken = pdFALSE;

	if( xQueueReceiveFromISR( vUSARTC0_TxQueue, &cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		USARTC0_DATA = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		USARTC0_CTRLA &= ~(USART_DREINTLVL_gm);			// Disable UDRE interrupt
	}
}

ISR(USARTC0_TXC_vect)
{
	TX0_DIS_RS485;
	USARTC0_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UTXC interrupt
}

//=============================================================================================================

// We don't care if the buffer is full.  Just signal we got one.
// The task may attempt one extra time to get data out of an empyt
// buffer (getc(), but so what.  Eventually it will block waiting
// for another character to be received.

ISR(USARTC1_RXC_vect)
{	
	signed char ucChar, xHigherPriorityTaskWoken = pdFALSE;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    ucChar = USARTC1_DATA;

    xQueueSendFromISR( vUSARTC1_RxQueue, &ucChar, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        //portYIELD_FROM_ISR();
    }
}

ISR(USARTC1_DRE_vect)
{
    signed char cChar, cTaskWoken = pdFALSE;

    if( xQueueReceiveFromISR( vUSARTC1_TxQueue, &cChar, &cTaskWoken ) == pdTRUE )
    {
	    /* Send the next character queued for Tx. */
	    USARTC1_DATA = cChar;
    }
    else
    {
	    /* Queue empty, nothing to send. */
	    USARTC1_CTRLA &= ~(USART_DREINTLVL_gm);			// Disable UDRE interrupt
    }
}

ISR(USARTC1_TXC_vect)
{
   TX1_DIS_RS485;
   USARTC1_CTRLA &= ~(USART_TXCINTLVL_gm);			// Disable UTXC interrupt
}


#endif /* OS_FREERTOS */