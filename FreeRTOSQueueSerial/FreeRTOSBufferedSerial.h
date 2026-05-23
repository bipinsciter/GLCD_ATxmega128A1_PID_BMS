#ifndef _FreeRTOSBufferedSerial_h_
#define _FreeRTOSBufferedSerial_h_
#include "FreeRTOS.h"
#include "queue.h"

// Channel queues
// these queues hold the datagrams ready to be sent out
//
QueueHandle_t vUSARTC0_RxQueue, vUSARTC0_TxQueue;
QueueHandle_t vUSARTC1_RxQueue, vUSARTC1_TxQueue;

// Buffer size can be any thing from 2 to 250
#define TX0_BUFSZ 192
#define RX0_BUFSZ 32
#define TX1_BUFSZ 128
#define RX1_BUFSZ 32

void put_char(USART_t * usart, char c);	

#endif //_FreeRTOSBufferedSerial_h_
