#ifndef __SERIALINTERFACE_H__
#define __SERIALINTERFACE_H__

#define TX0_ENB_RS485          PORTC_OUTSET = PIN4_bm 
#define TX0_DIS_RS485          PORTC_OUTCLR = PIN4_bm 
#define TX0_SETPORT_DIR        PORTC_DIRSET = PIN4_bm

#define TX1_ENB_RS485          PORTC_OUTSET = PIN5_bm 
#define TX1_DIS_RS485          PORTC_OUTCLR = PIN5_bm 
#define TX1_SETPORT_DIR        PORTC_DIRSET = PIN5_bm

extern uint8_t u8_resetTimer;

void SerialInterfaceInit();
void SerialBroadcast();
void SerialPrinter();
void SerialPrintAlarm( uint8_t alarmId, uint8_t state );
void SerialBroadcastAlarm( uint8_t alarmData );
void USART_SetBaudrate(USART_t * usart, int16_t bsel,int8_t bscale);
void InitUSART(USART_t * usart, PORT_t * port, uint8_t rxpin, uint8_t txpin, uint16_t ubrr, uint8_t noOfbits, uint8_t parity, uint8_t stopbit);
uint8_t CalCRC(uint8_t *ptr,uint16_t NoOfByte);
void software_reset(void);    
short findValue(char *ptr,unsigned char NoOfDigit);        
               
#endif
