/*************************************************************************
; Title	:    I2C (Single) Master Implementation
;
; DESCRIPTION
; 	Basic routines for communicating with I2C slave devices. This
;	"single" master implementation is limited to one bus master on the
;	I2C bus. 
;  
; USAGE
; 	Adapt the SCL and SDA port and pin definitions and eventually 
;	the delay routine to your target !
; 	Use 4.7k pull-up resistor on the SDA and SCL pin.
;
; NOTES
;	The I2C routines can be called either from non-interrupt or
;	interrupt routines, not both.
*************************************************************************/


#include <avr/io.h>
#include <util/delay.h>
#include "i2c1master.h"

#define SDA_HIGH		PORTC_OUTSET = PIN0_bm
#define SDA_LOW			PORTC_OUTCLR = PIN0_bm
#define SDA_SENSE		(PORTC_IN & PIN0_bm)
#define SDA_DIR_IN		PORTC_DIRCLR = PIN0_bm
#define SDA_DIR_OUT		PORTC_DIRSET = PIN0_bm

#define SCL_HIGH		PORTC_OUTSET = PIN1_bm
#define SCL_LOW			PORTC_OUTCLR = PIN1_bm
#define SCL_DIR_IN		PORTC_DIRCLR = PIN1_bm
#define SCL_DIR_OUT		PORTC_DIRSET = PIN1_bm

//static void delay1(uint16_t cnt)
//{
	//while(cnt)
	//{
		//asm volatile ( "nop" );
		//cnt--;
	//}
//}

/*************************************************************************
; Initialization of the I2C bus interface. Need to be called only once
; 
; extern void sw_i2c1_init(void)
;*************************************************************************/
void sw_i2c1_init(void)
{
	SCL_DIR_OUT;           					// Enable SCL as output.
	SDA_DIR_OUT;           					// Enable SDA as output.
  
	SDA_HIGH;           					// Enable pullup on SDA, to set high as released state.
	SCL_HIGH;						        // Enable pullup on SCL, to set high as released state.
}

/*************************************************************************	
; Issues a start condition and sends address and transfer direction.
; return 0 = device accessible, 1= failed to access device
;
; extern unsigned char sw_i2c1_start(unsigned char addr);
;*************************************************************************/

unsigned char sw_i2c1_start(unsigned char addr)
{
	SDA_HIGH;
	_delay_us(50);
	SCL_HIGH;
	_delay_us(50);
	SDA_LOW;
	_delay_us(50);
	SCL_LOW;
	_delay_us(50);

   return sw_i2c1_write(addr);    // write address
}

/*************************************************************************
; Issues a repeated start condition and sends address and transfer direction.
; return 0 = device accessible, 1= failed to access device
;
; extern unsigned char sw_i2c1_rep_start(unsigned char addr);
;*************************************************************************/

unsigned char sw_i2c1_rep_start(unsigned char addr)
{
	SDA_HIGH;
	_delay_us(50);
	SCL_HIGH;
	_delay_us(50);
	SDA_LOW;
	_delay_us(50);
	SCL_LOW;
	_delay_us(50);
	
    return sw_i2c1_write( addr ); // write address
}

/*************************************************************************	
; Issues a start condition and sends address and transfer direction.
; If device is busy, use ack polling to wait until device is ready
;
; unsigned char sw_i2c1_start_wait(unsigned char addr);
;*************************************************************************/
unsigned char sw_i2c1_start_wait(unsigned char addr)
{
   register int i = 0;
   while( i++ < 3 )
   {
      if( sw_i2c1_start( addr ) == 0 )
	  {
         return 0;
	  }
	  
	  sw_i2c1_stop();	// terminate write operation
   }
   return 1;
}

/*************************************************************************
; Terminates the data transfer and releases the I2C bus
;
; extern void sw_i2c1_stop(void)
;*************************************************************************/

void sw_i2c1_stop(void)
{
	SDA_LOW;
	_delay_us(50);
	SCL_HIGH;
	_delay_us(50);
	SDA_HIGH;
	_delay_us(50);
}

/*************************************************************************
; Send one byte to I2C device
; return 0 = write successful, 1 = write failed
;
; extern unsigned char sw_i2c1_write( unsigned char data );
;*************************************************************************/

unsigned char sw_i2c1_write( unsigned char data )
{
	unsigned char i;//register unsigned char i;
   
	//SCL_LOW;                // Pull SCL LOW.
    SDA_DIR_OUT;
	
	for (i=0; i<8; i++)
	{ 
		if (data & 0x80)
			SDA_HIGH;				// release SDA
		else
			SDA_LOW;   // sbi SDA_DDR,SDA	;force SDA low

		_delay_us(50);

		SCL_HIGH;  // release SCL
		_delay_us(50);
		SCL_LOW;   // sbi SCL_DDR,SCL	;force SCL low
		_delay_us(50);
		data = data << 1;
	}
   
	SDA_DIR_IN;
   
	_delay_us(50);

	SCL_HIGH;  // release SCL
	_delay_us(50);
	i = (SDA_SENSE)? 1: 0;
	SCL_LOW;
	
	SDA_DIR_OUT;
	
	_delay_us(50);
	
	return i;
}


/*************************************************************************
; read one byte from the I2C device, send ack or nak to device
; (ack=1, send ack, request more data from device 
;  ack=0, send nak, read is followed by a stop condition)
;
; extern unsigned char sw_i2c1_read(unsigned char ack);
;*************************************************************************/
unsigned char sw_i2c1_read(unsigned char ack)
{
	unsigned char i, res;//register unsigned char i, res;
	res = 0;
   
	SDA_DIR_IN;	  // release SDA (from previous ACK)
   
	for (i=0;i<8;i++)          // each bit at a time, MSB first
	{
		SCL_HIGH;
		res<<= 1;
		_delay_us(50);

		if(SDA_SENSE) res |= 0x01;

		_delay_us(50);
		SCL_LOW;
		_delay_us(50);
	}
	
	SDA_DIR_OUT;
   
	if (ack == 0)
		SDA_HIGH;  // Send ACK
	else
		SDA_LOW; // Send NO ACK

	_delay_us(50);

	SCL_HIGH;
	_delay_us(50);
	SCL_LOW;
	
	return res;
}


/*************************************************************************
; read one byte from the I2C device, send ack to device
; (send ack, request more data from device)
;
; extern unsigned char sw_i2c1_readAck(void);
;*************************************************************************/
unsigned char sw_i2c1_readAck(void)
{
   return sw_i2c1_read(1);
}

/*************************************************************************
; read one byte from the I2C device, send nak to device
; (send nak, read is followed by a stop condition)
;
; extern unsigned char sw_i2c1_readNak(void);
;*************************************************************************/
unsigned char sw_i2c1_readNak(void)
{
   return sw_i2c1_read(0);
}
