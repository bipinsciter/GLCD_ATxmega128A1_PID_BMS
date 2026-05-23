/*************************************************************************
* Title:    I2C master library using hardware TWI interface
**************************************************************************/
#include <inttypes.h>
#include <compat/twi.h>

#include "twimaster.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* I2C clock in Hz */
#define SCL_CLOCK  85000L

#define MAX_RETRY	3

#define MAX_RETRY_TIME_CNT	20000u

unsigned int retry;

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	for( retry = 0; retry < MAX_RETRY_TIME_CNT && (!(TWCR & (1<<TWINT))); retry++ )
	if( retry == MAX_RETRY_TIME_CNT )
	    return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	for( retry = 0; retry < MAX_RETRY_TIME_CNT && (!(TWCR & (1<<TWINT))); retry++ )
	if( retry == MAX_RETRY_TIME_CNT )
	    return 1;

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
unsigned char i2c_start_wait(unsigned char address)
{
    uint8_t   twst, i = 0;

    while ( i < MAX_RETRY )
    {
	    i++;
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
		for( retry = 0; retry < MAX_RETRY_TIME_CNT && (!(TWCR & (1<<TWINT))); retry++ )
		if( retry == MAX_RETRY_TIME_CNT )
		    return 1;
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
		for( retry = 0; retry < MAX_RETRY_TIME_CNT && (!(TWCR & (1<<TWINT))); retry++ )
		if( retry == MAX_RETRY_TIME_CNT )
		    return 1;
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
			for( retry = 0; retry < MAX_RETRY_TIME_CNT && (TWCR & (1<<TWSTO)); retry++ )
			if( retry == MAX_RETRY_TIME_CNT )
			    return 1;
	        
    	    continue;
    	}
    	if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }
	 if( i == MAX_RETRY )
	    return 1;
	 return 0;
}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	for( retry = 0; retry < (MAX_RETRY_TIME_CNT * 2) && (TWCR & (1<<TWSTO)); retry++ );
}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	for( retry = 0; retry < MAX_RETRY_TIME_CNT && (!(TWCR & (1<<TWINT))); retry++ )
	if( retry == MAX_RETRY_TIME_CNT )
	    return 1;

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	for( retry = 0; retry < (MAX_RETRY_TIME_CNT * 2) && (!(TWCR & (1<<TWINT))); retry++ );
    return TWDR;

}/* i2c_readAck */


unsigned char i2c_readAckWithStatus( unsigned char *ch)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	for( retry = 0; retry < (MAX_RETRY_TIME_CNT * 2) && (!(TWCR & (1<<TWINT))); retry++ );
    *ch = TWDR;
    return retry == (MAX_RETRY_TIME_CNT * 2);

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	for( retry = 0; retry < (MAX_RETRY_TIME_CNT * 2) && (!(TWCR & (1<<TWINT))); retry++ );
	
    return TWDR;

}/* i2c_readNak */

unsigned char i2c_readNakWithStatus( unsigned char *ch)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	for( retry = 0; retry < (MAX_RETRY_TIME_CNT * 2) && (!(TWCR & (1<<TWINT))); retry++ );
    *ch = TWDR;
    return retry == (MAX_RETRY_TIME_CNT * 2);

}/* i2c_readAck */

