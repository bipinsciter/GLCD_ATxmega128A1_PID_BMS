#ifndef _I2C1MASTER_H
#define _I2C1MASTER_H   1

#include <avr/io.h>

/** defines the data direction (reading from I2C device) in sw_i2c_start(),sw_i2c_rep_start() */
#define I2C_READ    1

/** defines the data direction (writing to I2C device) in sw_i2c_start(),sw_i2c_rep_start() */
#define I2C_WRITE   0


/**
 @brief initialize the I2C master interace. Need to be called only once 
 @param  void
 @return none
 */
void sw_i2c1_init(void);


/** 
 @brief Terminates the data transfer and releases the I2C bus 
 @param void
 @return none
 */
void sw_i2c1_stop(void);


/** 
 @brief Issues a start condition and sends address and transfer direction 
  
 @param    addr address and transfer direction of I2C device
 @retval   0   device accessible 
 @retval   1   failed to access device 
 */
unsigned char sw_i2c1_start(unsigned char addr);


/**
 @brief Issues a repeated start condition and sends address and transfer direction 

 @param   addr address and transfer direction of I2C device
 @retval  0 device accessible
 @retval  1 failed to access device
 */
unsigned char sw_i2c1_rep_start(unsigned char addr);


/**
 @brief Issues a start condition and sends address and transfer direction 
   
 If device is busy, use ack polling to wait until device ready 
 @param    addr address and transfer direction of I2C device
 @retval  0 device accessible
 @retval  1 failed to access device
 */
unsigned char sw_i2c1_start_wait(unsigned char addr);

 
/**
 @brief Send one byte to I2C device
 @param    data  byte to be transfered
 @retval   0 write successful
 @retval   1 write failed
 */
unsigned char sw_i2c1_write(unsigned char data);


/**
 @brief    read one byte from the I2C device, request more data from device 
 @return   byte read from I2C device
 */
unsigned char sw_i2c1_readAck(void);

/**
 @brief    read one byte from the I2C device, read is followed by a stop condition 
 @return   byte read from I2C device
 */
unsigned char sw_i2c1_readNak(void);

/** 
 @brief    read one byte from the I2C device
 
 Implemented as a macro, which calls either sw_i2c1_readAck or sw_i2c1_readNak
 
 @param    ack 1 send ack, request more data from device<br>
               0 send nak, read is followed by a stop condition 
 @return   byte read from I2C device
 */
unsigned char sw_i2c1_read(unsigned char ack);
unsigned char sw_i2c1_readAck();
unsigned char sw_i2c1_readNak();

/**@}*/
#endif
