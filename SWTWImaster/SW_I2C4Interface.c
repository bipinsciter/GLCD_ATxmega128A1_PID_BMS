
//#include "os_api.h"
#include "I2C4master.h"
#include "SW_I2C4Interface.h"

void SW_I2C4InterfaceInit()
{
   sw_i2c4_init();             // init I2C interface
}

// Reads a single byte from a specified address
uint8_t SW_I2C4ReadByte( uint8_t address, uint8_t * readData )
{
   if( sw_i2c4_start_wait(address+I2C_READ))  // set device address and read mode
		return 1;
	*readData = sw_i2c4_readNak();   // read last byte 
  	sw_i2c4_stop();    // set stop conditon = release bus

   return 0;
}

// Reads bytes from a specified address
uint8_t SW_I2C4ReadBytes( uint8_t address, uint8_t noOfReadBytes, uint8_t * readData )
{
   uint8_t i;

	if( noOfReadBytes )
	{
    	if( sw_i2c4_start_wait(address+I2C_READ))  // set device address and read mode
		{
			return 1;
		}
		
		for( i = 0; i < noOfReadBytes-1; i++)
			readData[i] = sw_i2c4_readAck();   // read one byte 
		readData[i] = sw_i2c4_readNak();   // read last byte 

    	sw_i2c4_stop();    // set stop condition = release bus
	}
	
   return 0;
}

// Writes a single byte to a specified address
uint8_t SW_I2C4WriteByte( uint8_t address, uint8_t dataVal )
{
   if( sw_i2c4_start_wait(address+I2C_WRITE)) // set device address and write mode
		return 1;
   sw_i2c4_write(dataVal); // write data
   sw_i2c4_stop();    // set stop conditon = release bus

   return 0;
}

// Writes bytes to a specified address
uint8_t SW_I2C4WriteBytes( uint8_t address, uint8_t noOfWriteBytes, uint8_t * writeData )
{
   int i;

   if( noOfWriteBytes )
	{
    	if( sw_i2c4_start_wait(address+I2C_WRITE)) // set device address and write mode
			return 1;
		for( i = 0; i < noOfWriteBytes; i++ )
		{
        	sw_i2c4_write(writeData[i]); // write data
		}
    	sw_i2c4_stop();    // set stop conditon = release bus
	}
	
   return 0;
}

// Writes and Reads bytes to and from a specified address
uint8_t SW_I2C4WriteReadBytes( uint8_t address, uint8_t noOfWriteBytes, uint8_t * writeData, uint8_t noOfReadBytes, uint8_t * readData )
{
   int i;
   
   if( noOfWriteBytes )
	{
    	if( sw_i2c4_start_wait(address+I2C_WRITE)) // set device address and write mode
			return 1;
		for( i = 0; i < noOfWriteBytes; i++ )
		{
        	sw_i2c4_write(writeData[i]); // write data
		}
	}
	if( noOfReadBytes )
	{
    	if( noOfWriteBytes )
        	i = sw_i2c4_rep_start(address+I2C_READ);       // set device address and read mode
		else
        	i = sw_i2c4_start_wait(address+I2C_READ);     // set device address and read mode

      if( i )
		{
			return 1;
		}

		for( i = 0; i < noOfReadBytes-1; i++)
			readData[i] = sw_i2c4_readAck();   // read one byte 
		readData[i] = sw_i2c4_readNak();   // read last byte 
	}
	if( noOfReadBytes || noOfWriteBytes )
    	sw_i2c4_stop();    // set stop conditon = release bus

   return 0;
}


// Writes bytes to a specified address slecting index register
uint8_t SW_I2C4IndexedWriteBytes( uint8_t address, uint8_t noOfIdxBytes, uint8_t *idxVal, uint8_t noOfWriteBytes, uint8_t * writeData )
{
    int i;
	
    if( noOfWriteBytes )
	{
    	if( sw_i2c4_start_wait(address+I2C_WRITE)) // set device address and write mode
			return 1;
		for( i = 0; i < noOfIdxBytes; i++ )
		{
        	sw_i2c4_write(idxVal[i]); // write register index
		}
		for( i = 0; i < noOfWriteBytes; i++ )
		{
        	sw_i2c4_write(writeData[i]); // write data
		}
    	sw_i2c4_stop();    // set stop conditon = release bus
	}

    return 0;
}

uint8_t SW_I2C4ReadBytesNoWait( uint8_t address, uint8_t noOfReadBytes, uint8_t * readData )
{
	uint8_t i;

	if( noOfReadBytes )
	{
		if( sw_i2c4_start(address+I2C_READ))  // set device address and read mode
		{
			return 1;
		}
		//    	tx = 0;
		for( i = 0; i < noOfReadBytes-1; i++)
		readData[i] = sw_i2c4_readAck();   // read one byte
		readData[i] = sw_i2c4_readNak();   // read last byte
		//		for( i = 0; i < noOfReadBytes-1; i++)
		//			tx |= i2c_readAckWithStatus(&readData[i]);   // read one byte
		//		tx |= i2c_readAckWithStatus(&readData[i]);   // read last byte

		sw_i2c4_stop();    // set stop conditon = release bus
	}

	return 0;
}

