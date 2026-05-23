#ifndef __SW_I2C4INTERFACE__
#define __SW_I2C4INTERFACE__

#include <avr/io.h>

void SW_I2C4InterfaceInit();

// Reads a single byte from a specified address
uint8_t SW_I2C4ReadByte( uint8_t address, uint8_t * readData );

// Reads bytes from a specified address
uint8_t SW_I2C4ReadBytes( uint8_t address, uint8_t noOfBytes, uint8_t * readData );
uint8_t SW_I2C4ReadBytesNoWait( uint8_t address, uint8_t noOfReadBytes, uint8_t * readData );

// Writes a single byte to a specified address
uint8_t SW_I2C4WriteByte( uint8_t address, uint8_t dataVal );

// Writes bytes to a specified address
uint8_t SW_I2C4WriteBytes( uint8_t address, uint8_t noOfBytes, uint8_t * writeData );

// Writes and Reads bytes to and from a specified address
uint8_t SW_I2C4WriteReadBytes( uint8_t address, uint8_t noOfWriteBytes, uint8_t * writeData, uint8_t noOfReadBytes, uint8_t * readData );

// Writes bytes to a specified address slecting index register
uint8_t SW_I2C4IndexedWriteBytes( uint8_t address, uint8_t noOfIdxBytes, uint8_t *idxVal, uint8_t noOfWriteBytes, uint8_t * writeData );

#endif
