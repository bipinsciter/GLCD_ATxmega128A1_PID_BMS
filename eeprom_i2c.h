#ifndef __EEPROM_I2C__
#define __EEPROM_I2C__

void eeprom_i2c_init();

uint8_t eeprom_i2c_ReadDataMemory( uint16_t address, uint8_t noOfBytes, uint8_t *value );

uint8_t eeprom_i2c_WriteDataMemory( uint16_t address, uint8_t noOfBytes, uint8_t *value );

#endif
