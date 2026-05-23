#include "os_api.h"
#include "SWTWImaster/SW_I2CInterface.h"
#include "eeprom_i2c.h"
#include "error.h"

#define I2C_ADDRESS			0xA0

static OSSemaMutex eepromI2CMutex;

#ifdef OS_AVRX
static TimerControlBlock eepromI2CWaitTimer;
#endif

void eeprom_i2c_init()
{
   OSSemaGive(eepromI2CMutex);
}

uint8_t eeprom_i2c_ReadDataMemory( uint16_t address, uint8_t noOfBytes, uint8_t *value )
{
    uint8_t * swap, temp;
    OSSemaTakeEver(eepromI2CMutex);
	swap = (uint8_t *)&address;
	temp = swap[0];
	swap[0] = swap[1];
	swap[1] = temp;
    // now address is MSB first when converted to array
    if( SW_I2CWriteReadBytes( I2C_ADDRESS, sizeof(address), (uint8_t *)&address, noOfBytes, value ))
	{
       OSSemaGive(eepromI2CMutex);
	   return ERROR_LOG_STORAGE_I2C;
	}
    OSSemaGive(eepromI2CMutex);
    return ERROR_OK;
}

uint8_t eeprom_i2c_WriteDataMemory( uint16_t address, uint8_t noOfBytes, uint8_t *value )
{
    uint8_t * swap, temp;
    OSSemaTakeEver(eepromI2CMutex);
	swap = (uint8_t *)&address;
	temp = swap[0];
	swap[0] = swap[1];
	swap[1] = temp;
    // now address is MSB first when converted to array
    if( SW_I2CIndexedWriteBytes( I2C_ADDRESS, sizeof(address), (uint8_t *)&address, noOfBytes, value ))
	{
       OSSemaGive(eepromI2CMutex);
	   return ERROR_LOG_STORAGE_I2C;
	}
	#ifdef OS_AVRX
	AvrXDelay(&eepromI2CWaitTimer, 30);
	#else
	OSSleep(30);
	#endif

    OSSemaGive(eepromI2CMutex);
    return ERROR_OK;
}
