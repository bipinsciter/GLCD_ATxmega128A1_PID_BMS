#include "SW_I2C1Interface.h"
#include "DAC7573.h"
#include "error.h"

#define DAC_ADDRESS        0x98

#define CTRL_BYTE_BASE_VAL 0x10

#define CHANNEL_SHIFT      0x1
#define DATA_SHIFT		   0x4



uint8_t DACWriteOutValue( uint8_t channel, uint16_t data )
{
    uint8_t regAdd = CTRL_BYTE_BASE_VAL | ((channel  & 0x3)<<CHANNEL_SHIFT);
	data = data << DATA_SHIFT;
	regAdd = (uint8_t)(data >> 8);
	data = data << 8;
	data |= regAdd;
    regAdd = CTRL_BYTE_BASE_VAL | ((channel  & 0x3)<<CHANNEL_SHIFT);
	
    if( SW_I2C1IndexedWriteBytes( DAC_ADDRESS, 1, &regAdd, sizeof(data), (uint8_t *)&data ))
	{
      return ERROR_ANALOG_OUT_I2C;
	}
	else
	{
		return ERROR_OK;
	}
}


