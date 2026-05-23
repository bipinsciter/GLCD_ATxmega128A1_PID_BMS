#include "SW_I2C2Interface.h"
#include "rtc.h"
#include "error.h"

#if defined(DS1307)

#define RTC_ADDRESS			0xD0

#elif defined(PCF8563)

#define RTC_ADDRESS			0xA2

#endif

//uint8_t RTCSetTime( TIME * time )
//{
    //uint8_t regAdd = RTC_TIMEREG_START;
    //time->hours &= 0x3f;
    //if( SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &regAdd, sizeof(TIME), (uint8_t *)time ))
	   //return ERROR_RTC_I2C;
    //return ERROR_OK;
//}

uint8_t RTCGetTime( TIME * time )
{
    uint8_t regAdd = RTC_TIMEREG_START;
    if( SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, sizeof(TIME), (uint8_t *)time ))
	   return ERROR_RTC_I2C;
	time->seconds &= 0x7F;
	time->minutes &= 0x7F;
	time->hours &= 0x3F;
	
    return ERROR_OK;
}


//uint8_t RTCSetDate( DATE * date )
//{
    //uint8_t regAdd = RTC_DATE_REG_START;
    //if( SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &regAdd, sizeof(DATE), (uint8_t *)date ))
	   //return ERROR_RTC_I2C;
    //return ERROR_OK;
//}


uint8_t RTCGetDate( DATE * date )
{
    uint8_t regAdd = RTC_DATE_REG_START;
    if( SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, sizeof(DATE), (uint8_t *)date ))
	   return ERROR_RTC_I2C;
	   
	date->month &= 0x1F;
	date->weekday &= 0x07;
	date->date &= 0x3F;
	   
    return ERROR_OK;
}

uint8_t RTCReadTimeDateRegister( uint8_t address, uint8_t *value )
{
    if( SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &address, 1, value ))
	   return ERROR_RTC_I2C;
    return ERROR_OK;
}

uint8_t RTCWriteTimeDateRegister( uint8_t address, uint8_t value )
{
    if( SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &address, 1, &value ))
	   return ERROR_RTC_I2C;
    return ERROR_OK;
}

//uint8_t RTCReadDataMemory( uint8_t offset, uint8_t noOfBytes, uint8_t *value )
//{
    //uint8_t regAdd = RTC_DATA_REG_START + offset;
    //if( SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, noOfBytes, value ))
	   //return ERROR_RTC_I2C;
    //return ERROR_OK;
//}
//
//uint8_t RTCWriteDataMemory( uint8_t offset, uint8_t noOfBytes, uint8_t *value )
//{
    //uint8_t regAdd = RTC_DATA_REG_START + offset;
    //if( SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &regAdd, noOfBytes, value ))
	   //return ERROR_RTC_I2C;
    //return ERROR_OK;
//}
//

void InitRTC(void)
{
	#if defined(DS1307)

	uint8_t regAdd = RTC_TIMESEC_REG;
	uint8_t value;

	regAdd = RTC_TIMESEC_REG;
	SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );
	value &= 0x7F;
	SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );

	regAdd = RTC_TIMEHOUR_REG;
	SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );
	
	if(( value & 0x40 ) != 0)
	{
		if( value & 0x20 )
		{
			value = (value & 0xf) + ((value & 0x10)>>4) * 10;
			value += 12;
			value = ((value / 10) << 4) + (value % 10);
		}
		SW_I2C2IndexedWriteBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );
	}

	#elif defined(PCF8563)

	uint8_t regAdd = RTC_CNTRL1_ADDR;
	uint8_t value=0x00;

	regAdd = RTC_CNTRL1_ADDR;
	value=0x00;
	SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );
	
	regAdd = RTC_CNTRL2_ADDR;
	value=0x00;
	SW_I2C2WriteReadBytes( RTC_ADDRESS, 1, &regAdd, 1, &value );

	#endif
}


