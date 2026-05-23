#include "SW_I2C1Interface.h"
//#include "SW_I2C3Interface.h"
#include "I2C1master.h"
#include "TempRHSens.h"
#include "error.h"
#include "os_api.h"
#include <avr/io.h>
#include <util/delay.h>

#define TEMP_ADDRESS		0x2
#define HUMIDITY_ADDRESS	0x4

#define SHT25_ADDRESS      0x80
#define SHT25_TEMP_CMD     0xF3
#define SHT25_HUMIDITY_CMD 0xF5

#define SHT35_ADDRESS          (0x44*2)
#define SHT35_RESET_CMD        0x30A2
//#define SHT35_PSTART_CMD       0x2B32
#define SHT35_PSTART_CMD       0x2130
#define SHT35_TEMPRH_READ_CMD  0xe000

#define IDT_HS3100_ADDRESS          (0x44*2)

#define TEMP_DELAY_MS      1000
#define HUMIDITY_DELAY_MS  1000

#define POLYNOMIAL			0x131			//P(x)=x^8+x^5+x^4+1 = 100110001
 
#ifdef OS_AVRX
TimerControlBlock   tempSensSleepTimer;  
#endif

uint8_t ResetSensorSHT35(uint8_t SensNo)
{
	uint8_t readData[2];

	readData[0] = ( SHT35_RESET_CMD & 0XFF00 ) >> 8;
	readData[1] = ( SHT35_RESET_CMD & 0XFF );

	if(!SensNo)
	{
		if( SW_I2C1WriteBytes( SHT35_ADDRESS, sizeof(readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1WriteBytes( SHT35_ADDRESS, sizeof(readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	
	return ERROR_OK;
}
      
uint8_t StartSensorSHT35(uint8_t SensNo)
{
	uint8_t readData[2];

	readData[0] = ( SHT35_PSTART_CMD & 0XFF00 ) >> 8;
	readData[1] = ( SHT35_PSTART_CMD & 0XFF );
	
	if(!SensNo)
	{
		if( SW_I2C1WriteBytes( SHT35_ADDRESS, sizeof(readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1WriteBytes( SHT35_ADDRESS, sizeof(readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	
	return ERROR_OK;
}

uint8_t GetSensorSHT35Readings(uint8_t SensNo, unsigned int * temperature, unsigned int * humidity )
{
	uint8_t readData[6];

	readData[0] = ( SHT35_TEMPRH_READ_CMD & 0XFF00 ) >> 8;   
	readData[1] = ( SHT35_TEMPRH_READ_CMD & 0XFF );   
   
	if(!SensNo)
	{
		if( SW_I2C1WriteReadBytes( SHT35_ADDRESS, 2, readData, 4, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1WriteReadBytes( SHT35_ADDRESS, 2, readData, 4, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}

	*temperature = ((unsigned int)readData[0] << 8 ) | (unsigned int)readData[1];
	*humidity = ((unsigned int)readData[3] << 8 ) | (unsigned int)readData[4]; 
	
	return ERROR_OK;
}

uint8_t StartSensor_IDT_HS3100(uint8_t SensNo)
{	
	if(!SensNo)
	{
		if( sw_i2c1_start_wait( IDT_HS3100_ADDRESS ) )
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( sw_i2c1_start_wait( IDT_HS3100_ADDRESS ) )
		return ERROR_TEMP_RH_SENS_I2C;
	}

	return ERROR_OK;
}

uint8_t GetSensorReadings_IDT_HS3100(uint8_t SensNo, unsigned int * temperature, unsigned int * humidity )
{
	uint8_t readData[4];

	if(!SensNo)
	{
		if( SW_I2C1ReadBytesNoWait( IDT_HS3100_ADDRESS, sizeof( readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1ReadBytesNoWait( IDT_HS3100_ADDRESS, sizeof( readData), readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	
	*humidity = ((readData[0] & 0x3F) << 8) + readData[1];
	*temperature = (((readData[2] << 8) + readData[3]) >> 2 ) & 0x3fff;
	return ERROR_OK;
}

uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
	uint8_t crc = 0;	
	uint8_t byteCtr;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ 
		crc ^= (data[byteCtr]);
		
		for (unsigned char bit = 8; bit > 0; --bit)
		{ 
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	
	if (crc != checksum) return 1;
	else return 0;
}

unsigned int SHT2x_CalcRH(unsigned int u16sRH)
{
	float humidityRH;              // variable for result

	u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
	
	//-- calculate relative humidity [%RH] --
	humidityRH = -6.0 + ((125.0/65536) * (float)u16sRH); // RH= -6 + 125 * SRH/2^16
	return (unsigned int)(humidityRH*100);
}

unsigned int SHT2x_CalcTemperatureC(unsigned int u16sT)
{
	float temperatureC;            // variable for result

	u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

	//-- calculate temperature [°C] --
	temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
	return (unsigned int)(temperatureC*100);
}

uint8_t GetSensorSHT25Temperature( uint8_t SensNo, unsigned int * temperature, unsigned char * checksum )
{
	uint8_t readData[3], *temp;
	unsigned int i = 0;

	readData[0] = SHT25_TEMP_CMD;   
	if(!SensNo)
	{
		if( SW_I2C1WriteBytes( SHT25_ADDRESS, 1, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1WriteBytes( SHT25_ADDRESS, 1, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	
	while ( i < TEMP_DELAY_MS )
	{
		if(!SensNo)
		{
			if( !SW_I2C1ReadBytes( SHT25_ADDRESS, sizeof( readData), readData ))
			break;
		}
		else
		{
			if( !SW_I2C1ReadBytes( SHT25_ADDRESS, sizeof( readData), readData ))
			break;
		}

		_delay_ms(5);
		
		i += 20;
	}
	
	if( i >= TEMP_DELAY_MS )
	return ERROR_TEMP_RH_SENS_I2C;

	temp = (uint8_t *)temperature;
	temp[0] = readData[1];
	temp[1] = readData[0];
	*checksum = readData[2];
  
	if(!SHT2x_CheckCrc (readData,2,readData[2]))// && (*temperature != 0xFFFF)) 
	{
		*temperature = SHT2x_CalcTemperatureC(*temperature);
		return ERROR_OK;
	}
	else 
	{
		return 1;
	}
}

uint8_t GetSensorSHT25Humidity( uint8_t SensNo,  unsigned int * humidity, unsigned char * checksum )
{
	uint8_t readData[3], *temp;
	unsigned int i = 0;

	readData[0] = SHT25_HUMIDITY_CMD; 
	
	if(!SensNo)
	{
		if( SW_I2C1WriteBytes( SHT25_ADDRESS, 1, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	else
	{
		if( SW_I2C1WriteBytes( SHT25_ADDRESS, 1, readData ))
		return ERROR_TEMP_RH_SENS_I2C;
	}
	  
	while ( i < HUMIDITY_DELAY_MS )
	{
		if(!SensNo)
		{
			if( !SW_I2C1ReadBytes( SHT25_ADDRESS, sizeof( readData), readData ))
			break;
		}
		else
		{
			if( !SW_I2C1ReadBytes( SHT25_ADDRESS, sizeof( readData), readData ))
			break;
		}

		_delay_ms(5);
		
		i += 20;
	}
	
	if( i >= HUMIDITY_DELAY_MS )
	return ERROR_TEMP_RH_SENS_I2C;

	temp = (uint8_t *)humidity;
	temp[0] = readData[1];
	temp[1] = readData[0];
	*checksum = readData[2];
   
	if(!SHT2x_CheckCrc (readData,2,readData[2]))// && (*humidity != 0xFFFF)) 
	{
		*humidity = SHT2x_CalcRH(*humidity);
		return ERROR_OK;
	}
	else 
	{
		return 1;
	}
}

