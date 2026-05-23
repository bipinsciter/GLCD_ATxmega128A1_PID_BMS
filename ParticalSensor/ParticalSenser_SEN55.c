#include "SW_I2C2Interface.h"
#include "SW_I2C3Interface.h"
#include "ParticalSenser_SEN55.h"
#include "error.h"
#include "UserInterface.h"
#include <util/delay.h>

#define PARTICAL_SENS_ADD_SEN55	    (0x69*2)

uint8_t CalcCrc(uint8_t data[2]) 
{
	uint8_t crc = 0xFF;
	for(int i = 0; i < 2; i++) 
	{
		crc ^= data[i];
		
		for(uint8_t bit = 8; bit > 0; --bit) 
		{
			if(crc & 0x80) 
			{
				crc = (crc << 1) ^ 0x31u;
			} 
			else 
			{
				crc = (crc << 1);
			}
		}
	}
	return crc;
}

uint8_t sen5x_start_measurement(uint8_t SenNo)
{
	uint8_t readData[2];

	readData[0] = ( SEN5x_start_measurement & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_start_measurement & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}

uint8_t sen5x_start_fan_manually(uint8_t SenNo)
{
	uint8_t readData[2];

	readData[0] = ( SEN5x_start_fan_manually & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_start_fan_manually & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}


uint8_t sen5x_stop_measurement(uint8_t SenNo)
{
	uint8_t readData[2];

	readData[0] = ( SEN5x_stop_measurement & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_stop_measurement & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}

uint8_t sen5x_start_gasOnly_measurement(uint8_t SenNo)
{
	uint8_t readData[2];

	readData[0] = ( SEN5x_start_gasonly_measurement & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_start_gasonly_measurement & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}

uint8_t sen5x_check_meaurement_ready(uint8_t SenNo)
{
	uint8_t readData[3];
	struct _datarady{
		uint8_t unused;
		uint8_t is_ready;
		uint8_t crc;
	}dataready;

	readData[0] = ( SEN5x_data_ready_flag & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_data_ready_flag & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	//_delay_us(100);
	_delay_ms(1);
	
	if(!SenNo)
	{
		if( SW_I2C2ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(dataready), (uint8_t *)&dataready ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(dataready), (uint8_t *)&dataready ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	if(dataready.crc != CalcCrc((uint8_t *)&dataready))
	{
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if(!dataready.is_ready)
			return ERROR_PARTICAL_SENS_I2C;
		else
			return ERROR_OK;
	}
}

uint8_t sen5x_read_measured_value(uint8_t SenNo, sen5x_measured_value_t* sen5x_params)
{
	uint8_t readData[24];
	
	readData[0] = ( SEN5x_read_measured_val & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_read_measured_val & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	//_delay_us(500);
	_delay_ms(1);
	
	if(!SenNo)
	{
		if( SW_I2C2ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	for(uint8_t i = 0; i< 8; i++)
	{
		uint8_t crc = CalcCrc(readData + (3*i));
		if(crc != readData[(3*i)+2])
		{
			return ERROR_PARTICAL_SENS_I2C;
		}
		else
		{
			((uint16_t*)sen5x_params)[i] = (readData[i*3]<<8) + (readData[(i*3) +1]) ; 
		}
	}
	
	return ERROR_OK;
}

uint8_t sen5x_reset_device(uint8_t SenNo)
{
	uint8_t readData[2];

	readData[0] = ( SEN5x_reset_device & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_reset_device & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	return ERROR_OK;
}

uint8_t sen5x_read_productname(uint8_t SenNo)
{
	uint8_t readData[48]={0};

	readData[0] = ( SEN5x_read_product_name & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_read_product_name & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	_delay_ms(1);
	
	if(!SenNo)
	{
		if( SW_I2C2ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}

uint8_t sen5x_read_SrNo(uint8_t SenNo)
{
	uint8_t readData[48]={0};

	readData[0] = ( SEN5x_read_sr_number & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_read_sr_number & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	_delay_ms(1);
	
	if(!SenNo)
	{
		if( SW_I2C2ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}


uint8_t sen5x_read_FwVersion(uint8_t SenNo)
{
	uint8_t readData[3]={0};

	readData[0] = ( SEN5x_read_fw_version & 0XFF00 ) >> 8;
	readData[1] = ( SEN5x_read_fw_version & 0XFF );
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_SEN55, 2, readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	_delay_ms(1);	  
	
	if(!SenNo)
	{
		if( SW_I2C2ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3ReadBytes( PARTICAL_SENS_ADD_SEN55, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}


