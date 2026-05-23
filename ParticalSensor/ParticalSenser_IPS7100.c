#include "SW_I2C2Interface.h"
#include "SW_I2C3Interface.h"
#include "ParticalSenser_IPS7100.h"
#include "error.h"
#include "UserInterface.h"
#include <util/delay.h>

uint16_t get_checksum(uint8_t *byte, uint8_t len) 
{
	int i, j;
	uint16_t data = 0;
	uint16_t crc = 0xffff;
	for (j = 0; j < len; j++)
	{
		data = (uint16_t)0xff & byte[j];
		for (i = 0; i < 8; i++, data >>= 1)
		{
			if ((crc & 0x0001) ^ (data & 0x0001))
				crc = (crc >> 1) ^ POLY;
			else
				crc >>= 1;
		}
	}
	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xff);
	return crc;
}

uint8_t ips7100_start_measurement(uint8_t SenNo, ips7100_measurement_interval_value_t value)
{
	uint8_t readData[2];

	readData[0] = IPS7100_start_measurement;
	readData[1] = value;
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_IPS7100, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_IPS7100, sizeof(readData), readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}

	return ERROR_OK;
}

ips7100_err_t ips7100_read_pc_value(uint8_t SenNo, ips7100_pc_value_t* ips7100_params)
{
	uint8_t readData[30];
	uint16_t rcvCRC=0,calCRC=0;
	
	readData[0] = IPS7100_read_pc_data;
	
	if(!SenNo)
	{	
		if( SW_I2C2WriteReadBytes( PARTICAL_SENS_ADD_IPS7100, 1, readData, 30, readData))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteReadBytes( PARTICAL_SENS_ADD_IPS7100, 1, readData, 30, readData))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	rcvCRC = (readData[28]<<8) | readData[29];
	calCRC = get_checksum(readData,sizeof(readData)-2);
	
	if(rcvCRC != calCRC)
	{
		return ips7100_crc_error;
	}
	else
	{
		for(uint8_t i=0; i<7; i++)
		{
			((uint32_t*)ips7100_params)[i] = ((uint32_t)readData[i*4]<<24) | ((uint32_t)readData[(i*4)+1]<<16) | (readData[(i*4)+2]<<8) | (readData[(i*4)+3]); 
		}
	}
	
	return ips7100_no_error;
}

ips7100_err_t ips7100_read_pm_value(uint8_t SenNo, ips7100_pm_value_t* ips7100_params)
{
	uint8_t readData[32];
	uint16_t rcvCRC=0,calCRC=0;
	union bytesToPM
	{
		float f;
		unsigned char byte[4];
	}b;
	
	readData[0] = IPS7100_read_pm_data;
	
	if(!SenNo)
	{	
		if( SW_I2C2WriteReadBytes( PARTICAL_SENS_ADD_IPS7100, 1, readData, 32, readData))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteReadBytes( PARTICAL_SENS_ADD_IPS7100, 1, readData, 32, readData))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	rcvCRC = (readData[30]<<8) | readData[31];
	calCRC = get_checksum(readData,sizeof(readData)-2);
	
	if(rcvCRC != calCRC)
	{
		return ips7100_crc_error;
	}
	else
	{
		for(uint8_t i=0; i<7; i++)
		{
			for(uint8_t j = 0; j < 4; ++j)
			{
				b.byte[j] = readData[j + (i * 4)];
			}
			
			((float*)ips7100_params)[i] = b.f;
		}
	}
	
	ips7100_params->event_status = (readData[28]<<8) | readData[29];
	
	return ips7100_no_error;
}

uint8_t ips7100_reset_device(uint8_t SenNo)
{
	uint8_t readData;

	readData = IPS7100_reset_module;
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_IPS7100, 1, &readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_IPS7100, 1, &readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	return ERROR_OK;
}

uint8_t ips7100_factory_reset_setting(uint8_t SenNo)
{
	uint8_t readData;

	readData = IPS7100_factory_reset_setting;
	
	if(!SenNo)
	{
		if( SW_I2C2WriteBytes( PARTICAL_SENS_ADD_IPS7100, 1, &readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	else
	{
		if( SW_I2C3WriteBytes( PARTICAL_SENS_ADD_IPS7100, 1, &readData ))
		return ERROR_PARTICAL_SENS_I2C;
	}
	
	return ERROR_OK;
}

