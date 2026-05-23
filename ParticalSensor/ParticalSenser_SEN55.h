#ifndef __PARTICAL_SENS__
#define __PARTICAL_SENS__

#include <avr/io.h>

typedef enum _sen5x_err_t
{

	sen5x_no_error,
	sen5x_comm_error,
	sen5x_crc_error,

}sen5x_err_t;

typedef enum _sen5x_command_t
{
	
	SEN5x_start_measurement = 0x0021,
	SEN5x_start_gasonly_measurement = 0x0037,
	SEN5x_stop_measurement = 0x0104,
	SEN5x_data_ready_flag = 0x0202,
	SEN5x_read_measured_val = 0x03C4,
	SEN5x_start_fan_manually = 0x5607,
	SEN5x_reset_device = 0xD304 ,
	SEN5x_read_product_name = 0xD014 ,
	SEN5x_read_sr_number = 0xD033 ,
	SEN5x_read_fw_version = 0xD100 ,
	
}sen5x_command_t;

typedef struct _sen5x_measured_value_t
{
	uint16_t PM1_0;
	uint16_t PM2_5;
	uint16_t PM4_0;
	uint16_t PM10_0;
	int16_t humidity;
	int16_t temperature;
	uint16_t VOC_index;
	uint16_t NOX_index;
	uint8_t error;

}sen5x_measured_value_t;

sen5x_err_t sen5x_start_measurement(uint8_t SenNo);
sen5x_err_t sen5x_start_gasOnly_measurement(uint8_t SenNo);
sen5x_err_t sen5x_stop_measurement(uint8_t SenNo);
uint8_t sen5x_start_fan_manually(uint8_t SenNo);
uint8_t sen5x_check_meaurement_ready(uint8_t SenNo);
sen5x_err_t sen5x_read_measured_value(uint8_t SenNo, sen5x_measured_value_t* sen5x_params);
sen5x_err_t sen5x_reset_device(uint8_t SenNo);
sen5x_err_t sen5x_read_productname(uint8_t SenNo);
uint8_t sen5x_read_SrNo(uint8_t SenNoid);
uint8_t sen5x_read_FwVersion(uint8_t SenNo);

#endif
