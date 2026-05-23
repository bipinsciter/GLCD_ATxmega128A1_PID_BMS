#ifndef __PARTICAL_SENS_IPS7100__
#define __PARTICAL_SENS_IPS7100__

#include <avr/io.h>

// For CRC16 checksum
#define POLY 0x8408

#define PARTICAL_SENS_ADD_IPS7100	    (0x4B*2)

typedef enum _ips7100_err_t
{

	ips7100_no_error,
	ips7100_comm_error,
	ips7100_crc_error,

}ips7100_err_t;

typedef enum _ips7100_write_command_t
{
	IPS7100_start_measurement = 0x10,
	IPS7100_set_cleaning_interval = 0x21,
	IPS7100_set_power_saving_mode = 0x23,
	IPS7100_set_data_unit = 0x24,
	IPS7100_set_detection_range_voltage = 0x26,
	IPS7100_set_sensitivity_voltage = 0x29,
	IPS7100_set_fan_operation = 0x2B,
	IPS7100_start_cleaning = 0x2C,
	IPS7100_reset_module = 0x2D,
	IPS7100_factory_reset_setting = 0x2E,
	
}ips7100_write_command_t;

typedef enum _ips7100_read_command_t
{
	IPS7100_read_pc_data = 0x11,
	IPS7100_read_pm_data = 0x12,
	IPS7100_read_cleaning_interval = 0x61,
	IPS7100_read_data_unit = 0x64,
	IPS7100_read_measurement_interval = 0x65,
	IPS7100_read_detection_range_voltage = 0x66,
	IPS7100_read_sensitivity_voltage = 0x69,
	IPS7100_read_status = 0x6A,
	IPS7100_read_sr_number = 0x77,
	IPS7100_read_ver_number = 0x78,
	IPS7100_read_nw_sr_key = 0x79,
	
}ips7100_read_command_t;

typedef struct _ips7100_pc_value_t
{
	uint32_t PC0_1;
	uint32_t PC0_3;
	uint32_t PC0_5;
	uint32_t PC1_0;
	uint32_t PC2_5;
	uint32_t PC5_0;
	uint32_t PC10_0;
	uint8_t error;
	
}ips7100_pc_value_t;

typedef struct _ips7100_pm_value_t
{
	float PM0_1;
	float PM0_3;
	float PM0_5;
	float PM1_0;
	float PM2_5;
	float PM5_0;
	float PM10_0;
	uint16_t event_status;
	uint8_t error;
	
}ips7100_pm_value_t;

typedef enum _ips7100_measurement_interval_value_t
{
	IPS7100_STOP_MEASUREMENT = 0,
	IPS7100_200MS_MEASURE_INTERVAL = 1,
	IPS7100_500MS_MEASURE_INTERVAL = 2,
	IPS7100_1000MS_MEASURE_INTERVAL = 3,
	
}ips7100_measurement_interval_value_t;


ips7100_err_t ips7100_read_pc_value(uint8_t SenNo, ips7100_pc_value_t* ips7100_params);
ips7100_err_t ips7100_read_pm_value(uint8_t SenNo, ips7100_pm_value_t* ips7100_params);
uint8_t ips7100_start_measurement(uint8_t SenNo, ips7100_measurement_interval_value_t value);
uint8_t ips7100_reset_device(uint8_t SenNo);
uint8_t ips7100_factory_reset_setting(uint8_t SenNo);
uint16_t get_checksum(uint8_t *byte, uint8_t len);

#endif
