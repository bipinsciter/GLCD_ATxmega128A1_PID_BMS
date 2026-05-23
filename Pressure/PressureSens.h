#ifndef __PRESSURE_SENS__
#define __PRESSURE_SENS__

#include <avr/io.h>

#define DP1_SENSOR_RESOLUTION_OTHER			0x7FFF  // 15 bits
#define DP1_SENSOR_RESOLUTION_SM9543		0x3FFF  // 14 bits
#define DP2_SENSOR_RESOLUTION_OTHER			0x7FFF  // 15 bits
#define DP2_SENSOR_RESOLUTION_SM9543		0x3FFF  // 14 bits
#define DP3_SENSOR_RESOLUTION_OTHER 		0x7FFF  // 15 bits
#define DP3_SENSOR_RESOLUTION_SM9543	    0x3FFF  // 14 bits

//static uint8_t GetSensorDP1_SM4331( int * pressure );
uint8_t GetSensorDP1_SM9543( int * pressure );
//static uint8_t GetSensorDP1_AMS5812( int * pressure );

//static uint8_t GetSensorDP2_SM4331( int * pressure );
uint8_t GetSensorDP2_SM9543( int * pressure );
//static uint8_t GetSensorDP2_AMS5812( int * pressure );

//static uint8_t GetSensorDP3_SM4331( int * pressure, int * temperature);
uint8_t GetSensorDP3_SM9543( int * pressure);
//static uint8_t GetSensorDP3_AMS5812( int * pressure, int * temperature);

// returns ERROR_OK or error code
//uint8_t GetSensorDP1( int * pressure);
//uint8_t GetSensorDP2( int * pressure );
//uint8_t GetSensorDP3( int * pressure, int * temperature);

#endif

