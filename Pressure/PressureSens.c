#include "SW_I2C1Interface.h"
#include "SW_I2C2Interface.h"
#include "SW_I2C3Interface.h"
#include "SW_I2C4Interface.h"
#include "PressureSens.h"
#include "error.h"
#include "UserInterface.h"

#define SENS_ADD_SM4331			(0x6C*2)
#define SENS_ADD_SM9543			(0x28*2)
#define SENS_ADD_AMS5812	    (0x78*2)

//uint8_t GetSensorDP1( int * pressure)
//{
	//switch( GetParameterValue(DP1_SENS_TYPE))
	//{
		//case PRES_SENS_SM4331:    return GetSensorDP1_SM4331( pressure );
		//
		//case PRES_SENS_SM9543:    return GetSensorDP1_SM9543( pressure );
		//
		//case PRES_SENS_AMS5812:   return GetSensorDP1_AMS5812( pressure );
		//
		//case PRES_SENS_NONE:
		//default:                  
			//*pressure = 0;
			//return ERROR_DP1_SENS_I2C;
	//}
//}
//
//uint8_t GetSensorDP2( int * pressure)
//{
	//switch( GetParameterValue(DP2_SENS_TYPE))
	//{ 
		//case PRES_SENS_SM4331:    return GetSensorDP2_SM4331( pressure );
//
		//case PRES_SENS_SM9543:    return GetSensorDP2_SM9543( pressure );
//
		//case PRES_SENS_AMS5812:   return GetSensorDP2_AMS5812( pressure );
//
		//case PRES_SENS_NONE:
		//default:                 
			//*pressure = 0;
			//*temperature = 0;
			//return ERROR_DP2_SENS_I2C;
	//}
//}
//
//uint8_t GetSensorDP3( int * pressure, int * temperature)
//{
	//switch( GetParameterValue(DP3_SENS_TYPE))
	//{
		//case PRES_SENS_SM4331:    return GetSensorDP3_SM4331( pressure, temperature );
//
		//case PRES_SENS_SM9543:    return GetSensorDP3_SM9543( pressure, temperature );
//
		//case PRES_SENS_AMS5812:   return GetSensorDP3_AMS5812( pressure, temperature );
//
		//case PRES_SENS_NONE:
		//default:
		//*pressure = 0;
		//*temperature = 0;
		//return ERROR_DP3_SENS_I2C;
	//}
//}

//static uint8_t GetSensorDP1_SM4331( int * pressure )
//{
	//uint8_t regNo = 0x30;
//
	//if( SW_I2CWriteReadBytes( SENS_ADD_SM4331, 1, &regNo, 2, (uint8_t *)pressure))
	//return ERROR_DP1_SENS_I2C;
//
	//if ( *pressure > 26215)
	//*pressure = 26215;
	//if ( *pressure < -26215)
	//*pressure = -26215;
	//*pressure = (long)(*pressure) * DP1_SENSOR_RESOLUTION_OTHER / 26215;
	//return ERROR_OK;
//}

//static uint8_t GetSensorDP2_SM4331( int * pressure)
//{
	//uint8_t regNo = 0x30;
//
	//if( SW_I2C3WriteReadBytes( SENS_ADD_SM4331, 1, &regNo, 2, (uint8_t *)pressure))
	//return ERROR_DP2_SENS_I2C;
//
	//if ( *pressure > 26215)
	//*pressure = 26215;
	//if ( *pressure < -26215)
	//*pressure = -26215;
	//*pressure = (long)(*pressure) * DP2_SENSOR_RESOLUTION_OTHER / 26215;
//
	//return ERROR_OK;
//}
//
//static uint8_t GetSensorDP3_SM4331( int * pressure, int * temperature)
//{
	//uint8_t regNo = 0x30;
//
	//if( SW_I2C4WriteReadBytes( SENS_ADD_SM4331, 1, &regNo, 2, (uint8_t *)pressure))
	//return ERROR_DP3_SENS_I2C;
//
	//if ( *pressure > 26215)
	//*pressure = 26215;
	//if ( *pressure < -26215)
	//*pressure = -26215;
	//*pressure = (long)(*pressure) * DP3_SENSOR_RESOLUTION_OTHER / 26215;
//
	//return ERROR_OK;
//}

uint8_t GetSensorDP1_SM9543( int * pressure )
{
	uint8_t data[2];

	if( SW_I2C3ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP1_SENS_I2C;
	
	*pressure = data[0];
	*pressure<<=8;
	*pressure |= data[1];
	*pressure &= 0x3FFF;
	
	//*pressure = *pressure - 1638;

	//if ( *pressure > 14745)
	//*pressure = 14745;
	//if ( *pressure < 1638)
	//*pressure = 1638;
    
	//*pressure = (long)(*pressure) * DP1__SENSOR_RESOLUTION_SM9543 / 1638;
	//*pressure = *pressure - (DP1__SENSOR_RESOLUTION_SM9543 / 2);
				
	return ERROR_OK;
}

uint8_t GetSensorDP2_SM9543( int * pressure )
{
	uint8_t data[2];

	if( SW_I2C4ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP2_SENS_I2C;
	
	*pressure = data[0];
	*pressure<<=8;
	*pressure |= data[1];
	*pressure &= 0x3FFF;
	
	//*pressure = *pressure - 1638;

	//if ( *pressure > 14745)
	//*pressure = 14745;
	//if ( *pressure < 1638)
	//*pressure = 1638;
    
	//*pressure = (long)(*pressure) * DP2_SENSOR_RESOLUTION_SM9543 / 1638;
	//*pressure = *pressure - (DP2_SENSOR_RESOLUTION_SM9543 / 2);
				
	return ERROR_OK;
}

uint8_t GetSensorDP3_SM9543( int *pressure)
{
	uint8_t data[2]={0};
	//long temp1=0;
		
	//*pressure = 0;
	//*temperature = 0;
	
	if( SW_I2C2ReadBytes( SENS_ADD_SM9543, 2, (uint8_t *)&data[0]))
	return ERROR_DP3_SENS_I2C;
	
	*pressure = data[0];
	*pressure <<= 8;
	*pressure |= data[1];
	*pressure &= 0x3FFF;
	
	//*pressure = *pressure - 1638;
	
	//temp1 = data[2];
	//temp1 <<= 8;
	//temp1 |= data[3];
	//
	//temp1 = (temp1 - 1638.0)*100;
	//temp1 /= 187.24;
	//temp1 /= 10;
	//*temperature = temp1-500;
	
	//float temp = ((data[2] & 0xFF) * 256 + (data[3] & 0xFF));
	//float cTemp = ((*temperature - 1638.0) / 187.24) - 5.0;
	
	
	//*temperature = cTemp * 100;
	 
	 
	//*temperature = (data[2]<<3) | (data[3]>>5);
	
	//*temperature = data[2];
	//*temperature <<= 8;
	//*temperature |= data[3];
	//*temperature &= 0xFFE0;
	//*temperature >>= 5;
	//
	//temp1 = *temperature;
	//temp1 *= 200;
	//temp1 /= 2047;
	//temp1 -= 50;
	//
	//*temperature = temp1;
	//*temperature *= 100;
	
				
	return ERROR_OK;
}

//static uint8_t GetSensorDP1_AMS5812( int * pressure )
//{
	//uint8_t data[2];
	//
	//if(SW_I2CReadBytes(SENS_ADD_AMS5812, 2, (uint8_t *)&data[0]))
	//return ERROR_DP1_SENS_I2C;
	//
	//*pressure = data[0];
	//*pressure<<=8;
	//*pressure |= data[1];
	//*pressure &= 0x7FFF;
	//
	////data = data & 0xffff;
	////*pressure = data >> 8;
	////*pressure = *pressure | (data << 8);
		//
	//*pressure = *pressure - 3277;
	//if (*pressure > 26214) *pressure = 26214;
	////*pressure -= 13107;
	////
	////*pressure = (long)(*pressure) * DP1__SENSOR_RESOLUTION / 13107;
	//
	//return ERROR_OK;
//}
//
//static uint8_t GetSensorDP2_AMS5812( int * pressure )
//{
	//uint8_t data[4];
	//
	//if(SW_I2C3ReadBytes(SENS_ADD_AMS5812, 2, (uint8_t *)&data[0]))
	//return ERROR_DP2_SENS_I2C;
	//
	//*pressure = data[0];
	//*pressure<<=8;
	//*pressure |= data[1];
	//*pressure &= 0x7FFF;
	//
	////data = data & 0xffff;
	////*pressure = data >> 8;
	////*pressure = *pressure | (data << 8);
	//
	//*pressure = *pressure - 3277;
	//if (*pressure > 26214) *pressure = 26214;
	////*pressure -= 13107;
	////
	////*pressure = (long)(*pressure) * DP2_SENSOR_RESOLUTION / 13107;
	//
	//return ERROR_OK;
//}
//
//static uint8_t GetSensorDP3_AMS5812( int * pressure, int * temperature)
//{
	//uint8_t data[4];
	//
	//if(SW_I2C3ReadBytes(SENS_ADD_AMS5812, 2, (uint8_t *)&data[0]))
	//return ERROR_DP3_SENS_I2C;
	//
	//*pressure = data[0];
	//*pressure<<=8;
	//*pressure |= data[1];
	//*pressure &= 0x7FFF;
	//
	////data = data & 0xffff;
	////*pressure = data >> 8;
	////*pressure = *pressure | (data << 8);
	//
	//*pressure = *pressure - 3277;
	//if (*pressure > 26214) *pressure = 26214;
	////*pressure -= 13107;
	////
	////*pressure = (long)(*pressure) * DP2_SENSOR_RESOLUTION / 13107;
	//
	//return ERROR_OK;
//}


