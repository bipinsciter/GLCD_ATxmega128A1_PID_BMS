#ifndef __TEMP_RH_SENS__
#define __TEMP_RH_SENS__

uint8_t GetSensorSHT25Temperature( uint8_t SensNo, unsigned int * temperature, unsigned char * checksum );
uint8_t GetSensorSHT25Humidity( uint8_t SensNo, unsigned int * humidity, unsigned char * checksum );

uint8_t GetSensorSHT35Readings( uint8_t SensNo, unsigned int * temperature, unsigned int * humidity );
uint8_t ResetSensorSHT35(uint8_t SensNo);
uint8_t StartSensorSHT35(uint8_t SensNo);

uint8_t GetSensorReadings_IDT_HS3100( uint8_t SensNo, unsigned int * temperature, unsigned int * humidity );
uint8_t StartSensor_IDT_HS3100( uint8_t SensNo );

#endif
