#ifndef __DEVICEIO_H__
#define __DEVICEIO_H__

#include <avr/io.h>

#include "PressureSens.h"

#define OUTPUT_PORT        PORTA_OUT
#define OUTPUT_PORT_DIR    PORTA_DIR
#define OUTPUT1_HIGH	   PORTA_OUTSET = _BV(0)
#define OUTPUT1_LOW	       PORTA_OUTCLR = _BV(0)
#define OUTPUT2_HIGH	   PORTA_OUTSET = _BV(1)
#define OUTPUT2_LOW	       PORTA_OUTCLR = _BV(1)
#define OUTPUT3_HIGH	   PORTA_OUTSET = _BV(2)
#define OUTPUT3_LOW	       PORTA_OUTCLR = _BV(2)
#define OUTPUT4_HIGH	   PORTA_OUTSET = _BV(3)
#define OUTPUT4_LOW	       PORTA_OUTCLR = _BV(3)
//#define OUTPUT5_HIGH	   PORTA_OUTSET = _BV(4)
//#define OUTPUT5_LOW	       PORTA_OUTCLR = _BV(4)
//#define OUTPUT6_HIGH	   PORTA_OUTSET = _BV(5)
//#define OUTPUT6_LOW	       PORTA_OUTCLR = _BV(5)
//#define OUTPUT7_HIGH	   PORTA_OUTSET = _BV(6)
//#define OUTPUT7_LOW	       PORTA_OUTCLR = _BV(6)
//#define OUTPUT8_HIGH	   PORTA_OUTSET = _BV(7)
//#define OUTPUT8_LOW	       PORTA_OUTCLR = _BV(7)

#define DOOR_SENSE		(PORTB_IN & PIN3_bm)

#define INPUT1_SENSE	(PORTK_IN & PIN0_bm)
#define INPUT2_SENSE	(PORTK_IN & PIN1_bm)
#define INPUT3_SENSE	(PORTK_IN & PIN2_bm)
#define INPUT4_SENSE	(PORTK_IN & PIN3_bm)
#define INPUT5_SENSE	(PORTK_IN & PIN4_bm)

#define INPUT_PORT        PORTK_OUT
#define INPUT_PORT_DIR    PORTK_DIR
#define INPUT_PORT_IN     PORTK_IN

#define LED_ABD_PRES_OK_OFF		PORTQ_OUTSET = _BV(2)
#define LED_ABD_PRES_OK_ON		PORTQ_OUTCLR = _BV(2)
#define LED_ABD_PRES_OK_DIR_OUT	PORTQ_DIRSET = _BV(2)

#define LED_ABD_PRES_ALM_OFF		PORTQ_OUTSET = _BV(3)
#define LED_ABD_PRES_ALM_ON			PORTQ_OUTCLR = _BV(3)
#define LED_ABD_PRES_ALM_DIR_OUT	PORTQ_DIRSET = _BV(3)

#define LED_DP2_OK_OFF		PORTF_OUTSET = _BV(5)
#define LED_DP2_OK_ON		PORTF_OUTCLR = _BV(5)
#define LED_DP2_OK_DIR_OUT	PORTF_DIRSET = _BV(5)

#define LED_DP2_ALM_OFF		PORTF_OUTSET = _BV(4)
#define LED_DP2_ALM_ON		PORTF_OUTCLR = _BV(4)
#define LED_DP2_ALM_DIR_OUT	PORTF_DIRSET = _BV(4)

#define LED_TEMP_OK_OFF		PORTF_OUTSET = _BV(3)
#define LED_TEMP_OK_ON		PORTF_OUTCLR = _BV(3)
#define LED_TEMP_OK_DIR_OUT	PORTF_DIRSET = _BV(3)

#define LED_TEMP_ALM_OFF		PORTF_OUTSET = _BV(2)
#define LED_TEMP_ALM_ON			PORTF_OUTCLR = _BV(2)
#define LED_TEMP_ALM_DIR_OUT	PORTF_DIRSET = _BV(2)

#define LED_HUM_OK_OFF		PORTF_OUTSET = _BV(7)
#define LED_HUM_OK_ON		PORTF_OUTCLR = _BV(7)
#define LED_HUM_OK_DIR_OUT	PORTF_DIRSET = _BV(7)

#define LED_HUM_ALM_OFF		PORTF_OUTSET = _BV(6)
#define LED_HUM_ALM_ON		PORTF_OUTCLR = _BV(6)
#define LED_HUM_ALM_DIR_OUT	PORTF_DIRSET = _BV(6)

#define LED_ALM1_OFF		PORTK_OUTSET = _BV(5)
#define LED_ALM1_ON			PORTK_OUTCLR = _BV(5)
#define LED_ALM1_DIR_OUT	PORTK_DIRSET = _BV(5)

#define LED_ALM2_OFF		PORTK_OUTSET = _BV(6)
#define LED_ALM2_ON			PORTK_OUTCLR = _BV(6)
#define LED_ALM2_DIR_OUT	PORTK_DIRSET = _BV(6)

#define LED_ALM3_OFF		PORTK_OUTSET = _BV(7)
#define LED_ALM3_ON			PORTK_OUTCLR = _BV(7)
#define LED_ALM3_DIR_OUT	PORTK_DIRSET = _BV(7)

#define LED_ALM4_OFF		PORTA_OUTSET = _BV(4)
#define LED_ALM4_ON			PORTA_OUTCLR = _BV(4)
#define LED_ALM4_DIR_OUT	PORTA_DIRSET = _BV(4)


#define CLOSE	0
#define OPEN	1

typedef struct
{
   uint8_t errorCode;
   uint16_t rawValue;
   int      convertedValue;
   //int      FinalValue;
}SENSOR_VALUE;

typedef union
{
	uint16_t alarmByte;
	struct
	{
		uint16_t absPressHigh    :1;  //LSB
		uint16_t absPressLow     :1;
		uint16_t diffPressHigh   :1;
		uint16_t diffPressLow    :1;
		uint16_t tempHigh        :1;
		uint16_t tempLow         :1;
		uint16_t humidityHigh    :1;
		uint16_t humidityLow     :1;
		uint16_t diffPress3High  :1;
		uint16_t diffPress3Low   :1;
		uint16_t fire   :1;
		uint16_t door   :1;
	}alarm;
}ALARMS;

typedef union
{
	uint16_t ledByte;
	struct
	{
		unsigned absPressOk       :1;
		unsigned absPressAlarm    :1;
		unsigned diffPressOk      :1;
		unsigned diffPressAlarm   :1;
		unsigned tempOk           :1;
		unsigned tempAlarm        :1;
		unsigned humidityOk       :1;
		unsigned humidityAlarm    :1;
		unsigned diffPress3Ok     :1;
		unsigned diffPress3Alarm  :1;
	}led;
}LEDS;

#define ALARM_UPPER_DP1    0
#define ALARM_LOWER_DP1    1
#define ALARM_UPPER_DP2    2
#define ALARM_LOWER_DP2    3
#define ALARM_UPPER_TEMPERATURE     4
#define ALARM_LOWER_TEMPERATURE     5
#define ALARM_UPPER_HUMIDITY        6
#define ALARM_LOWER_HUMIDITY        7
#define ALARM_UPPER_DP3   8
#define ALARM_LOWER_DP3   9

#define DP1_VAL_INDEX      0
#define DP2_VAL_INDEX     1
#define DP3_VAL_INDEX    2
#define TEMPERATURE_VAL_INDEX       3
#define TEMPERATURE2_VAL_INDEX      4
#define HUMIDITY_VAL_INDEX          5
#define HUMIDITY2_VAL_INDEX          6
#define TOTAL_VAL_INDEX      7

void DeviceIOInit();

void GetDP1( SENSOR_VALUE * value );
void GetDP2( SENSOR_VALUE * value );
void GetDP3( SENSOR_VALUE * value );
void GetTemperature( SENSOR_VALUE * value );
void GetTemperature2( SENSOR_VALUE * value );
void GetHumidity( SENSOR_VALUE * value );
void GetPareValue(uint8_t ParaNumber, SENSOR_VALUE * value );

uint8_t GetSystemError();
uint16_t GetAlarms();

void SetOutput( uint8_t outValue, uint8_t* retVal );
void GetOutput( uint8_t* retVal );
void GetInput( uint8_t* retVal );


unsigned int AlarmActive_Buzzer();
void SetBuzzerSuppressionReq();
int GetAndResetBuzzerSuppressionReq();
uint8_t CurrentDoorStatus(void);

#endif
