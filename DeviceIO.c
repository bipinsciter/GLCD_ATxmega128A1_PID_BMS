#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <stdlib.h>
#include <avr/wdt.h>
#include "os_api.h"
#include "error.h"
#include "DeviceIO.h"
#include "HardwareInfo.h"
#include "TempRHSens.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DAC7573.h"
#include "locked_printf.h"
#include "PressureSens.h"
#include "eepromAdr.h"
//#include "ParticalSenser_IPS7100.h"
#include "ParticalSenser_SEN55.h"

#define TRUE  1
#define FALSE 0

#define SENSOR_1  0
#define SENSOR_2  1

#define DEVICEIO_STACK_SIZE   300
#define DEVICEIO_PRIORITY     1
#define DEVICEIO_FUNC_NAME    DeviceIO

#define TEMPRHIO_STACK_SIZE   300
#define TEMPRHIO_PRIORITY     2//3
#define TEMPRHIO_FUNC_NAME    TempRHIO

//#define PARTSENS_STACK_SIZE   200
//#define PARTSENS_PRIORITY     3//1
//#define PARTSENS_FUNC_NAME    PartSensIO

static SENSOR_VALUE sensorVal[TOTAL_VAL_INDEX];
//static sen5x_measured_value_t sen55_1,sen55_2;
//static ips7100_pc_value_t isp7100_pc_1,isp7100_pc_2;
//static ips7100_pm_value_t isp7100_pm_1,isp7100_pm_2;

//static int lastval1=0,lastval2=0,lastval3=0;
//static uint8_t checktime1=0,checktime2=0,checktime3=0;

// Kalman filter structure
typedef struct {
	float Q;       // Process noise covariance
	float R;       // Measurement noise covariance
	float X;       // State estimate
	float P;       // Estimate covariance
	float K;       // Kalman gain
} KalmanFilter;
static KalmanFilter Kalmanfilter[TOTAL_VAL_INDEX] = {0};
	
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_estimate);
int Kalman_Update(KalmanFilter *kf, float measurement);

// Initialize the Kalman filter
void Kalman_Init(KalmanFilter *kf, float q, float r, float initial_estimate)
{
	kf->Q = q;          // Process noise covariance
	kf->R = r;          // Measurement noise covariance
	kf->X = initial_estimate;  // Initial estimate
	kf->P = 1.0;        // Initial estimate covariance
}

// Update the Kalman filter with a new measurement
int Kalman_Update(KalmanFilter *kf, float measurement)
{
	// Prediction update
	kf->P += kf->Q;

	// Measurement update
	kf->K = kf->P / (kf->P + kf->R);
	kf->X += kf->K * (measurement - kf->X);
	kf->P *= (1 - kf->K);

	return (int)kf->X;
}


	
static OSSemaMutex DeviceValueMutex;

static void AveragePara( uint8_t SenNo, uint8_t error, unsigned int rawVal, int convertedVal );

static void DoIOPortMapping();

static ALARMS alarmOut;
static ALARMS oldAlarmOut;
static ALARMS buzzerOut;
static LEDS ledState;
static SENSOR_VALUE senVal;
//static sen5x_measured_value_t sen55;

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

/*
   Device IO Task
*/
#ifdef OS_AVRX
TimerControlBlock devIOLoopSleepTimer;
AVRX_GCC_TASKDEF(DEVICEIO_FUNC_NAME, DEVICEIO_STACK_SIZE, DEVICEIO_PRIORITY)
#else
void DEVICEIO_FUNC_NAME( void * taskPara );
void DEVICEIO_FUNC_NAME( void * taskPara )
#endif
{
	uint8_t retVal;
	//uint16_t value1=0;
	int value;//, temprValue;
	int convertedValue;
	static uint16_t TempfireAlmTimer=0;
	
	ledState.ledByte = 0;
	oldAlarmOut.alarmByte = alarmOut.alarmByte = 0;
	LED_ABD_PRES_OK_DIR_OUT;
	LED_ABD_PRES_ALM_DIR_OUT;
	LED_DP2_OK_DIR_OUT;
	LED_DP2_ALM_DIR_OUT;
	LED_TEMP_OK_DIR_OUT;
	LED_TEMP_ALM_DIR_OUT;
	LED_HUM_OK_DIR_OUT;
	LED_HUM_ALM_DIR_OUT;
	
	//LED_ABD_PRES_OK_ON;
	//LED_ABD_PRES_OK_OFF;
	//LED_ABD_PRES_ALM_ON;
	//LED_ABD_PRES_ALM_OFF;
	//LED_DP2_OK_ON;
	//LED_DP2_OK_OFF;
	//LED_DP2_ALM_ON;
	//LED_DP2_ALM_OFF;
	//LED_TEMP_OK_ON;
	//LED_TEMP_OK_OFF;
	//LED_TEMP_ALM_ON;
	//LED_TEMP_ALM_OFF;
	//LED_HUM_OK_ON;
	//LED_HUM_OK_OFF;
	//LED_HUM_ALM_ON;
	//LED_HUM_ALM_OFF;
	
	OUTPUT_PORT_DIR = 0xFF;  // As outputs 
	
	uint16_t Output;
	
	Output = OSReadEEPromWord((unsigned int *)EA_OUTPUT);
	if(Output > 0x0F)
	{
		
	}
	else
	{
		SetOutput( (uint8_t)Output, (uint8_t*)&Output );
	}
	
	//OUTPUT_PORT = 0xE0;      // set high

	INPUT_PORT_DIR = 0xE0;  // As inputs
	INPUT_PORT = 0xFF;      // Pullups on
	
	//while(1)
	//{
		//PORTA_OUTTGL = _BV(0);
		//wdt_reset();			//Serve Watchdog Timer
		//OSSleep(1000);
	//}
	
	// Kalman filter for pressure
	Kalman_Init(&Kalmanfilter[DP1_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalmanfilter[DP2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalmanfilter[DP3_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values

	while (1)
	{
		//PORTA_OUTTGL = _BV(0);
		wdt_reset();			//Serve Watchdog Timer
	   
		systemError = ERROR_OK;

		if (IsDP1Enabled())
		{
			retVal = GetSensorDP1_SM9543( &value );
		  
			// Convert the value to PSI
			convertedValue = value + GetParameterValue(DP1_ZERO_ADJ);

			//if((abs(lastval1-sensorVal[DP1_VAL_INDEX].convertedValue)>GetParameterValue(DP1_SENS_MIN)) && (abs(lastval1-sensorVal[DP1_VAL_INDEX].convertedValue)<GetParameterValue(DP1_SENS_MAX)))
			//{
				//sensorVal[DP1_VAL_INDEX].convertedValue=lastval1;
			//}
			//else
			//{
				//lastval1=sensorVal[DP1_VAL_INDEX].convertedValue;
			//}
			
			AveragePara(DP1_VAL_INDEX, retVal, value, convertedValue );
		}
		
		if (IsDP2Enabled())
		{
			retVal = GetSensorDP2_SM9543( &value );
		 
			// Convert the value to PSI
			convertedValue = value + GetParameterValue(DP2_ZERO_ADJ);

			//if((abs(lastval2-sensorVal[DP2_VAL_INDEX].convertedValue)>GetParameterValue(DP2_SENS_MIN)) && (abs(lastval2-sensorVal[DP2_VAL_INDEX].convertedValue)<GetParameterValue(DP2_SENS_MAX)))
			//{
				//sensorVal[DP2_VAL_INDEX].convertedValue=lastval2;
			//}
			//else
			//{
				//lastval2=sensorVal[DP2_VAL_INDEX].convertedValue;
			//}

			AveragePara(DP2_VAL_INDEX, retVal, value, convertedValue );
		}
	  
		if (IsDP3Enabled())
		{
			retVal = GetSensorDP3_SM9543( &value );

			// Convert the value to PSI
			convertedValue = value + GetParameterValue(DP3_ZERO_ADJ);

			//if((abs(lastval3-sensorVal[DP3_VAL_INDEX].convertedValue)>GetParameterValue(DP3_SENS_MIN)) && (abs(lastval3-sensorVal[DP3_VAL_INDEX].convertedValue)<GetParameterValue(DP3_SENS_MAX)))
			//{
				//sensorVal[DP3_VAL_INDEX].convertedValue=lastval3;
			//}
			//else
			//{
				//lastval3=sensorVal[DP3_VAL_INDEX].convertedValue;
			//}
			
			AveragePara(DP3_VAL_INDEX, retVal, value, convertedValue );
		}
		
		oldAlarmOut.alarmByte = alarmOut.alarmByte;
		if (IsAlarmEnabled())
		{
			if( IsDP1Enabled() && GetParameterValue(DP1_ALARM_SETUP) != ALARM_DISABLED)
			{
				GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue < GetParameterValue(DP1_LOWER_ALARM_ON_LIMIT))
						alarmOut.alarm.absPressLow = 1;
					if( senVal.convertedValue > GetParameterValue(DP1_UPPER_ALARM_ON_LIMIT))
						alarmOut.alarm.absPressHigh = 1;
					if( senVal.convertedValue > GetParameterValue(DP1_LOWER_ALARM_OFF_LIMIT))
						alarmOut.alarm.absPressLow = 0;
					if( senVal.convertedValue < GetParameterValue(DP1_UPPER_ALARM_OFF_LIMIT))
						alarmOut.alarm.absPressHigh = 0;

					ledState.led.absPressAlarm = alarmOut.alarm.absPressLow | alarmOut.alarm.absPressHigh;
					ledState.led.absPressOk = ~alarmOut.alarm.absPressHigh;
				}
				else
				{
					ledState.led.absPressOk = ledState.led.absPressAlarm = 0;
				}
			}
			else
			{
				ledState.led.absPressOk = ledState.led.absPressAlarm = 0;
				alarmOut.alarm.absPressLow = 0;
				alarmOut.alarm.absPressHigh = 0;
			}

			if( IsDP2Enabled() && GetParameterValue(DP2_ALARM_SETUP) != ALARM_DISABLED)
			{
				GetPareValue(DP2_VAL_INDEX, &senVal);//GetDP2( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue < GetParameterValue(DP2_LOWER_ALARM_ON_LIMIT))
					alarmOut.alarm.diffPressLow = 1;
					if( senVal.convertedValue > GetParameterValue(DP2_UPPER_ALARM_ON_LIMIT))
					alarmOut.alarm.diffPressHigh = 1;
					if( senVal.convertedValue > GetParameterValue(DP2_LOWER_ALARM_OFF_LIMIT))
					alarmOut.alarm.diffPressLow = 0;
					if( senVal.convertedValue < GetParameterValue(DP2_UPPER_ALARM_OFF_LIMIT))
					alarmOut.alarm.diffPressHigh = 0;

					ledState.led.diffPressAlarm = alarmOut.alarm.diffPressHigh | alarmOut.alarm.diffPressLow;
					ledState.led.diffPressOk = ~alarmOut.alarm.diffPressHigh;
				}
				else
				{
					ledState.led.diffPressOk = ledState.led.diffPressAlarm = 0;
				}
			}
			else
			{
				ledState.led.diffPressOk = ledState.led.diffPressAlarm = 0;
				alarmOut.alarm.diffPressLow = 0;
				alarmOut.alarm.diffPressHigh = 0;
			}
			
			
			if( IsDP3Enabled() && GetParameterValue(DP3_ALARM_SETUP) != ALARM_DISABLED)
			{
				GetPareValue(DP3_VAL_INDEX, &senVal);//GetDP3( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue < GetParameterValue(DP3_LOWER_ALARM_ON_LIMIT))
					alarmOut.alarm.diffPress3Low = 1;
					if( senVal.convertedValue > GetParameterValue(DP3_UPPER_ALARM_ON_LIMIT))
					alarmOut.alarm.diffPress3High = 1;
					if( senVal.convertedValue > GetParameterValue(DP3_LOWER_ALARM_OFF_LIMIT))
					alarmOut.alarm.diffPress3Low = 0;
					if( senVal.convertedValue < GetParameterValue(DP3_UPPER_ALARM_OFF_LIMIT))
					alarmOut.alarm.diffPress3High = 0;

					ledState.led.diffPress3Alarm = alarmOut.alarm.diffPress3High | alarmOut.alarm.diffPress3Low;
					ledState.led.diffPress3Ok = ~alarmOut.alarm.diffPress3High;
				}
				else
				{
					ledState.led.diffPress3Ok = ledState.led.diffPress3Alarm = 0;
				}
			}
			else
			{
				ledState.led.diffPress3Ok = ledState.led.diffPress3Alarm = 0;
				alarmOut.alarm.diffPress3Low = 0;
				alarmOut.alarm.diffPress3High = 0;
			}
			
			//if (IsTemperatureEnabled() && GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED)
			if (IsTemperatureEnabled())
			{
				GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if(GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED)
					{
						if( senVal.convertedValue < GetParameterValue(TEMP_LOWER_ALARM_ON_LIMIT))
						alarmOut.alarm.tempLow = 1;
						if( senVal.convertedValue > GetParameterValue(TEMP_UPPER_ALARM_ON_LIMIT))
						alarmOut.alarm.tempHigh = 1;
						if( senVal.convertedValue > GetParameterValue(TEMP_LOWER_ALARM_OFF_LIMIT))
						alarmOut.alarm.tempLow = 0;
						if( senVal.convertedValue < GetParameterValue(TEMP_UPPER_ALARM_OFF_LIMIT))
						alarmOut.alarm.tempHigh = 0;

						ledState.led.tempAlarm = alarmOut.alarm.tempLow | alarmOut.alarm.tempHigh;
						ledState.led.tempOk = ~alarmOut.alarm.tempHigh;
					}
					else
					{
						ledState.led.tempOk = ledState.led.tempAlarm = 0;
					}
					
					if( senVal.convertedValue >= GetParameterValue(TEMP_FIRE_ALM_SET))
					{
						TempfireAlmTimer++;
						if(TempfireAlmTimer>=GetParameterValue(TEMP_FIRE_ALM_TIME))
						{
							alarmOut.alarm.fire = 1;
							TempfireAlmTimer=0;
						}
					}
					else
					{
						TempfireAlmTimer=0;
						alarmOut.alarm.fire = 0;
					}
				}
				else
				{
					ledState.led.tempOk = ledState.led.tempAlarm = 0;
					alarmOut.alarm.fire = 0;
				}
			}
			else
			{
				ledState.led.tempOk = ledState.led.tempAlarm = 0;
				alarmOut.alarm.tempLow = 0;
				alarmOut.alarm.tempHigh = 0;
				alarmOut.alarm.fire = 0;
			}

			if (IsHumidityEnabled() && GetParameterValue(RH_ALARM_SETUP) != ALARM_DISABLED)
			{
				GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue < GetParameterValue(RH_LOWER_ALARM_ON_LIMIT))
					alarmOut.alarm.humidityLow = 1;
					if( senVal.convertedValue > GetParameterValue(RH_UPPER_ALARM_ON_LIMIT))
					alarmOut.alarm.humidityHigh = 1;
					if( senVal.convertedValue > GetParameterValue(RH_LOWER_ALARM_OFF_LIMIT))
					alarmOut.alarm.humidityLow = 0;
					if( senVal.convertedValue < GetParameterValue(RH_UPPER_ALARM_OFF_LIMIT))
					alarmOut.alarm.humidityHigh = 0;

					ledState.led.humidityAlarm = alarmOut.alarm.humidityLow | alarmOut.alarm.humidityHigh;
					ledState.led.humidityOk = ~alarmOut.alarm.humidityHigh;
				}
				else
				{
					ledState.led.humidityOk = ledState.led.humidityAlarm = 0;
				}
			}
			else
			{
				ledState.led.humidityOk = ledState.led.humidityAlarm = 0;
				alarmOut.alarm.humidityLow = 0;
				alarmOut.alarm.humidityHigh = 0;
			}
			
			if( IsSerialEnabled() && IsBroadcastEnabled() && oldAlarmOut.alarmByte != alarmOut.alarmByte )
			{
				oldAlarmOut.alarmByte ^= alarmOut.alarmByte;
				if( oldAlarmOut.alarm.absPressHigh )
				{  // Change in state
					SerialPrintAlarm( ALARM_UPPER_DP1, alarmOut.alarm.absPressHigh );
				}
				if( oldAlarmOut.alarm.absPressLow )
				{  // Change in state
					SerialPrintAlarm( ALARM_LOWER_DP1, alarmOut.alarm.absPressLow );
				}
				if( oldAlarmOut.alarm.diffPressHigh )
				{  // Change in state
					SerialPrintAlarm( ALARM_UPPER_DP2, alarmOut.alarm.diffPressHigh );
				}
				if( oldAlarmOut.alarm.diffPressLow )
				{  // Change in state
					SerialPrintAlarm( ALARM_LOWER_DP2, alarmOut.alarm.diffPressLow );
				}
				if( oldAlarmOut.alarm.diffPress3High )
				{  // Change in state
					SerialPrintAlarm( ALARM_UPPER_DP3, alarmOut.alarm.diffPress3High );
				}
				if( oldAlarmOut.alarm.diffPress3Low )
				{  // Change in state
					SerialPrintAlarm( ALARM_LOWER_DP3, alarmOut.alarm.diffPress3Low );
				}
				if( oldAlarmOut.alarm.tempHigh )
				{  // Change in state
					SerialPrintAlarm( ALARM_UPPER_TEMPERATURE, alarmOut.alarm.tempHigh );
				}
				if( oldAlarmOut.alarm.tempLow )
				{  // Change in state
					SerialPrintAlarm( ALARM_LOWER_TEMPERATURE, alarmOut.alarm.tempLow );
				}
				if( oldAlarmOut.alarm.humidityHigh )
				{  // Change in state
					SerialPrintAlarm( ALARM_UPPER_HUMIDITY, alarmOut.alarm.humidityHigh );
				}
				if( oldAlarmOut.alarm.humidityLow )
				{  // Change in state
					SerialPrintAlarm( ALARM_LOWER_HUMIDITY, alarmOut.alarm.humidityLow );
				}
			}

			//Set buzzer and relay out values based on alarm setup parameter
			if( GetParameterValue(DP1_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
			{
				buzzerOut.alarm.absPressHigh = alarmOut.alarm.absPressHigh;
				buzzerOut.alarm.absPressLow = alarmOut.alarm.absPressLow;
			}
			else
			{
				buzzerOut.alarm.absPressHigh = 0;
				buzzerOut.alarm.absPressLow = 0;
			}
		 
			if( GetParameterValue(DP2_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
			{
				buzzerOut.alarm.diffPressHigh = alarmOut.alarm.diffPressHigh;
				buzzerOut.alarm.diffPressLow = alarmOut.alarm.diffPressLow;
			}
			else
			{
				buzzerOut.alarm.diffPressHigh = 0;
				buzzerOut.alarm.diffPressLow = 0;
			}
			
			if( GetParameterValue(DP3_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
			{
				buzzerOut.alarm.diffPress3High = alarmOut.alarm.diffPress3High;
				buzzerOut.alarm.diffPress3Low = alarmOut.alarm.diffPress3Low;
			}
			else
			{
				buzzerOut.alarm.diffPress3High = 0;
				buzzerOut.alarm.diffPress3Low = 0;
			}
			
			if( GetParameterValue(TEMP_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
			{
				buzzerOut.alarm.tempHigh = alarmOut.alarm.tempHigh;
				buzzerOut.alarm.tempLow = alarmOut.alarm.tempLow;
			}
			else
			{
				buzzerOut.alarm.tempHigh = 0;
				buzzerOut.alarm.tempLow = 0;
			}
			
			if( GetParameterValue(RH_ALARM_SETUP) == ALARM_ENB_BUZ_ON )
			{
				buzzerOut.alarm.humidityHigh = alarmOut.alarm.humidityHigh;
				buzzerOut.alarm.humidityLow = alarmOut.alarm.humidityLow;
			}
			else
			{
				buzzerOut.alarm.humidityHigh = 0;
				buzzerOut.alarm.humidityLow = 0;
			}
			
			buzzerOut.alarm.fire = alarmOut.alarm.fire;
			buzzerOut.alarm.door = alarmOut.alarm.door;
		}

		//LED_PORT = ~ledState.ledByte;
		if(ledState.led.absPressOk) LED_ABD_PRES_OK_ON; else LED_ABD_PRES_OK_OFF;
		if(ledState.led.absPressAlarm) LED_ABD_PRES_ALM_ON; else LED_ABD_PRES_ALM_OFF;
		if(ledState.led.diffPressOk) LED_DP2_OK_ON; else LED_DP2_OK_OFF;
		if(ledState.led.diffPressAlarm) LED_DP2_ALM_ON; else LED_DP2_ALM_OFF;
		if(ledState.led.tempOk) LED_TEMP_OK_ON; else LED_TEMP_OK_OFF;
		if(ledState.led.tempAlarm) LED_TEMP_ALM_ON; else LED_TEMP_ALM_OFF;
		if(ledState.led.humidityOk) LED_HUM_OK_ON; else LED_HUM_OK_OFF;
		if(ledState.led.humidityAlarm) LED_HUM_ALM_ON; else LED_HUM_ALM_OFF;

		if (IsAlalogOutEnabled())
		{
			if( IsDP1Enabled() && (GetParameterValue(DP1_OUTPUT_TYPE) == 1 || IsPIDEnabled() == 0))
			{
				GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue <= GetParameterValue(DP1_OUT_LOW_PRES) )
						value = GetParameterValue(DP1_OUT_LOW_COUNT);
					else if( senVal.convertedValue >= GetParameterValue(DP1_OUT_HIGH_PRES) )
						value = GetParameterValue(DP1_OUT_HIGH_COUNT);
					else
					{
						value = GetParameterValue(DP1_OUT_HIGH_PRES) - GetParameterValue(DP1_OUT_LOW_PRES);
						convertedValue = GetParameterValue(DP1_OUT_HIGH_COUNT) - GetParameterValue(DP1_OUT_LOW_COUNT);
						value = (((unsigned long)(senVal.convertedValue - GetParameterValue(DP1_OUT_LOW_PRES))) * convertedValue) / value;
						value += GetParameterValue(DP1_OUT_LOW_COUNT);
					}
					
					if( ! GetParameterValue(DP1_OUT_POLARITY))
					{
						value -= GetParameterValue(DP1_OUT_LOW_COUNT);
						value = GetParameterValue(DP1_OUT_HIGH_COUNT) - GetParameterValue(DP1_OUT_LOW_COUNT) - value;
						value += GetParameterValue(DP1_OUT_LOW_COUNT);
					}
				}
				else
				{
					// write zero
					value = 0;
				}
				
				// Write to DAC
				convertedValue = DACWriteOutValue( DP1_ANALOG_OUT, value );
				if( convertedValue != ERROR_OK )
				systemError = convertedValue;
			}
			
			
			if( IsDP2Enabled() && (GetParameterValue(DP2_OUTPUT_TYPE) == 1 || IsPIDEnabled() == 0) )
			{
				GetPareValue(DP2_VAL_INDEX, &senVal);//GetDP2( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue <= GetParameterValue(DP2_OUT_LOW_PRES) )
						value = GetParameterValue(DP2_OUT_LOW_COUNT);
					else if( senVal.convertedValue >= GetParameterValue(DP2_OUT_HIGH_PRES) )
						value = GetParameterValue(DP2_OUT_HIGH_COUNT);
					else
					{
						value = GetParameterValue(DP2_OUT_HIGH_PRES) - GetParameterValue(DP2_OUT_LOW_PRES);
						convertedValue = GetParameterValue(DP2_OUT_HIGH_COUNT) - GetParameterValue(DP2_OUT_LOW_COUNT);
						value = (((unsigned long)(senVal.convertedValue - GetParameterValue(DP2_OUT_LOW_PRES))) * convertedValue) / value;
						value += GetParameterValue(DP2_OUT_LOW_COUNT);
					}
					
					if( ! GetParameterValue(DP2_OUT_POLARITY))
					{
						value -= GetParameterValue(DP2_OUT_LOW_COUNT);
						value = GetParameterValue(DP2_OUT_HIGH_COUNT) - GetParameterValue(DP2_OUT_LOW_COUNT) - value;
						value += GetParameterValue(DP2_OUT_LOW_COUNT);
					}
				}
				else
				{
					// write zero
					value = 0;
				}
				// Write to DAC
				convertedValue = DACWriteOutValue( DP2_ANALOG_OUT, value );
				if( convertedValue != ERROR_OK )
				systemError = convertedValue;
			}
 
			if (IsTemperatureEnabled() && (GetParameterValue(TEMP_OUTPUT_TYPE) == 1 || IsPIDEnabled() == 0))
			{
				GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue <= GetParameterValue(TEMP_OUT_LOW_TEMP) )
						value = GetParameterValue(TEMP_OUT_LOW_COUNT);
					else if( senVal.convertedValue >= GetParameterValue(TEMP_OUT_HIGH_TEMP) )
						value = GetParameterValue(TEMP_OUT_HIGH_COUNT);
					else
					{
						value = GetParameterValue(TEMP_OUT_HIGH_TEMP) - GetParameterValue(TEMP_OUT_LOW_TEMP);
						convertedValue = GetParameterValue(TEMP_OUT_HIGH_COUNT) - GetParameterValue(TEMP_OUT_LOW_COUNT);
						value = (((unsigned long)(senVal.convertedValue - GetParameterValue(TEMP_OUT_LOW_TEMP))) * convertedValue) / value;
						value += GetParameterValue(TEMP_OUT_LOW_COUNT);
					}
					
					if( ! GetParameterValue(TEMP_OUT_POLARITY))
					{
						value -= GetParameterValue(TEMP_OUT_LOW_COUNT);
						value = GetParameterValue(TEMP_OUT_HIGH_COUNT) - GetParameterValue(TEMP_OUT_LOW_COUNT) - value;
						value += GetParameterValue(TEMP_OUT_LOW_COUNT);
					}
				}
				else
				{
					value = 0;
				}
				// Write to DAC
				convertedValue = DACWriteOutValue( TEMP_ANALOG_OUT, value );
				if( convertedValue != ERROR_OK )
				{
					systemError = convertedValue;
				}
			}

			if (IsHumidityEnabled() && (GetParameterValue(RH_OUTPUT_TYPE) == 1 || IsPIDEnabled() == 0))
			{
				GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
				if( senVal.errorCode == ERROR_OK )
				{
					if( senVal.convertedValue <= GetParameterValue(RH_OUT_LOW_RH) )
						value = GetParameterValue(RH_OUT_LOW_COUNT);
					else if( senVal.convertedValue >= GetParameterValue(RH_OUT_HIGH_RH) )
						value = GetParameterValue(RH_OUT_HIGH_COUNT);
					else
					{
						value = GetParameterValue(RH_OUT_HIGH_RH) - GetParameterValue(RH_OUT_LOW_RH);
						convertedValue = GetParameterValue(RH_OUT_HIGH_COUNT) - GetParameterValue(RH_OUT_LOW_COUNT);
						value = (((unsigned long)(senVal.convertedValue - GetParameterValue(RH_OUT_LOW_RH))) * convertedValue) / value;
						value += GetParameterValue(RH_OUT_LOW_COUNT);
					}
				
					if( ! GetParameterValue(RH_OUT_POLARITY))
					{
						value -= GetParameterValue(RH_OUT_LOW_COUNT);
						value = GetParameterValue(RH_OUT_HIGH_COUNT) - GetParameterValue(RH_OUT_LOW_COUNT) - value;
						value += GetParameterValue(RH_OUT_LOW_COUNT);
					}
				}
				else
				{
					// write zero
					value = 0;
				}
				// Write to DAC
				convertedValue = DACWriteOutValue( RH_ANALOG_OUT, value );
				if( convertedValue != ERROR_OK )
				systemError = convertedValue;
			}
		}
		
		DoIOPortMapping();
		
		#ifdef OS_AVRX
		AvrXDelay(&devIOLoopSleepTimer, 100);
		//AvrXDelay(&devIOLoopSleepTimer, 500);
		#else
		OSSleep( 100 );
		//OSSleep( 500 );
		#endif
	}
}
//#pragma GCC pop_options

static void DoIOPortMapping()
{
   if(alarmOut.alarm.fire==1) return;
   if(alarmOut.alarm.door==1) return;
	
   uint8_t inputs = INPUT_PORT_IN & 0x1F;
   if(DOOR_SENSE) inputs |= PIN5_bm;

   int mapping, pol = 0;
   
   //mapping = GetParameterValue(OUTPUT1_MAPPING);
   //if( mapping )
   //{
      //mapping = mapping - 1;  // Normalize to 0 numbering from 1
      //pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      //mapping /= 2;  // Discard last bit
	  //mapping = 0x1 << mapping;   // generate mask to read input
      //pol = pol * mapping;  // Set polarity mask
	  //if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     //OUTPUT1_HIGH;
      //else
         //OUTPUT1_LOW;
   //}
   mapping = GetParameterValue(OUTPUT2_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT2_HIGH; 
      else
         OUTPUT2_LOW;
   }
   mapping = GetParameterValue(OUTPUT3_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT3_HIGH; 
      else
         OUTPUT3_LOW;
   }
   mapping = GetParameterValue(OUTPUT4_MAPPING);
   if( mapping )
   {
      mapping = mapping - 1;  // Normalize to 0 numbering from 1
      pol = mapping % 2;  // Last bet as polarity, pol = 0 for normal, 1 for reverse polarity
      mapping /= 2;  // Discard last bit
	  mapping = 0x1 << mapping;   // generate mask to read input
      pol = pol * mapping;  // Set polarity mask
	  if(( inputs ^ pol ) & mapping )  // Normalize input based on output polarity
	     OUTPUT4_HIGH;  
      else
         OUTPUT4_LOW;
   }
}


void SetOutput( uint8_t outValue, uint8_t * retVal )
{
   int mapping;

   //mapping = GetParameterValue(OUTPUT1_MAPPING);
   //if( !mapping )
   //{
	  //if( outValue & 0x1 )
	     //OUTPUT1_HIGH;
	  //else
	     //OUTPUT1_LOW;
   //}
   mapping = GetParameterValue(OUTPUT2_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x2 )
	     OUTPUT2_HIGH;
	  else
	     OUTPUT2_LOW;
   }
   mapping = GetParameterValue(OUTPUT3_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x4 )
	     OUTPUT3_HIGH;
	  else
	     OUTPUT3_LOW;
   }
   mapping = GetParameterValue(OUTPUT4_MAPPING);
   if( !mapping )
   {
	  if( outValue & 0x8 )
	     OUTPUT4_HIGH;
	  else
	     OUTPUT4_LOW;
   }
   *retVal = (OUTPUT_PORT & 0x0F); 
}

void GetOutput( uint8_t * retVal )
{
   *retVal = (OUTPUT_PORT & 0x0F); 
}

void GetInput( uint8_t * retVal )
{
	uint8_t inputs = INPUT_PORT_IN & 0x1F;
	if(DOOR_SENSE) inputs |= PIN5_bm;
	
   *retVal = inputs; 
}

/*
   TempRH IO Task
 */

//static OSSemaMutex TempRHMutex;

uint16_t tempDelay = 1000;

#ifdef OS_AVRX
TimerControlBlock tempLoopSleepTimer;
AVRX_GCC_TASKDEF(TEMPRHIO_FUNC_NAME, TEMPRHIO_STACK_SIZE, TEMPRHIO_PRIORITY)
#else
void TEMPRHIO_FUNC_NAME( void * taskPara );
void TEMPRHIO_FUNC_NAME( void * taskPara )
#endif
{
	uint8_t retVal = -1, checkSum;
	unsigned int value, value1;
	int convertedValue;
	
	//if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_SHT35 )
	//{
		//ResetSensorSHT35(0);
		//
		//#ifdef OS_AVRX
		//AvrXDelay(&tempLoopSleepTimer, 2000);
		//#else
		//OSSleep(2000);
		//#endif
		//
		//StartSensorSHT35(0);
	//}
	//
	//if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_SHT35 )
	//{
		//ResetSensorSHT35(1);
		//
		//#ifdef OS_AVRX
		//AvrXDelay(&tempLoopSleepTimer, 2000);
		//#else
		//OSSleep(2000);
		//#endif
		//
		//StartSensorSHT35(1);
	//}
   
	#ifdef OS_AVRX
	AvrXDelay(&tempLoopSleepTimer, 3000);
	#else
	OSSleep(3000);
	#endif

	//while(1)
	//{
		//PORTA_OUTTGL = _BV(2);
		//wdt_reset();			//Serve Watchdog Timer
		//OSSleep(1000);
	//}
	
	Kalman_Init(&Kalmanfilter[TEMPERATURE_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalmanfilter[HUMIDITY_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalmanfilter[TEMPERATURE2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
	Kalman_Init(&Kalmanfilter[HUMIDITY2_VAL_INDEX], 0.01, 0.1, 0.0);  // Initialize with default values
 
	while (1)
	{
		//PORTA_OUTTGL = _BV(2);
		wdt_reset();			//Serve Watchdog Timer
	    	
		if(CurrentDoorStatus()==OPEN)
		//if(INPUT1_SENSE==CLOSE)
		{
			alarmOut.alarm.door=1;
		}
		else
		{
			alarmOut.alarm.door=0;
		}
		
		if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_SHT25 )
		{
			if (IsTemperatureEnabled())
			{   
				retVal = GetSensorSHT25Temperature(0, &value, &checkSum );
			
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = value + GetParameterValue(TEMP_ZERO_ADJ);
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(TEMPERATURE_VAL_INDEX, retVal, value, convertedValue );
			}

			if (IsHumidityEnabled())
			{  
				retVal = GetSensorSHT25Humidity(0, &value, &checkSum );
			
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = value + GetParameterValue(RH_ZERO_ADJ);
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(HUMIDITY_VAL_INDEX, retVal, value, convertedValue );
			}
		}
		//else if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_SHT35 )
		//{ 
			//if(IsTemperatureEnabled() || IsHumidityEnabled())
			//{
				//retVal = GetSensorSHT35Readings(0, &value, &value1 );
			//}
//
			//if(IsTemperatureEnabled())
			//{   /* Timer * 4 */
				//if ( retVal == ERROR_OK )
				//{
					//convertedValue = 17500ul * value / 0xFFFF - 4500 + GetParameterValue(TEMP_ZERO_ADJ);    /* degree C */
				//}
				//else
				//{
					//value = convertedValue = 0;
				//}
				//AveragePara(TEMPERATURE_VAL_INDEX, retVal, value, convertedValue );
			//}
			//else
			//{
				//if (IsHumidityEnabled())
				//{  /* Timer * 3 */
					//// Convert the value to unit
					//if ( retVal == ERROR_OK )
					//{
						//convertedValue = 10000ul * value1 / 0xFFFF + GetParameterValue(RH_ZERO_ADJ);
					//}
				//
				//}
				//value1 = convertedValue = 0;
//
				//AveragePara(HUMIDITY_VAL_INDEX, retVal, value1, convertedValue );
			//}
		//}
		else if( GetParameterValue(TEMP_RH_SENS_TYPE) == TEMP_RH_SENS_IDT_HS3100 )
		{
			if(IsTemperatureEnabled() || IsHumidityEnabled())
			{
				StartSensor_IDT_HS3100(0);
				
				#ifdef OS_AVRX
				AvrXDelay(&tempLoopSleepTimer, 500);
				#else
				OSSleep(500);
				#endif
			
				retVal = GetSensorReadings_IDT_HS3100(0, &value, &value1 );
			}  

			if(IsTemperatureEnabled())
			{   /* Timer * 4 */
				if ( retVal == ERROR_OK )
				{
					convertedValue = (value * 16500l / 0x3FFF) - 4000 + GetParameterValue(TEMP_ZERO_ADJ);    /* degree C */
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(TEMPERATURE_VAL_INDEX, retVal, value, convertedValue );
			}
		
			if (IsHumidityEnabled())
			{  /* Timer * 3 */
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = (value1 * 10000l / 0x3FFF) + GetParameterValue(RH_ZERO_ADJ); 
				}
				else
				{
					value1 = convertedValue = 0;
				}
			
				AveragePara(HUMIDITY_VAL_INDEX, retVal, value1, convertedValue );
			}
		}
		//====================================================================================================
		if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_SHT25 )
		{
			if(IsTemperature2Enabled())
			{
				//----------------------------------------------------------------------
				retVal = GetSensorSHT25Temperature(1, &value, &checkSum );
				
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = value + GetParameterValue(TEMP2_ZERO_ADJ);
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(TEMPERATURE2_VAL_INDEX, retVal, value, convertedValue );
	
				retVal = GetSensorSHT25Humidity(1, &value, &checkSum );
				
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = value + GetParameterValue(RH2_ZERO_ADJ);
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(HUMIDITY2_VAL_INDEX, retVal, value, convertedValue );
			}
		}
		//else if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_SHT35 )
		//{
			//if(IsTemperature2Enabled())
			//{
				//retVal = GetSensorSHT35Readings(1, &value, &value1 );
//
				//if ( retVal == ERROR_OK )
				//{
					//convertedValue = 17500ul * value / 0xFFFF - 4500 + GetParameterValue(TEMP2_ZERO_ADJ);    /* degree C */
				//}
				//else
				//{
					//value = convertedValue = 0;
				//}
				//AveragePara(TEMPERATURE2_VAL_INDEX, retVal, value, convertedValue );
	//
				//if (IsHumidityEnabled())
				//{  /* Timer * 3 */
					//// Convert the value to unit
					//if ( retVal == ERROR_OK )
					//{
						//convertedValue = 10000ul * value1 / 0xFFFF + GetParameterValue(RH2_ZERO_ADJ);
					//}
					//
				//}
				//value1 = convertedValue = 0;
//
				//AveragePara(HUMIDITY2_VAL_INDEX, retVal, value1, convertedValue );
			//}	
		//}
		else if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_IDT_HS3100 )
		{
			if(IsTemperature2Enabled())
			{
				StartSensor_IDT_HS3100(1);
				
				#ifdef OS_AVRX
				AvrXDelay(&tempLoopSleepTimer, 500);
				#else
				OSSleep(500);
				#endif
				
				retVal = GetSensorReadings_IDT_HS3100(1, &value, &value1 );

				if ( retVal == ERROR_OK )
				{
					convertedValue = (value * 16500l / 0x3FFF) - 4000 + GetParameterValue(TEMP2_ZERO_ADJ);    /* degree C */
				}
				else
				{
					value = convertedValue = 0;
				}
				AveragePara(TEMPERATURE2_VAL_INDEX, retVal, value, convertedValue );
	
				// Convert the value to unit
				if ( retVal == ERROR_OK )
				{
					convertedValue = (value1 * 10000l / 0x3FFF) + GetParameterValue(RH2_ZERO_ADJ);
				}
				else
				{
					value1 = convertedValue = 0;
				}
				
				AveragePara(HUMIDITY2_VAL_INDEX, retVal, value1, convertedValue );
			}
		}

		OSSleep(250);
		
		//tempDelay = GetParameterValue(TEMP_RH_SCAN_TIME);
		//tempDelay = tempDelay * 1000;
		//
		//if(( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_SHT25 ))// || ( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_SHT35 ))
		//{
			//#ifdef OS_AVRX
			//AvrXDelay(&tempLoopSleepTimer, tempDelay);
			//#else
			//OSSleep(tempDelay);
			//#endif
		//}
		//else if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_IDT_HS3100 )
		//{
			//#ifdef OS_AVRX
			//AvrXDelay(&tempLoopSleepTimer, tempDelay);
			//#else
			//OSSleep(tempDelay);
			//#endif
		//}
		////else if( GetParameterValue(TEMP_RH2_SENS_TYPE) == TEMP_RH_SENS_IDT_HS3100 )
		////{
			////#ifdef OS_AVRX
			////AvrXDelay(&tempLoopSleepTimer, tempDelay-500);
			////#else
			////OSSleep(tempDelay-500);
			////#endif
		////}
		//else
		//{
			//#ifdef OS_AVRX
			//AvrXDelay(&tempLoopSleepTimer, 1000);
			//#else
			//OSSleep(1000);
			//#endif
		//}
	}
}

//#ifdef OS_AVRX
//TimerControlBlock tempLoopSleepTimer;
//AVRX_GCC_TASKDEF(PARTSENS_FUNC_NAME, PARTSENS_STACK_SIZE, PARTSENS_PRIORITY)
//#else
//void PARTSENS_FUNC_NAME( void * taskPara );
//void PARTSENS_FUNC_NAME( void * taskPara )
//#endif
//{
	////if(GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_SEN55)
	//{
		//sen5x_reset_device(SENSOR_1);
		//sen5x_reset_device(SENSOR_2);
		//
		//#ifdef OS_AVRX
		//AvrXDelay(&tempLoopSleepTimer, 1000);
		//#else
		//OSSleep(1000);
		//#endif
		//
		//sen5x_start_measurement(SENSOR_1);
		//sen5x_start_measurement(SENSOR_2);
		//
		//#ifdef OS_AVRX
		//AvrXDelay(&tempLoopSleepTimer, 1000);
		//#else
		//OSSleep(1000);
		//#endif
	//}
	////else if((GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_ISP7100) || (GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_ISP5100))
	////{
	////ips7100_reset_device(SENSOR_1);
	////ips7100_reset_device(SENSOR_2);
	////
	////#ifdef OS_AVRX
	////AvrXDelay(&tempLoopSleepTimer, 5000);
	////#else
	////OSSleep(5000);
	////#endif
	////}
	//
	////while(1)
	////{
	////PORTA_OUTTGL = _BV(3);
	////wdt_reset();			//Serve Watchdog Timer
	////OSSleep(1000);
	////}
	//
	////sen5x_start_fan_manually(SENSOR_1);
//
	//while (1)
	//{
		////PORTA_OUTTGL = _BV(3);
		//wdt_reset();			//Serve Watchdog Timer
		//
		////if(GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_SEN55)
		//{
			//if(sen5x_check_meaurement_ready(SENSOR_1) == ERROR_OK)
			//{
				//if (sen5x_read_measured_value(SENSOR_1,&sen55_1) == ERROR_OK)
				//{
					////sen55_1.temperature += GetParameterValue(TEMP_ZERO_ADJ);
					////sen55_1.humidity += GetParameterValue(RH_ZERO_ADJ);
					////sen55_1.temperature /= 2;
					//
					//sen55_1.error = ERROR_OK;
				//}
				//else
				//{
					//sen55_1.error = ERROR_PARTICAL_SENS_I2C;
				//}
			//}
			//
			////if(sen5x_check_meaurement_ready(SENSOR_2)==ERROR_OK)
			////{
				////if (sen5x_read_measured_value(SENSOR_2,&sen55_2) == ERROR_OK)
				////{
					////sen55_2.temperature += GetParameterValue(TEMP_ZERO_ADJ);
					////sen55_2.humidity += GetParameterValue(RH_ZERO_ADJ);
					////
					////sen55_2.temperature /= 2;
					////
					////sen55_2.error = ERROR_OK;
				////}
				////else
				////{
					////sen55_2.error = ERROR_PARTICAL_SENS_I2C;
				////}
			////}
		//}
		////else if((GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_ISP7100) || (GetParameterValue(PARTICAL_SENS_TYPE) == PARTCAL_SENS_ISP5100))
		////{
		//////Sensor 1 ----------------------------------------------------------------------------
		////if(ips7100_read_pc_value(SENSOR_1,&isp7100_pc_1) == ERROR_OK)
		////{
		////
		////}
		////else
		////{
		////isp7100_pc_1.error = ERROR_PARTICAL_SENS_I2C;
		////}
		////
		////if(ips7100_read_pm_value(SENSOR_1,&isp7100_pm_1) == ERROR_OK)
		////{
		////
		////}
		////else
		////{
		////isp7100_pm_1.error = ERROR_PARTICAL_SENS_I2C;
		////}
		////
		//////Sensor 2 ----------------------------------------------------------------------------
		////if(ips7100_read_pc_value(SENSOR_2,&isp7100_pc_2) == ERROR_OK)
		////{
		////
		////}
		////else
		////{
		////isp7100_pc_2.error = ERROR_PARTICAL_SENS_I2C;
		////}
		////
		////if(ips7100_read_pm_value(SENSOR_2,&isp7100_pm_2) == ERROR_OK)
		////{
		////
		////}
		////else
		////{
		////isp7100_pm_2.error = ERROR_PARTICAL_SENS_I2C;
		////}
		////}
		//
		//#ifdef OS_AVRX
		////AvrXDelay(&tempLoopSleepTimer, GetParameterValue(TEMP_RH_SCAN_TIME) * 1000);
		//#else
		////OSSleep(GetParameterValue(TEMP_RH_SCAN_TIME) * 1000);
		//OSSleep(1000);
		//#endif
	//}
//}


//static uint16_t AverageSamples[TOTAL_VAL_INDEX] = {-1};
//static long sampleSum[TOTAL_VAL_INDEX] = {0};
//static int sampleBuf[TOTAL_VAL_INDEX][30] = {0};
//static char BufPtr[TOTAL_VAL_INDEX] = {0};
//static int LastconvertedVal[TOTAL_VAL_INDEX] = {0};
static void AveragePara( uint8_t SenNo, uint8_t error, unsigned int rawVal, int convertedVal )
{
	//if( AverageSamples[SenNo] != NoofSample)
	//{
		//AverageSamples[SenNo] = NoofSample;
//
		//// Kalman filter for pressure
		//Kalman_Init(&Kalmanfilter[SenNo], 0.01, 0.1, 0.0);  // Initialize with default values
	//}
	//else
	//{
		//convertedVal = Kalman_Update(&Kalmanfilter[SenNo], convertedVal);
	//}
	
	convertedVal = Kalman_Update(&Kalmanfilter[SenNo], convertedVal);
	OSSemaTakeEver(DeviceValueMutex);
	sensorVal[SenNo].errorCode = error;
	sensorVal[SenNo].rawValue = rawVal;
	sensorVal[SenNo].convertedValue = convertedVal;
	OSSemaGive(DeviceValueMutex);
}

static int isBuzzerSuppressionReq = FALSE;			

void GetPareValue(uint8_t ParaNumber, SENSOR_VALUE * value )
{
	OSSemaTakeEver(DeviceValueMutex);
	value->errorCode = sensorVal[ParaNumber].errorCode;
	value->rawValue = sensorVal[ParaNumber].rawValue;
	value->convertedValue = sensorVal[ParaNumber].convertedValue;

	if(ParaNumber==HUMIDITY_VAL_INDEX)
	{
		if(value->convertedValue>10000) value->convertedValue = 10000;
	}
	OSSemaGive(DeviceValueMutex);
}

//void GetPartical( sen5x_measured_value_t *value )
//{
	//OSSemaTakeEver(DeviceValueMutex);
	//*value = sen55_1;
	//OSSemaGive(DeviceValueMutex);
//}

uint8_t GetSystemError()
{
   return systemError;
}

uint16_t GetAlarms()
{
   return alarmOut.alarmByte;
}

unsigned int AlarmActive_Buzzer()
{
   return buzzerOut.alarmByte;
}


void SetBuzzerSuppressionReq()
{
   isBuzzerSuppressionReq = TRUE;
}

int GetAndResetBuzzerSuppressionReq()
{
   if( isBuzzerSuppressionReq )
   {
      isBuzzerSuppressionReq = FALSE;
	  return TRUE;
   }
   return FALSE;
}

void DeviceIOInit()
{
   uint8_t i;

   for( i = 0; i < sizeof(sensorVal)/sizeof(sensorVal[0]); i++ )
   {
       sensorVal[i].errorCode = -1;
       sensorVal[i].rawValue = -1;
       sensorVal[i].convertedValue = -1;
   }

   OSSemaCreateMutex( DeviceValueMutex );
   OSSemaGive(DeviceValueMutex);

#ifdef OS_AVRX
   AvrXRunTask(TCB(DeviceIO));
   AvrXRunTask(TCB(TempRHIO));
   //if (IsParticalSensorEnabled())
   //{
	   //AvrXRunTask(TCB(PartSensIO));
   //}
#else
   /* Spawn the producer and consumer. */
   xTaskCreate( DEVICEIO_FUNC_NAME, "DIO", DEVICEIO_STACK_SIZE, ( void * ) 0, DEVICEIO_PRIORITY, NULL );
   xTaskCreate( TEMPRHIO_FUNC_NAME, "TIO", TEMPRHIO_STACK_SIZE, ( void * ) 0, TEMPRHIO_PRIORITY, NULL );
   //if (IsParticalSensorEnabled())
   //{
	   //xTaskCreate( PARTSENS_FUNC_NAME, "PARSEN", PARTSENS_STACK_SIZE, ( void * ) 0, PARTSENS_PRIORITY, NULL );
   //}
#endif
}

uint8_t CurrentDoorStatus(void)
{
	if(DOOR_SENSE)
	{
		return OPEN;
	}
	else
	{
		return CLOSE;
	}
}



