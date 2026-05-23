#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/wdt.h>
#include "os_api.h"
#include "error.h"
#include "PeriodicProcess.h"
#include "HardwareInfo.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DeviceIO.h"
#include "pid.h"

#define PERIODIC_STACK_SIZE   500
#define PERIODIC_PRIORITY     2//2
#define PERIODIC_FUNC_NAME    PeriodicProcess


static int serialTime, loggingTime;
static int buzzerSpan, buzzerOnTime, buzzerTimeCounter, buzzerDisableTime, oldBuzzVal, currBuzzVal;

/*
   {eriodic Process Task
*/
#ifdef OS_AVRX
TimerControlBlock periodicLoopSleepTimer;
AVRX_GCC_TASKDEF(PERIODIC_FUNC_NAME, PERIODIC_STACK_SIZE, PERIODIC_PRIORITY)
#else
void PERIODIC_FUNC_NAME( void * taskPara );
void PERIODIC_FUNC_NAME( void * taskPara )
#endif
{
	loggingTime = GetParameterValue( LOGGING_INTERVAL );
	serialTime = GetParameterValue( BROADCAST_INTERVAL );
	buzzerTimeCounter = 0;
	buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );

	while (1)
	{
		wdt_reset();			//Serve Watchdog Timer
	   
		DisplaySensorValues();

		if( IsLoggingEnabled() && --loggingTime == 0 )
		{
			loggingTime = GetParameterValue( LOGGING_INTERVAL );
		}

		if( IsSerialEnabled() && IsBroadcastEnabled() && --serialTime == 0 )
		{
			serialTime = GetParameterValue( BROADCAST_INTERVAL );

			SerialPrinter();
			SerialBroadcast();
		}

		currBuzzVal = AlarmActive_Buzzer();
		oldBuzzVal &= currBuzzVal;

		if( currBuzzVal ^ oldBuzzVal )
			buzzerDisableTime = 0;
		if( GetAndResetBuzzerSuppressionReq() )
			buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );
		if( buzzerDisableTime > 0)
			buzzerDisableTime--;   
		      
		oldBuzzVal = currBuzzVal;
		buzzerOnTime = GetParameterValue( BUZZER_ON_TIME );
		buzzerSpan = buzzerOnTime + GetParameterValue( BUZZER_OFF_TIME );
		
		if(( buzzerTimeCounter < buzzerOnTime ) && buzzerOnTime && buzzerDisableTime <= 0)
		{
			if( currBuzzVal )
				BuzzerOn();
			else
				BuzzerOff();
		}
		else
		{
			BuzzerOff();
		}
		
		buzzerTimeCounter = (buzzerTimeCounter + 1) % buzzerSpan;

		if( IsAlalogOutEnabled() && IsPIDEnabled())
		{
			PIDLoopControl();
		}
	  
		#ifdef OS_AVRX
		AvrXDelay(&periodicLoopSleepTimer, 1000);
		#else
		OSSleep( 1000 );
		#endif
	}
}

void SuppressBuzzer()
{
   buzzerDisableTime = GetParameterValue( BUZZER_DISABLE_TIME );
}


void PeriodicProcessInit()
{
#ifdef OS_AVRX
    AvrXRunTask(TCB(PeriodicProcess));
#else
      /* Spawn the producer and consumer. */
    xTaskCreate( PERIODIC_FUNC_NAME, "PP", PERIODIC_STACK_SIZE, ( void * ) 0, PERIODIC_PRIORITY, NULL );
#endif
}


