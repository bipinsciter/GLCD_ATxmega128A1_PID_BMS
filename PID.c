#include "UserInterface.h"
#include "HardwareInfo.h"
#include "DeviceIO.h"
#include "DAC7573.h"
#include "error.h"
#include "pid.h"

static int pidThreadReset = 0;

static int tempPIDKp = 0;
static int tempPIDTd = 0;
static int tempPIDTi = 0;
static long tempSum = 0;
static long tempSumLimit = 0;
static int tempLastError = 0;

static int rhPIDKp = 0;
static int rhPIDTd = 0;
static int rhPIDTi = 0;
static long rhSum = 0;
static long rhSumLimit = 0;
static int rhLastError = 0;

static int absPresPIDKp = 0;
static int absPresPIDTd = 0;
static int absPresPIDTi = 0;
static long absPresSum = 0;
static long absPresSumLimit = 0;
static int absPresLastError = 0;

static int diffPresPIDKp = 0;
static int diffPresPIDTd = 0;
static int diffPresPIDTi = 0;
static long diffPresSum = 0;
static long diffPresSumLimit = 0;
static int diffPresLastError = 0;

static int tempPIDOutput = 0;
static int rhPIDOutput = 0;
static int absPresPIDOutput = 0;
static int diffPresPIDOutput = 0;
static int tempPIDOutputPercent = 0;
static int rhPIDOutputPercent = 0;
static int absPresPIDOutputPercent = 0;
static int diffPresPIDOutputPercent = 0;

static int tempPIDOutputCount = 0;
static int rhPIDOutputCount = 0;
static int absPresPIDOutputCount = 0;
static int diffPresPIDOutputCount = 0;

static int humiditySetpointAdjust = 0;
static int tempSetpointAdjust = 0;


SENSOR_VALUE sensValue; 

// process variables for stack use reduction
int error, deltaError;
int tempRHCascadeError, rhTempCascadeError;
int cascadeState, cascadeFactor;

// PID output scaled to 4096 counts
void PIDLoopControl()
{
    // Kd 0.00 to 327.67
	// Ti 0.0 to 3276.7 seconds
	// Td 0.00 to 327.67 seconds
	if( !pidThreadReset )
	{
       absPresPIDKp = GetParameterValue(DP1_PID_KP);
       absPresPIDTi = GetParameterValue(DP1_PID_TI);
       diffPresPIDKp = GetParameterValue(DP2_PID_KP);
       diffPresPIDTi = GetParameterValue(DP2_PID_TI);
       tempPIDKp = GetParameterValue(TEMP_PID_KP);
	   tempPIDTi = GetParameterValue(TEMP_PID_TI);
       rhPIDKp = GetParameterValue(RH_PID_KP);
       rhPIDTi = GetParameterValue(RH_PID_TI);
       
	   error = GetParameterValue(DP1_PID_STARTUP_PERCENT); // % with One decimal
	   absPresSum = 4095.0 * error * absPresPIDTi / absPresPIDKp / 100;
	   absPresSumLimit = 4095.0 * 10 * absPresPIDTi / absPresPIDKp;
	   error = GetParameterValue(DP2_PID_STARTUP_PERCENT); // % with One decimal
	   diffPresSum = 4095.0 * error * diffPresPIDTi / diffPresPIDKp / 100;
	   diffPresSumLimit = 4095.0 * 10 * diffPresPIDTi / diffPresPIDKp;
	   error = GetParameterValue(TEMP_PID_STARTUP_PERCENT); // % with One decimal
	   tempSum = 4095.0 * error * tempPIDTi / tempPIDKp / 100;
	   tempSumLimit = 4095.0 * 10 * tempPIDTi / tempPIDKp;
	   error = GetParameterValue(RH_PID_STARTUP_PERCENT); // % with One decimal
	   rhSum = 4095.0 * error * rhPIDTi / rhPIDKp / 100;
	   rhSumLimit = 4095.0 * 10 * rhPIDTi / rhPIDKp;

       pidThreadReset = 1;
	}

    if( GetParameterValue(TEMP_OUTPUT_TYPE) == 2 && IsTemperatureEnabled() )
	{ 
	    // load and check for change of Kp, Ki and Kd parameters
		// Calculate I sum limits
	    // 4095 = PIDKp * Sum / (10 * PIdTi)
	    // Sum = 4096 * 10 * PidTi / PIDKp
        if( tempPIDKp != GetParameterValue(TEMP_PID_KP))
		{
		   tempSum = (float)tempSum * tempPIDKp / GetParameterValue(TEMP_PID_KP);
		   tempPIDKp = GetParameterValue(TEMP_PID_KP);
		   tempSumLimit = 4095l * 10 * tempPIDTi / tempPIDKp;  
		}
        if( tempPIDTi != GetParameterValue(TEMP_PID_TI))
		{
		   tempSum = (float)tempSum * GetParameterValue(TEMP_PID_TI) / tempPIDTi;
		   tempPIDTi = GetParameterValue(TEMP_PID_TI);
         tempSumLimit = 4095l * 10 * tempPIDTi / tempPIDKp;  
		}
		tempPIDTd = GetParameterValue(TEMP_PID_TD);

	    // Get/calculate set points
	    // calculate error
	    // error = SP - AV
		GetPareValue(TEMPERATURE_VAL_INDEX, &sensValue);//GetTemperature( &sensValue );
        if( sensValue.errorCode == ERROR_OK )
        {
    		error = GetParameterValue(TEMP_PID_SET_VALUE) - sensValue.convertedValue;
			deltaError = error - tempLastError;
    		tempLastError = error;

            if( rhTempCascadeError )
            {
                if( rhTempCascadeError > 0 )
                {
                   cascadeFactor = GetParameterValue(RH_LOW_PID_TEMP_CASCADE_FACTOR);
                   cascadeState = GetParameterValue(RH_LOW_PID_CASCADE_TEMP_STATE);
                }
                else
                {
                   cascadeFactor = GetParameterValue(RH_HIGH_PID_TEMP_CASCADE_FACTOR);
                   cascadeState = GetParameterValue(RH_HIGH_PID_CASCADE_TEMP_STATE);
                }
                if( cascadeFactor && (cascadeState == 0 || ( cascadeState < 0 && error > 0 ) || ( cascadeState > 0 && error < 0 )))
                {
                    error = (long)rhTempCascadeError * cascadeFactor / 100;
                    deltaError = 0;
                }
                else
                   rhTempCascadeError = 0;
            }

            error = error + tempSetpointAdjust;

       	    tempSum = tempSum + error;

            tempRHCascadeError = 0;
			humiditySetpointAdjust = 0;
            if( tempSum < 0 )
			{
                if( !rhTempCascadeError )
				{
                   tempRHCascadeError = tempLastError;
				   if( tempLastError < 0 )
				   {  // adjust rh set point only if temp is higher than regular setpoint and output is out of range. 
				       humiditySetpointAdjust = GetParameterValue(TEMP_HIGH_PID_RH_ADJUST);
				   }
                }
     		    tempSum = 0;
      	        tempPIDOutput = 0;
            }
			else if( tempSum > tempSumLimit )
			{
                if( !rhTempCascadeError )
				{
                   tempRHCascadeError = tempLastError;
				   if( tempLastError > 0 )
				   {  // adjust rh set point only if temp is lower than regular setpoint and output is out of range. 
				       humiditySetpointAdjust = GetParameterValue(TEMP_LOW_PID_RH_ADJUST);
				   }
                }
    		    tempSum = tempSumLimit;
	    	    tempPIDOutput = 4095;
            }
            else
			{  // Divide by 100 is because of temprature unit
	    	   tempPIDOutput = (long)tempPIDKp * (error + (long)tempPIDTd*deltaError/100 + (tempSum * 10 /tempPIDTi))/100;
	    	   if( tempPIDOutput < 0 )
	    	      tempPIDOutput = 0;
	    	   if( tempPIDOutput > 4095 )
	    		  tempPIDOutput = 4095;
			}
        }
        else
            tempPIDOutput = 0;

        tempPIDOutputPercent = tempPIDOutput * 1000l / 4095;

	    if( ! GetParameterValue(TEMP_OUT_POLARITY))
			tempPIDOutput = 4095 - tempPIDOutput;

        // Convert to output counts
        tempPIDOutputCount = GetParameterValue(TEMP_OUT_LOW_COUNT) + (long)tempPIDOutput * (GetParameterValue(TEMP_OUT_HIGH_COUNT) - GetParameterValue(TEMP_OUT_LOW_COUNT)) / 4096l;
        // Write to DAC
		error = DACWriteOutValue( TEMP_ANALOG_OUT, tempPIDOutputCount );
        if( error != ERROR_OK )
           systemError = error;
    }
    else
    {
        tempRHCascadeError = 0;  
    }
    if( GetParameterValue(RH_OUTPUT_TYPE) == 2 && IsHumidityEnabled() )
	{ 
	    // load and check for change of Kp, Ki and Kd parameters
		// Calculate I sum limits
	    // 4095 = PIDKp * Sum / (10 * PIdTi)
	    // Sum = 4096 * 10 * PidTi / PIDKp
        if( rhPIDKp != GetParameterValue(RH_PID_KP))
		{
			rhSum = (float)rhSum * rhPIDKp / GetParameterValue(RH_PID_KP);
		    rhPIDKp = GetParameterValue(RH_PID_KP);
		    rhSumLimit = 4095l * 10 * rhPIDTi / rhPIDKp;  
		}
        if( rhPIDTi != GetParameterValue(RH_PID_TI))
		{
 		   rhSum = (float)rhSum * GetParameterValue(RH_PID_TI) / rhPIDTi;
	       rhPIDTi = GetParameterValue(RH_PID_TI);
		    rhSumLimit = 4095l * 10 * rhPIDTi / rhPIDKp;  
		}
		rhPIDTd = GetParameterValue(RH_PID_TD);

	    // Get/calculate set points
	    // calculate error
	    // error = SP - AV
		GetPareValue(HUMIDITY_VAL_INDEX, &sensValue);//GetHumidity(&sensValue);
        if( sensValue.errorCode == ERROR_OK )
        {
    		error = GetParameterValue(RH_PID_SET_VALUE) - sensValue.convertedValue;
			deltaError = error - rhLastError;
    		rhLastError = error;

            if( tempRHCascadeError )
			{
                if( tempRHCascadeError > 0 )
                {
                    cascadeFactor = GetParameterValue(TEMP_LOW_PID_RH_CASCADE_FACTOR);
                    cascadeState = GetParameterValue(TEMP_LOW_PID_CASCADE_RH_STATE);
                }
                else 
                {
                    cascadeFactor = GetParameterValue(TEMP_HIGH_PID_RH_CASCADE_FACTOR);
                    cascadeState = GetParameterValue(TEMP_HIGH_PID_CASCADE_RH_STATE);
                }
                if( cascadeFactor && (cascadeState == 0 || ( cascadeState < 0 && error > 0 ) || ( cascadeState > 0 && error < 0 )))
                {
                    error = (long)tempRHCascadeError * cascadeFactor / 100;
                    deltaError = 0;
                }
                else
                    tempRHCascadeError = 0;
            }

            error = error + humiditySetpointAdjust;
    	    rhSum = rhSum + error;


			rhTempCascadeError = 0;
			tempSetpointAdjust = 0;
    	    if( rhSum < 0 )
			{
                if( !tempRHCascadeError )
				{
                    rhTempCascadeError = rhLastError;
				   if( rhLastError < 0 )
				   {  // adjust temp set point only if rh is higher than regular setpoint and output is out of range. 
				       tempSetpointAdjust = GetParameterValue(RH_HIGH_PID_TEMP_ADJUST);
				   }
                }
    		    rhSum = 0;
     		    rhPIDOutput = 0;
			}
    	    else if( rhSum > rhSumLimit )
			{
                if( !tempRHCascadeError )
				{
                   rhTempCascadeError = rhLastError;
				   if( rhLastError > 0 )
				   {  // adjust temp set point only if rh is lower than regular setpoint and output is out of range. 
				       tempSetpointAdjust = GetParameterValue(RH_LOW_PID_TEMP_ADJUST);
				   }
                }
    		    rhSum = rhSumLimit;
			    rhPIDOutput = 4095;
			}
            else
			{
                rhPIDOutput = (long)rhPIDKp * (error + (long)rhPIDTd*deltaError/100 + (rhSum * 10 /rhPIDTi))/100;
                if( rhPIDOutput < 0 )
                    rhPIDOutput = 0;
                if( rhPIDOutput > 4095 )
                    rhPIDOutput = 4095;
            }
        }
        else
            rhPIDOutput = 0;

        rhPIDOutputPercent = rhPIDOutput * 1000l / 4095;

	    if( ! GetParameterValue(RH_OUT_POLARITY))
			rhPIDOutput = 4095 - rhPIDOutput;

        // Convert to output counts
        rhPIDOutputCount = GetParameterValue(RH_OUT_LOW_COUNT) + (long)rhPIDOutput * (GetParameterValue(RH_OUT_HIGH_COUNT) - GetParameterValue(RH_OUT_LOW_COUNT)) / 4096l;
        // Write to DAC
 		  error = DACWriteOutValue( RH_ANALOG_OUT, rhPIDOutputCount );
        if( error != ERROR_OK )
           systemError = error;
     }
     else
     {
         rhTempCascadeError = 0;  
     }

     if( GetParameterValue(DP2_OUTPUT_TYPE) == 2 && IsDP2Enabled() )
	 { 
	    // load and check for change of Kp, Ki and Kd parameters
	    // Calculate I sum limits
	    // 4095 = PIDKp * Sum / (10 * PIdTi)
	    // Sum = 4096 * 10 * PidTi / PIDKp
      if( diffPresPIDKp != GetParameterValue(DP2_PID_KP))
		{
			diffPresSum = (float)diffPresSum * diffPresPIDKp / GetParameterValue(DP2_PID_KP);
		    diffPresPIDKp = GetParameterValue(DP2_PID_KP);
		    diffPresSumLimit = 4095l * 10 * diffPresPIDTi / diffPresPIDKp;  
		}
      if( diffPresPIDTi != GetParameterValue(DP2_PID_TI))
		{
			diffPresSum = (float)diffPresSum * GetParameterValue(DP2_PID_TI) / diffPresPIDTi ;
		    diffPresPIDTi = GetParameterValue(DP2_PID_TI);
		    diffPresSumLimit = 4095l * 10 * diffPresPIDTi / diffPresPIDKp;  
		}
		diffPresPIDTd = GetParameterValue(DP2_PID_TD);

	    // Get/calculate set points
	    // calculate error
	    // error = SP - AV
		GetPareValue(DP2_VAL_INDEX, &sensValue);//GetDP2(&sensValue);
        if( sensValue.errorCode == ERROR_OK )
        {
    		error = GetParameterValue(DP2_PID_SET_VALUE) - sensValue.convertedValue;
			deltaError = error - diffPresLastError;
    		diffPresLastError = error;

    	    diffPresSum = diffPresSum + error;
    	    if( diffPresSum < 0 )
    		   diffPresSum = 0;
    	    if( diffPresSum > diffPresSumLimit )
    		   diffPresSum = diffPresSumLimit;
    	    diffPresPIDOutput = (long)diffPresPIDKp * (error + (long)diffPresPIDTd*deltaError/100 + (diffPresSum*10/diffPresPIDTi))/100;
    		diffPresLastError = error;
    	    if( diffPresPIDOutput < 0 )
    		   diffPresPIDOutput = 0;
    	    if( diffPresPIDOutput > 4095 )
    		   diffPresPIDOutput = 4095;

        }
        else
            diffPresPIDOutput = 0;

        diffPresPIDOutputPercent = diffPresPIDOutput * 1000l / 4095;

		if( ! GetParameterValue(DP2_OUT_POLARITY))
            diffPresPIDOutput = 4095 - diffPresPIDOutput;

        // Convert to output counts
        diffPresPIDOutputCount = GetParameterValue(DP2_OUT_LOW_COUNT) + (long)diffPresPIDOutput * (GetParameterValue(DP2_OUT_HIGH_COUNT) - GetParameterValue(DP2_OUT_LOW_COUNT)) / 4096l;
        // Write to DAC
        error = DACWriteOutValue( DP2_ANALOG_OUT, diffPresPIDOutputCount );
        if( error != ERROR_OK )
           systemError = error;
    }
    if( GetParameterValue(DP1_OUTPUT_TYPE) == 2 && IsDP1Enabled() )
	{ 
	    // load and check for change of Kp, Ki and Kd parameters
		// Calculate I sum limits
	    // 4095 = PIDKp * Sum / (10 * PIdTi)
	    // Sum = 4096 * 10 * PidTi / PIDKp
        if( absPresPIDKp != GetParameterValue(DP1_PID_KP))
		{
		   absPresSum = absPresSum * absPresPIDKp / GetParameterValue(DP1_PID_KP);
		   absPresPIDKp = GetParameterValue(DP1_PID_KP);
	       absPresSumLimit = 4095l * 10 * absPresPIDTi / absPresPIDKp;  
		}
        if( absPresPIDTi != GetParameterValue(DP1_PID_TI))
		{
		   absPresSum = absPresSum * GetParameterValue(DP1_PID_TI) / absPresPIDTi;
		   absPresPIDTi = GetParameterValue(DP1_PID_TI);
	       absPresSumLimit = 4095l * 10 * absPresPIDTi / absPresPIDKp;  
		}
		absPresPIDTd = GetParameterValue(DP1_PID_TD);

	    // Get/calculate set points
	    // calculate error
	    // error = SP - AV
		GetPareValue(DP1_VAL_INDEX, &sensValue);//GetDP1(&sensValue);
       if( sensValue.errorCode == ERROR_OK )
       {
    		error = GetParameterValue(DP1_PID_SET_VALUE) - sensValue.convertedValue;
			deltaError = error - absPresLastError;
    		absPresLastError = error;

    	    absPresSum = absPresSum + error;
    	    if( absPresSum < 0 )
    		   absPresSum = 0;
    	    if( absPresSum > absPresSumLimit )
    		   absPresSum = absPresSumLimit;
    	    absPresPIDOutput = (long)absPresPIDKp * (error + (long)absPresPIDTd*deltaError/100 + (absPresSum*10/absPresPIDTi))/100;
    		absPresLastError = error;
    	    if( absPresPIDOutput < 0 )
    		   absPresPIDOutput = 0;
    	    if( absPresPIDOutput > 4095 )
    		   absPresPIDOutput = 4095;
       }
       else
          absPresPIDOutput = 0;

       absPresPIDOutputPercent = absPresPIDOutput * 1000l / 4095;

	   if( ! GetParameterValue(DP1_OUT_POLARITY))
          absPresPIDOutput = 4095 - absPresPIDOutput;

       // Convert to output counts
       absPresPIDOutputCount = GetParameterValue(DP1_OUT_LOW_COUNT) + (long)absPresPIDOutput * (GetParameterValue(DP1_OUT_HIGH_COUNT) - GetParameterValue(DP1_OUT_LOW_COUNT)) / 4096l;

       // Write to DAC
	   error = DACWriteOutValue( DP1_ANALOG_OUT, absPresPIDOutputCount );
       if( error != ERROR_OK )
           systemError = error;
    }
}


int GetAbsPresPIDPercent()
{
    return absPresPIDOutputPercent;
}

int GetDiffPresPIDPercent()
{
    return diffPresPIDOutputPercent;
}
int GetTempPIDPercent()
{
    return tempPIDOutputPercent;
}
int GetRHPIDPercent()
{
    return rhPIDOutputPercent;
}


