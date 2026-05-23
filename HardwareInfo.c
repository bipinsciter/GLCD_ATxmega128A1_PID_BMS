#include <avr/io.h>
#include "os_api.h"
#include "HardwareInfo.h"
#include "UserInterface.h"
#include "eepromAdr.h"

#define TRUE  1
#define FALSE 0


#define DIFF_PRESSURE1_ENB	  (1<<0)
#define DIFF_PRESSURE2_ENB	  (1<<1)
#define TEMPARATURE_ENB      (1<<2)
#define HUMIDITY_ENB         (1<<3)
#define LCD_DISPLAY_ENB      (1<<4)
#define ALARM_ENB            (1<<5)
#define ANALOGOUT_ENB        (1<<6)
#define LOG_ENB              (1<<7)
#define SERIAL_HANDSHACK_ENB (1<<8)
#define WIRELESS_BRDCST_ENB  (1<<9)
#define SERIAL_PRINTER_ENB   (1<<10)
#define PID_ENB				 (1<<11)
#define PRES_SENS_TEMP_ENB	 (1<<12)
#define DIFF_PRESSURE3_ENB	 (1<<13)
#define POST_TEMPARATURE_ENB (1<<14)
#define PARTICAL_SENS_ENB	 (1<<15)

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

unsigned char IsDP1Enabled()
{
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & DIFF_PRESSURE1_ENB )? 1: 0;
}

unsigned char IsDP2Enabled()
{
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & DIFF_PRESSURE2_ENB )? 1: 0;
}

unsigned char IsDP3Enabled()
{
	return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & DIFF_PRESSURE3_ENB )? 1: 0;
}

unsigned char IsHumidityEnabled()
{
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & HUMIDITY_ENB )? 1: 0;
}

unsigned char IsTemperatureEnabled()
{
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & TEMPARATURE_ENB )? 1: 0;
}

unsigned char IsTemperature2Enabled()
{
	return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & POST_TEMPARATURE_ENB )? 1: 0;
}

//unsigned char IsPressureSenTempEnabled()
//{
   //return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & PRES_SENS_TEMP_ENB )? 1: 0;
//}

unsigned char IsParticalSensorEnabled()
{
	return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & PARTICAL_SENS_ENB )? 1: 0;
}

unsigned char IsLCDDisplay()
{
//   return TRUE;
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & LCD_DISPLAY_ENB )? 1: 0;
}

unsigned char IsAlarmEnabled()
{
//return TRUE;
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & ALARM_ENB )? 1: 0;
}


unsigned char IsAlalogOutEnabled()
{
//return TRUE;
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & ANALOGOUT_ENB )? 1: 0;
}

unsigned char IsLoggingEnabled()
{
//return TRUE;
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & LOG_ENB )? 1: 0;
}

unsigned char IsSerialHandshackEnabled()
{
//return TRUE;
   if(!(OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM) ^ 0xffff))
      return TRUE;   // if not configured (all bits one) keep serial enabled for computer programing
   if(!OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM))
      return TRUE;   // if not configured (all bits zero) keep serial enabled for computer programing

   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & SERIAL_HANDSHACK_ENB )? 1: 0;
}

unsigned char IsWirelessBroadcastEnabled()
{
   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & WIRELESS_BRDCST_ENB )? 1: 0;
}

unsigned char IsSerialPrinterEnabled()
{
   //return FALSE;

   return ((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & SERIAL_PRINTER_ENB )? 1: 0;
}

unsigned char IsSerialEnabled()
{
	//return FALSE;
	
   char mode;

   mode = IsSerialPrinterEnabled()? 1 : 0;
   mode += (IsSerialHandshackEnabled() || IsWirelessBroadcastEnabled())? 1 : 0;

   if( mode )
      return TRUE;
   else
      return FALSE;
}

unsigned char IsBroadcastEnabled()
{
	//return FALSE;
	
   return (IsSerialPrinterEnabled() || IsWirelessBroadcastEnabled())? 1 : 0;
}


unsigned char IsPIDEnabled()
{
   return (((~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM)) & PID_ENB )? 1: 0) && IsAlalogOutEnabled();
}

unsigned char IsDP1CFMEnabled()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP1_UNIT)) == PRESSURE_CFM) && IsDP1Enabled());
}

unsigned char IsDP2CFMEnabled()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP2_UNIT)) == PRESSURE_CFM) && IsDP2Enabled());
}

unsigned char IsDP3CFMEnabled()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP3_UNIT)) == PRESSURE_CFM) && IsDP3Enabled());
}

unsigned char IsDP1CFMAreaSquare()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP1_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP1CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP1_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP1_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP1Enabled());
}

unsigned char IsDP2CFMAreaSquare()
{	
	return (((OSReadEEPromWord((unsigned int *)EA_DP2_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP2CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP2_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP2_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP2Enabled());
}

unsigned char IsDP3CFMAreaSquare()
{	
	if( !IsDP3CFMEnabled())
	return FALSE;
	return (GetParameterValue(DP3_AREA_TYPE) == PRES_AREA_SQUARE);
	
	//return (((OSReadEEPromWord((unsigned int *)EA_DP3_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP3CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP3_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP3_AREA_TYPE)) == PRES_AREA_SQUARE) && IsDP3Enabled());
}

unsigned char IsDP1CFMAreaRound()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP1_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP1CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP1_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP1_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP1Enabled());
}

unsigned char IsDP2CFMAreaRound()
{
	return (((OSReadEEPromWord((unsigned int *)EA_DP2_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP2CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP2_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP2_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP2Enabled());
}

unsigned char IsDP3CFMAreaRound()
{
	if( !IsDP3CFMEnabled())
	return FALSE;
	return (GetParameterValue(DP3_AREA_TYPE) == PRES_AREA_ROUND);
	
	//return (((OSReadEEPromWord((unsigned int *)EA_DP3_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP3CFMEnabled());
	//return (((OSReadEEPromWord((unsigned int *)EA_DP3_UNIT)) == PRESSURE_CFM) && ((OSReadEEPromWord((unsigned int *)EA_DP3_AREA_TYPE)) == PRES_AREA_ROUND) && IsDP3Enabled());
}

//#pragma GCC pop_options
