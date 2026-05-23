#ifndef __HARDWAREINFO_H__
#define __HARDWAREINFO_H__

// Analog channel definations
#define DP1_ANALOG_OUT		0
#define DP2_ANALOG_OUT		1
#define TEMP_ANALOG_OUT     2
#define RH_ANALOG_OUT       3

// returns 1 (enabled) or 0 (disabled)

unsigned char IsDP1Enabled();

unsigned char IsDP2Enabled();

unsigned char IsDP3Enabled();

unsigned char IsHumidityEnabled();

unsigned char IsTemperatureEnabled();

unsigned char IsTemperature2Enabled();

unsigned char IsPressureSenTempEnabled();

unsigned char IsParticalSensorEnabled();

unsigned char IsLCDDisplay();

unsigned char IsAlarmEnabled();

unsigned char IsAlalogOutEnabled();

unsigned char IsLoggingEnabled();

unsigned char IsSerialEnabled();

unsigned char IsBroadcastEnabled();

unsigned char IsSerialHandshackEnabled();

unsigned char IsWirelessBroadcastEnabled();

unsigned char IsSerialPrinterEnabled();

unsigned char IsPIDEnabled();

unsigned char IsDP1CFMEnabled();

unsigned char IsDP2CFMEnabled();

unsigned char IsDP3CFMEnabled();

unsigned char IsDP1CFMAreaSquare();

unsigned char IsDP2CFMAreaSquare();

unsigned char IsDP3CFMAreaSquare();

unsigned char IsDP1CFMAreaRound();

unsigned char IsDP2CFMAreaRound();

unsigned char IsDP3CFMAreaRound();

#endif
