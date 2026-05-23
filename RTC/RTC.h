#ifndef __RTC__
#define __RTC__

//#define DS1307
#define PCF8563

typedef struct time
{
   unsigned char seconds;
   unsigned char minutes;
   unsigned char hours;
}TIME;

typedef struct date
{
   unsigned char date;
   #if defined(PCF8563)
   unsigned char weekday;
   #endif
   unsigned char month;
   unsigned char year;
}DATE;

#if defined(DS1307)

#define RTC_TIMEREG_START		0
#define RTC_TIMESEC_REG		    0
#define RTC_TIMEMIN_REG		    1
#define RTC_TIMEHOUR_REG		2

#define RTC_DATE_REG_START		4
#define RTC_DATE_DATE_REG		4
#define RTC_DATE_MONTH_REG		5
#define RTC_DATE_YEAR_REG		6

//#define RTC_DATA_REG_START		8

#elif defined(PCF8563)

#define RTC_CNTRL1_ADDR			0
#define RTC_CNTRL2_ADDR			1

#define RTC_TIMEREG_START		2
#define RTC_TIMESEC_REG		    2
#define RTC_TIMEMIN_REG		    3
#define RTC_TIMEHOUR_REG		4

#define RTC_DATE_REG_START		5
#define RTC_DATE_DATE_REG		5
#define RTC_DATE_MONTH_REG		7
#define RTC_DATE_YEAR_REG		8

#endif

//uint8_t RTCSetTime( TIME * time );

uint8_t RTCGetTime( TIME * time );

//uint8_t RTCSetDate( DATE * date );

uint8_t RTCGetDate( DATE * date );

uint8_t RTCReadTimeDateRegister( uint8_t address, uint8_t *value );

uint8_t RTCWriteTimeDateRegister( uint8_t address, uint8_t value );

void InitRTC();

//uint8_t RTCSetTimeSec( uint8_t value );
//
//
//uint8_t RTCSetTimeMin( uint8_t value );
//
//
//uint8_t RTCSetTimeHour( uint8_t value );
//
//
//uint8_t RTCGetTimeSec( uint8_t *value );
//
//
//uint8_t RTCGetTimeMin( uint8_t *value );
//
//
//uint8_t RTCGetTimeHour( uint8_t *value );
//
//
//uint8_t RTCSetDateDate( uint8_t value );
//
//
//uint8_t RTCSetDateMonth( uint8_t value );
//
//
//uint8_t RTCSetDateYear( uint8_t value );
//
//
//uint8_t RTCGetDateDate( uint8_t *value );
//
//
//uint8_t RTCGetDateMonth( uint8_t *value );
//
//
//uint8_t RTCGetDateYear( uint8_t *value );

//uint8_t RTCReadDataMemory( uint8_t offset, uint8_t noOfBytes, uint8_t *value );
//uint8_t RTCWriteDataMemory( uint8_t offset, uint8_t noOfBytes, uint8_t *value );

#endif
