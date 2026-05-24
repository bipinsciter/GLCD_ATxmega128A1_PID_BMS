#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/wdt.h>
#include "avr\eeprom.h"
#include <stdio.h>        
#include <stdlib.h>        
#include <string.h>        
#include <math.h>        
#include "os_api.h"
#include "locked_printf.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "HardwareInfo.h"
#include "DeviceIO.h"
#include "ks0xxx.h"
#include "Watchdog.h"

//#include "arial_bold_14.h"  //Arial_Bold_14
//#include "Bitmap.h"       //IMAGE
//#include "corsiva_12.h"      //Corsiva_12
//#include "font12X16.h"    //Font12x16
//#include "Font12X24OnlyDigitsAndDot.h"    //Font12x16
//#include "Font12X16OnlyDigitsAndDot.h"    //Font12x16
//#include "Font_Arial_Nerrow.h"    //Font12x16
//#include "Font_Vrinda.h"    //Font12x16
//#include "font5X7.h"      //Font5x7
//#include "Font_Arial_Narrow_Mid.h"
//#include "Font_arial_Narrow_14_Full.h"
//#include "Font_arial_Narrow_Bold_45.h"
//#include "font_arial_narrow_bold_72.h"
#include "Font_Courier_14_Bold.h"
#include "FontVerdana10x24.h"    //Font12x16
#include "font_arial_narrow_bold_58.h"
#include "font6x8.h"      //Font6x8
#include "Arial18Bold.h"      //

#include "error.h"
#include "eepromAdr.h" 
#include "rtc.h" 
#include "pid.h"

#define SOFTWARE_VERSION                     206

#define TRUE 1
#define FALSE 0

#define CENTER 0
#define LEFT 1

/* Default values */
////
#define D_TIME_HOURS                          12               
#define D_TIME_MINUTES                        0                
#define D_TIME_SECONDS                        0                
#define D_DATE_YEAR                           24                
#define D_DATE_MONTH                          7                
#define D_DATE_DAY                            1   
             
#define D_DP1_OUT_LOW_COUNT					0          
#define D_DP1_OUT_HIGH_COUNT				3604          
#define D_DP2_OUT_LOW_COUNT					0              
#define D_DP2_OUT_HIGH_COUNT				3604     
#define D_DP3_OUT_LOW_COUNT					0
#define D_DP3_OUT_HIGH_COUNT				3604        
#define D_TEMP_OUT_LOW_COUNT                0
#define D_TEMP_OUT_HIGH_COUNT               3604
#define D_RH_OUT_LOW_COUNT                  0
#define D_RH_OUT_HIGH_COUNT                 3604
           
#define D_DP1_UNIT						PRESSURE_PA     
#define D_DP2_UNIT						PRESSURE_PA     
#define D_DP3_UNIT						PRESSURE_PA  
#define D_TEMPERATURE_UNIT              TEMPERATURE_DEG_C
#define D_HUMIDITY_UNIT                 HUMIDITY_RH      
#define D_LOGGING_INTERVAL              3600             
#define D_DEVICE_ID                     1
#define D_DP1_ZERO_RNG                  0                
#define D_DP2_ZERO_RNG                  0  
#define D_DP3_ZERO_RNG                  0              
#define D_DP1_ZERO_ADJ                  0                
#define D_DP2_ZERO_ADJ                  0   
#define D_DP3_ZERO_ADJ                  0              
#define D_TEMP_ZERO_ADJ                 0                
#define D_RH_ZERO_ADJ                   0  
#define D_DP1_RANGE						1220
#define D_DP2_RANGE						1220
#define D_DP3_RANGE						1220
              
#define D_BROADCAST_INTERVAL            120              
#define D_DISPLAY_INTERVAL              1
#define D_SERIAL_BAUD_RATE_VAL          SERIAL_BAUD_57600
#define D_SERIAL_DATA_STOP_PARITY_VAL   ((3<<SERIAL_DATA_BITS_SHIFT) | (0<<SERIAL_STOP_BIT_SHIFT) | (SERIAL_PARITY_NONE << SERIAL_PARITY_SHIFT))//3    // BIT 0-1 = Data bits, BIT 2 = STOP bits, BIT 3-4 = PARITY :: 8 bit, 1 stop, no parity
#define D_BUZZER_ON_TIME                1    
#define D_BUZZER_OFF_TIME               5   
#define D_BUZZER_DISABLE_TIME           0xFFFF

#define D_DP1_PID_STARTUP_PERCENT        0
#define D_DP1_PID_KP                     10
#define D_DP1_PID_TI                     10
#define D_DP1_PID_TD                     1
#define D_DP1_OUT_POLARITY				 1

#define D_DP2_PID_STARTUP_PERCENT       0
#define D_DP2_PID_KP                    10
#define D_DP2_PID_TI                    10
#define D_DP2_PID_TD                    1
#define D_DP2_OUT_POLARITY				1

#define D_TEMP_PID_STARTUP_PERCENT            0
#define D_TEMP_PID_KP                         10
#define D_TEMP_PID_TI                         10
#define D_TEMP_PID_TD                         1
#define D_TEMP_OUT_POLARITY				      1

#define D_RH_PID_STARTUP_PERCENT              0
#define D_RH_PID_KP                           10
#define D_RH_PID_TI                           10
#define D_RH_PID_TD                           1
#define D_RH_OUT_POLARITY				      1

#define D_TEMP_HIGH_PID_RH_CASCADE_FACTOR     0
#define D_TEMP_LOW_PID_RH_CASCADE_FACTOR      0
#define D_TEMP_HIGH_PID_CASCADE_RH_STATE	  0
#define D_TEMP_LOW_PID_CASCADE_RH_STATE       0
#define D_RH_HIGH_PID_TEMP_CASCADE_FACTOR     0
#define D_RH_LOW_PID_TEMP_CASCADE_FACTOR      0
#define D_RH_HIGH_PID_CASCADE_TEMP_STATE	  0
#define D_RH_LOW_PID_CASCADE_TEMP_STATE       0
#define D_TEMP_HIGH_PID_RH_ADJUST             0
#define D_TEMP_LOW_PID_RH_ADJUST              0
#define D_RH_HIGH_PID_TEMP_ADJUST             0
#define D_RH_LOW_PID_TEMP_ADJUST              0




/* MIN values */
////
#define MIN_TIME_HOURS                        0    
#define MIN_TIME_MINUTES                      0    
#define MIN_TIME_SECONDS                      0    
#define MIN_DATE_YEAR                         8    
#define MIN_DATE_MONTH                        1    
#define MIN_DATE_DAY                          1    
#define MIN_DP1_OUT_LOW_COUNT            0    
#define MIN_DP2_OUT_LOW_COUNT           0    
#define MIN_TEMP_OUT_LOW_COUNT                0    
#define MIN_RH_OUT_LOW_COUNT                  0    
#define MIN_LOGGING_INTERVAL                  60   
#define MIN_DEVICE_ID                         0    
#define MIN_BROADCAST_INTERVAL                3    
#define MIN_DISPLAY_INTERVAL                  1             
#define MIN_SERIAL_BAUD_RATE_VAL              0
#define MIN_SERIAL_DATA_STOP_PARITY_VAL       0
#define MIN_BUZZER_ON_TIME                    0    
#define MIN_BUZZER_OFF_TIME                   0    
#define MIN_BUZZER_DISABLE_TIME               (-1)
#define MIN_DP1_PID_STARTUP_PERCENT           0
#define MIN_DP1_PID_KP                   1
#define MIN_DP1_PID_TI                   1
#define MIN_DP1_PID_TD                   0
#define MIN_DP2_PID_STARTUP_PERCENT           0
#define MIN_DP2_PID_KP                  1
#define MIN_DP2_PID_TI                  1
#define MIN_DP2_PID_TD                  0
#define MIN_TEMP_PID_STARTUP_PERCENT          0
#define MIN_TEMP_PID_KP                       1
#define MIN_TEMP_PID_TI                       1
#define MIN_TEMP_PID_TD                       0
#define MIN_RH_PID_STARTUP_PERCENT            0
#define MIN_RH_PID_KP                         1
#define MIN_RH_PID_TI                         1
#define MIN_RH_PID_TD                         0
#define MIN_TEMP_HIGH_PID_RH_CASCADE_FACTOR   (-1000)
#define MIN_TEMP_LOW_PID_RH_CASCADE_FACTOR    (-1000)
#define MIN_TEMP_HIGH_PID_CASCADE_RH_STATE	  (-1)
#define MIN_TEMP_LOW_PID_CASCADE_RH_STATE     (-1)
#define MIN_RH_HIGH_PID_TEMP_CASCADE_FACTOR   (-1000)
#define MIN_RH_LOW_PID_TEMP_CASCADE_FACTOR    (-1000)
#define MIN_RH_HIGH_PID_CASCADE_TEMP_STATE	  (-1)
#define MIN_RH_LOW_PID_CASCADE_TEMP_STATE     (-1)
#define MIN_TEMP_HIGH_PID_RH_ADJUST           (-10000)
#define MIN_TEMP_LOW_PID_RH_ADJUST            (-10000)
#define MIN_RH_HIGH_PID_TEMP_ADJUST           (-10000)
#define MIN_RH_LOW_PID_TEMP_ADJUST            (-10000)



/* MAX values */
////
#define MAX_TIME_HOURS                        23
#define MAX_TIME_MINUTES                      59
#define MAX_TIME_SECONDS                      59
#define MAX_DATE_YEAR                         99
#define MAX_DATE_MONTH                        12
#define MAX_DATE_DAY                          31
#define MAX_DP1_OUT_HIGH_COUNT				  4095
#define MAX_DP2_OUT_HIGH_COUNT				  4095
#define MAX_TEMP_OUT_HIGH_COUNT               4095
#define MAX_RH_OUT_HIGH_COUNT                 4095
#define MAX_LOGGING_INTERVAL                  32767
#define MAX_DEVICE_ID                         250
#define MAX_BROADCAST_INTERVAL                32767
#define MAX_DISPLAY_INTERVAL                  30
#define MAX_SERIAL_BAUD_RATE_VAL              8
#define MAX_SERIAL_DATA_STOP_PARITY_VAL       23
#define MAX_BUZZER_ON_TIME                    60    
#define MAX_BUZZER_OFF_TIME                   3600    
#define MAX_BUZZER_DISABLE_TIME               32767  
#define MAX_DP1_PID_STARTUP_PERCENT           1000  
#define MAX_DP1_PID_KP                   32767
#define MAX_DP1_PID_TI                   32767
#define MAX_DP1_PID_TD                   32767
#define MAX_DP2_PID_STARTUP_PERCENT           1000
#define MAX_DP2_PID_KP                  32767
#define MAX_DP2_PID_TI                  32767
#define MAX_DP2_PID_TD                  32767
#define MAX_TEMP_PID_STARTUP_PERCENT          1000
#define MAX_TEMP_PID_KP                       32767
#define MAX_TEMP_PID_TI                       32767
#define MAX_TEMP_PID_TD                       32767
#define MAX_RH_PID_STARTUP_PERCENT            1000
#define MAX_RH_PID_KP                         32767
#define MAX_RH_PID_TI                         32767
#define MAX_RH_PID_TD                         32767
#define MAX_TEMP_HIGH_PID_RH_CASCADE_FACTOR   1000
#define MAX_TEMP_LOW_PID_RH_CASCADE_FACTOR    1000
#define MAX_TEMP_HIGH_PID_CASCADE_RH_STATE	  1
#define MAX_TEMP_LOW_PID_CASCADE_RH_STATE     1
#define MAX_RH_HIGH_PID_TEMP_CASCADE_FACTOR   1000
#define MAX_RH_LOW_PID_TEMP_CASCADE_FACTOR    1000
#define MAX_RH_HIGH_PID_CASCADE_TEMP_STATE	  1
#define MAX_RH_LOW_PID_CASCADE_TEMP_STATE     1
#define MAX_TEMP_HIGH_PID_RH_ADJUST           (10000)
#define MAX_TEMP_LOW_PID_RH_ADJUST            (10000)
#define MAX_RH_HIGH_PID_TEMP_ADJUST           (10000)
#define MAX_RH_LOW_PID_TEMP_ADJUST            (10000)



#define BLOCKED         0
#define SENSOR_VALUES   1
#define EDITOR          2
#define SYSTEM_LOG      3

#define NULL ((void *)0)

#define PARA_READ_VALUE_ONLY  0
#define PARA_READ             1
#define PARA_WRITE            2
#define PARA_DEFAULT          3
#define PARA_WRITE_VALUE_ONLY 4

#define CONFIG_PASSWORD           1965
#define MASTER_CONFIG_PASSWORD    31//20217



#define systemErrorFont    Font6x8 
#define smallFont          Font6x8 
#define largeFont          DispFont 
//#define largeFont          Font12X24OnlyDigitsAndDot 
                            
                                        
#define BUZZER_DIR_OP		PORTH_DIRSET = PIN0_bm
#define BUZZER_ON 			PORTH_OUTSET = PIN0_bm
#define BUZZER_OFF			PORTH_OUTCLR = PIN0_bm

#define SETDFLTSW_HIGH		PORTA_OUTSET = PIN6_bm
#define SETDFLTSW_DIR_IN	PORTA_DIRCLR = PIN6_bm
#define SETDFLTSW_IN		(PORTA_IN & PIN6_bm)

#define KEYPAD_PORT        	PORTD_OUT
#define KEYPAD_PORT_DIR    	PORTD_DIR
#define KEYPAD_PORT_IN     	PORTD_IN

#define MODE_SEL_KEY       (PIN4_bm)   
#define SETUP_KEY          (PIN5_bm)   
#define UP_KEY             (PIN6_bm)   
#define DOWN_KEY           (PIN7_bm)   
#define KEYS_EDITOR_MASK   (SETUP_KEY | UP_KEY | DOWN_KEY) 
#define KEYS_ALL_MASK      (KEYS_EDITOR_MASK | MODE_SEL_KEY) 

#define KEYBOARD_IDLE_TIME       60000u
#define KEYBOARD_LOOP_TIME       50
#define KEY_AUTO_CHG_TIME        300
#define KEY_AUTO_CHG_FAST_TIME   3000
#define KEY_AUTO_FAST_STEP       10

#define PARA_SETUP_Y_LOC         4
#define PARA_INFO_Y_LOC          48//20
#define PARA_VAL_Y_LOC           72//32

#define SCREEN_START_X_VAL    1
#define SCREEN_START_Y_VAL    0
#define DATA_Y_OFFSET    (32-SCREEN_START_Y_VAL)//(40-SCREEN_START_Y_VAL)//(10-SCREEN_START_Y_VAL)
#define UNIT_Y_OFFSET    (46-SCREEN_START_Y_VAL)
#define SYSERR_Y_OFFSET  (136-SCREEN_START_Y_VAL)//(55-SCREEN_START_Y_VAL)

//#define DP1_SENSOR_RANGE_PSI        (GetParameterValue(DP1_RANGE)/6895.7)  // Pa to PSI conversion
//#define DP2_SENSOR_RANGE_PSI        (GetParameterValue(DP2_RANGE)/6895.7)  // Pa to PSI conversion
//#define DP3_SENSOR_RANGE_PSI        (GetParameterValue(DP3_RANGE)/6895.7)  // Pa to PSI conversion

//#define DP1_SENSOR_RANGE_MILLIBAR   (DP1_SENSOR_RANGE_PSI*68.9476)    // 2 digit
//#define DP2_SENSOR_RANGE_MILLIBAR  (DP2_SENSOR_RANGE_PSI*68.9476)   // 1 digit
//#define DP1_SENSOR_RANGE_PA         (DP1_SENSOR_RANGE_PSI*6894.76)    // 3 digit
//#define DP2_SENSOR_RANGE_PA        (DP2_SENSOR_RANGE_PSI*6894.76)   // 3 digit
//#define DP1_SENSOR_RANGE_H2O_MM     (DP1_SENSOR_RANGE_PSI*703.069578295612)    // 3 digit
//#define DP2_SENSOR_RANGE_H2O_MM    (DP2_SENSOR_RANGE_PSI*703.069578295612)   // 2 digit
//#define DP1_SENSOR_RANGE_H2O_INCH   (DP1_SENSOR_RANGE_PSI*27.7076)   // 1 Digit
//#define DP2_SENSOR_RANGE_H2O_INCH  (DP2_SENSOR_RANGE_PSI*27.7076)  // 1 Digit
//#define DP1_SENSOR_RANGE_HG_INCH    (DP1_SENSOR_RANGE_PSI*2.03602)     // 0 digit
//#define DP2_SENSOR_RANGE_HG_INCH   (DP2_SENSOR_RANGE_PSI*2.03602)    // 0 digit
//#define DP1_SENSOR_RANGE_PSF        (DP1_SENSOR_RANGE_PSI*144.0)     // 2 digit
//#define DP2_SENSOR_RANGE_PSF       (DP2_SENSOR_RANGE_PSI*144.0)    // 2 digit


#define D_DP1_UPPER_ALARM_OFF_LIMIT              d_pres_abs_upper_alarm_off_limit()   
#define D_DP1_UPPER_ALARM_ON_LIMIT               d_pres_abs_upper_alarm_on_limit()    
#define D_DP1_LOWER_ALARM_OFF_LIMIT              d_pres_abs_lower_alarm_off_limit()   
#define D_DP1_LOWER_ALARM_ON_LIMIT               d_pres_abs_lower_alarm_on_limit()    
#define D_DP2_UPPER_ALARM_OFF_LIMIT             d_pres_diff_upper_alarm_off_limit()  
#define D_DP2_UPPER_ALARM_ON_LIMIT              d_pres_diff_upper_alarm_on_limit()   
#define D_DP2_LOWER_ALARM_OFF_LIMIT             d_pres_diff_lower_alarm_off_limit()  
#define D_DP2_LOWER_ALARM_ON_LIMIT              d_pres_diff_lower_alarm_on_limit()   
#define D_DP3_UPPER_ALARM_OFF_LIMIT            d_pres_diff3_upper_alarm_off_limit()
#define D_DP3_UPPER_ALARM_ON_LIMIT             d_pres_diff3_upper_alarm_on_limit()
#define D_DP3_LOWER_ALARM_OFF_LIMIT            d_pres_diff3_lower_alarm_off_limit()
#define D_DP3_LOWER_ALARM_ON_LIMIT             d_pres_diff3_lower_alarm_on_limit()
#define D_TEMP_UPPER_ALARM_OFF_LIMIT                  d_temp_upper_alarm_off_limit()       
#define D_TEMP_UPPER_ALARM_ON_LIMIT                   d_temp_upper_alarm_on_limit()        
#define D_TEMP_LOWER_ALARM_OFF_LIMIT                  d_temp_lower_alarm_off_limit()       
#define D_TEMP_LOWER_ALARM_ON_LIMIT                   d_temp_lower_alarm_on_limit()        
#define D_RH_UPPER_ALARM_OFF_LIMIT                    d_rh_upper_alarm_off_limit()         
#define D_RH_UPPER_ALARM_ON_LIMIT                     d_rh_upper_alarm_on_limit()          
#define D_RH_LOWER_ALARM_OFF_LIMIT                    d_rh_lower_alarm_off_limit()         
#define D_RH_LOWER_ALARM_ON_LIMIT                     d_rh_lower_alarm_on_limit()  

#define D_PART_MC_UPPER_ALARM_OFF_LIMIT               d_part_mc_upper_alarm_off_limit()
#define D_PART_MC_UPPER_ALARM_ON_LIMIT                d_part_mc_upper_alarm_on_limit()
#define D_PART_VOC_UPPER_ALARM_OFF_LIMIT              d_part_voc_upper_alarm_off_limit()
#define D_PART_VOC_UPPER_ALARM_ON_LIMIT               d_part_voc_upper_alarm_on_limit()
#define D_PART_NOX_UPPER_ALARM_OFF_LIMIT              d_part_nox_upper_alarm_off_limit()
#define D_PART_NOX_UPPER_ALARM_ON_LIMIT               d_part_nox_upper_alarm_on_limit()
        
#define D_DP1_OUT_LOW_PRES                       d_pres_abs_out_low_pres()            
#define D_DP1_OUT_HIGH_PRES                      d_pres_abs_out_high_pres()           
#define D_DP2_OUT_LOW_PRES                      d_pres_diff_out_low_pres()           
#define D_DP2_OUT_HIGH_PRES                     d_pres_diff_out_high_pres()          
#define D_TEMP_OUT_LOW_TEMP                           d_temp_out_low_temp()                
#define D_TEMP_OUT_HIGH_TEMP                          d_temp_out_high_temp()               
#define D_RH_OUT_LOW_RH                               d_rh_out_low_rh()                    
#define D_RH_OUT_HIGH_RH                              d_rh_out_high_rh()                   
#define D_DP1_PID_SET_VALUE							  d_pres_abs_pid_setpoint_limit()
#define D_DP2_PID_SET_VALUE							  d_pres_diff_pid_setpoint_limit()
//#define D_TEMP_PID_SET_VALUE                          ((D_TEMP_UPPER_ALARM_OFF_LIMIT + D_TEMP_UPPER_ALARM_ON_LIMIT)/2)
//#define D_RH_PID_SET_VALUE                            ((D_RH_UPPER_ALARM_OFF_LIMIT + D_RH_UPPER_ALARM_ON_LIMIT)/2)

//#define D_DP1_PID_SET_VALUE                      (0)
//#define D_DP2_PID_SET_VALUE                      (0)
#define D_TEMP_PID_SET_VALUE                     (2600)
#define D_RH_PID_SET_VALUE                       (5500)

#define D_DP1_AREA_LEN                           (600) 
#define D_DP2_AREA_LEN                           (600) 
#define D_DP3_AREA_LEN                           (600) 
#define D_DP1_AREA_WID                           (600)
#define D_DP2_AREA_WID                           (600)
#define D_DP3_AREA_WID                           (600)
#define D_DP1_AREA_RAD                           (300)
#define D_DP2_AREA_RAD                           (300)
#define D_DP3_AREA_RAD                           (300)

#define MIN_DP1_UPPER_ALARM_OFF_LIMIT            min_pres_abs_upper_alarm_off_limit() 
#define MIN_DP1_LOWER_ALARM_ON_LIMIT             min_pres_abs_lower_alarm_on_limit()  
#define MIN_DP2_UPPER_ALARM_OFF_LIMIT           min_pres_diff_upper_alarm_off_limit()
#define MIN_DP2_LOWER_ALARM_ON_LIMIT            min_pres_diff_lower_alarm_on_limit() 
#define MIN_DP3_UPPER_ALARM_OFF_LIMIT          min_pres_diff3_upper_alarm_off_limit()
#define MIN_DP3_LOWER_ALARM_ON_LIMIT           min_pres_diff3_lower_alarm_on_limit()
#define MIN_TEMP_UPPER_ALARM_OFF_LIMIT                min_temp_upper_alarm_off_limit()     
#define MIN_TEMP_LOWER_ALARM_ON_LIMIT                 min_temp_lower_alarm_on_limit()      
#define MIN_RH_UPPER_ALARM_OFF_LIMIT                  min_rh_upper_alarm_off_limit()       
#define MIN_RH_LOWER_ALARM_ON_LIMIT                   min_rh_lower_alarm_on_limit()  
#define MIN_PART_MC_UPPER_ALARM_OFF_LIMIT             min_part_mc_upper_alarm_off_limit()
#define MIN_PART_VOC_UPPER_ALARM_OFF_LIMIT            min_part_voc_upper_alarm_off_limit()
#define MIN_PART_NOX_UPPER_ALARM_OFF_LIMIT            min_part_nox_upper_alarm_off_limit()
      
#define MIN_DP1_ZERO_RNG                         0              
#define MIN_DP2_ZERO_RNG                        0         
#define MIN_DP3_ZERO_RNG                       0        
#define MIN_DP1_ZERO_ADJ                         min_abs_pres_zero_adj()              
#define MIN_DP2_ZERO_ADJ                        min_diff_pres_zero_adj()  
#define MIN_DP3_ZERO_ADJ                       min_diff_pres3_zero_adj()             
#define MIN_TEMP_ZERO_ADJ                             min_temp_zero_adj()                  
#define MIN_RH_ZERO_ADJ                               min_rh_zero_adj()                    
#define MIN_DP1_OUT_LOW_PRES                     min_pres_abs_out_low_pres()          
#define MIN_DP2_OUT_LOW_PRES                    min_pres_diff_out_low_pres()         
#define MIN_TEMP_OUT_LOW_TEMP                         min_temp_out_low_temp()              
#define MIN_RH_OUT_LOW_RH                             min_rh_out_low_rh()                  
#define MIN_DP1_PID_SET_VALUE                    MIN_DP1_UPPER_ALARM_OFF_LIMIT
#define MIN_DP2_PID_SET_VALUE                   MIN_DP2_UPPER_ALARM_OFF_LIMIT
#define MIN_TEMP_PID_SET_VALUE                        MIN_TEMP_UPPER_ALARM_OFF_LIMIT
#define MIN_RH_PID_SET_VALUE                          MIN_RH_UPPER_ALARM_OFF_LIMIT
#define MIN_DP1_AREA                             (1) 
#define MIN_DP2_AREA                            (1) 
#define MIN_DP3_AREA                            (1) 

#define MAX_DP1_UPPER_ALARM_ON_LIMIT             max_pres_abs_upper_alarm_on_limit()  
#define MAX_DP1_LOWER_ALARM_OFF_LIMIT            max_pres_abs_lower_alarm_off_limit() 
#define MAX_DP2_UPPER_ALARM_ON_LIMIT            max_pres_diff_upper_alarm_on_limit() 
#define MAX_DP2_LOWER_ALARM_OFF_LIMIT           max_pres_diff_lower_alarm_off_limit()
#define MAX_DP3_UPPER_ALARM_ON_LIMIT           max_pres_diff3_upper_alarm_on_limit()
#define MAX_DP3_LOWER_ALARM_OFF_LIMIT          max_pres_diff3_lower_alarm_off_limit()
#define MAX_TEMP_UPPER_ALARM_ON_LIMIT                 max_temp_upper_alarm_on_limit()      
#define MAX_TEMP_LOWER_ALARM_OFF_LIMIT                max_temp_lower_alarm_off_limit()     
#define MAX_RH_UPPER_ALARM_ON_LIMIT                   max_rh_upper_alarm_on_limit()        
#define MAX_RH_LOWER_ALARM_OFF_LIMIT                  max_rh_lower_alarm_off_limit()

#define MAX_PART_MC_UPPER_ALARM_ON_LIMIT              max_part_mc_upper_alarm_on_limit()
#define MAX_PART_VOC_UPPER_ALARM_ON_LIMIT             max_part_voc_upper_alarm_on_limit()
#define MAX_PART_NOX_UPPER_ALARM_ON_LIMIT             max_part_nox_upper_alarm_on_limit()

#define MAX_DP1_ZERO_RNG                         max_abs_pres_zero_rng()
#define MAX_DP2_ZERO_RNG                        max_diff_pres_zero_rng()   
#define MAX_DP3_ZERO_RNG                       max_diff_pres3_zero_rng()             
#define MAX_DP1_ZERO_ADJ                         max_abs_pres_zero_adj()              
#define MAX_DP2_ZERO_ADJ                        max_diff_pres_zero_adj()     
#define MAX_DP3_ZERO_ADJ                        max_diff_pres3_zero_adj()         
#define MAX_TEMP_ZERO_ADJ                             max_temp_zero_adj()                  
#define MAX_RH_ZERO_ADJ                               max_rh_zero_adj()                    
#define MAX_DP1_OUT_HIGH_PRES                    max_pres_abs_out_high_pres()         
#define MAX_DP2_OUT_HIGH_PRES                   max_pres_diff_out_high_pres()        
#define MAX_TEMP_OUT_HIGH_TEMP                        max_temp_out_high_temp()             
#define MAX_RH_OUT_HIGH_RH                            max_rh_out_high_rh()                 
#define MAX_DP1_PID_SET_VALUE                    MAX_DP1_UPPER_ALARM_ON_LIMIT
#define MAX_DP2_PID_SET_VALUE                   MAX_DP2_UPPER_ALARM_ON_LIMIT
#define MAX_PRES_DIF3F_PID_SET_VALUE                  MAX_DP3_UPPER_ALARM_ON_LIMIT
#define MAX_TEMP_PID_SET_VALUE                        MAX_TEMP_UPPER_ALARM_ON_LIMIT
#define MAX_RH_PID_SET_VALUE                          MAX_RH_UPPER_ALARM_ON_LIMIT
#define MAX_DP1_AREA                             (32766) 
#define MAX_DP2_AREA                             (32766) 
#define MAX_DP3_AREA                             (32766) 

int d_pres_abs_upper_alarm_off_limit();
int d_pres_abs_upper_alarm_on_limit();      
int d_pres_abs_lower_alarm_off_limit();     
int d_pres_abs_lower_alarm_on_limit();      
int d_pres_diff_upper_alarm_off_limit();    
int d_pres_diff_upper_alarm_on_limit();     
int d_pres_diff_lower_alarm_off_limit();    
int d_pres_diff_lower_alarm_on_limit();   
int d_pres_diff3_upper_alarm_off_limit();
int d_pres_diff3_upper_alarm_on_limit();
int d_pres_diff3_lower_alarm_off_limit();
int d_pres_diff3_lower_alarm_on_limit();  
int d_temp_upper_alarm_off_limit();         
int d_temp_upper_alarm_on_limit();          
int d_temp_lower_alarm_off_limit();         
int d_temp_lower_alarm_on_limit();          
int d_rh_upper_alarm_off_limit();           
int d_rh_upper_alarm_on_limit();            
int d_rh_lower_alarm_off_limit();           
int d_rh_lower_alarm_on_limit();   
int d_pres_abs_pid_setpoint_limit();
int d_pres_diff_pid_setpoint_limit();

int d_part_mc_upper_alarm_off_limit();
int d_part_mc_upper_alarm_on_limit();
int d_part_voc_upper_alarm_off_limit();
int d_part_voc_upper_alarm_on_limit();
int d_part_nox_upper_alarm_off_limit();
int d_part_nox_upper_alarm_on_limit();
		 
int d_pres_abs_out_low_pres();              
int d_pres_abs_out_high_pres();             
int d_pres_diff_out_low_pres();            
int d_pres_diff_out_high_pres();            
int d_temp_out_low_temp();                  
int d_temp_out_high_temp();                 
int d_rh_out_low_rh();                      
int d_rh_out_high_rh();                     
int min_pres_abs_upper_alarm_off_limit();   
int min_pres_abs_lower_alarm_on_limit();    
int min_pres_diff_upper_alarm_off_limit();  
int min_pres_diff_lower_alarm_on_limit();   
int min_pres_diff3_upper_alarm_off_limit();
int min_pres_diff3_lower_alarm_on_limit();
int min_temp_upper_alarm_off_limit();       
int min_temp_lower_alarm_on_limit();        
int min_rh_upper_alarm_off_limit();         
int min_rh_lower_alarm_on_limit();    
int min_part_mc_upper_alarm_off_limit();
int min_part_voc_upper_alarm_off_limit();
int min_part_nox_upper_alarm_off_limit();
      
int min_abs_pres_zero_adj();                
int min_diff_pres_zero_adj();     
int min_diff_pres3_zero_adj();             
int min_temp_zero_adj();                    
int min_rh_zero_adj();                      
int min_pres_abs_out_low_pres();            
int min_pres_diff_out_low_pres();           
int min_temp_out_low_temp();                
int min_rh_out_low_rh();                    
int max_pres_abs_upper_alarm_on_limit();    
int max_pres_abs_lower_alarm_off_limit();   
int max_pres_diff_upper_alarm_on_limit();   
int max_pres_diff_lower_alarm_off_limit();  
int max_pres_diff3_upper_alarm_on_limit();
int max_pres_diff3_lower_alarm_off_limit();
int max_temp_upper_alarm_on_limit();        
int max_temp_lower_alarm_off_limit();       
int max_rh_upper_alarm_on_limit();          
int max_rh_lower_alarm_off_limit();    

int max_part_mc_upper_alarm_on_limit();
int max_part_mc_upper_alarm_off_limit();
int max_part_voc_upper_alarm_on_limit();
int max_part_voc_upper_alarm_off_limit();
int max_part_nox_upper_alarm_on_limit();
int max_part_nox_upper_alarm_off_limit();
     
int max_abs_pres_zero_rng();                
int max_diff_pres_zero_rng();   
int max_diff_pres3_zero_rng();              
int max_abs_pres_zero_adj();                
int max_diff_pres_zero_adj();  
int max_diff_pres3_zero_adj();              
int max_temp_zero_adj();                    
int max_rh_zero_adj();                      
int max_pres_abs_out_high_pres();           
int max_pres_diff_out_high_pres();          
int max_temp_out_high_temp();               
int max_rh_out_high_rh();                   

int findDewPoint(int Tempvalue, int RHvalue);
static void StartDisplaySensorValues();
static void DisplayEditor(char keysPressed, char keys);
static void DisplayLog(char keysPressed, char keys);

static void PrintLine( uint8_t align, char * str );
static void PrintLineCentered( uint8_t yPos, char * str );
static void PrintLineInBox( uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str );
static void PrintLineInBoxWOClear(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str );

static char * PresAbsUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiff3UpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiff3UpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiff3LowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiff3LowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * TempUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * TempUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue );
//static char * DisplayIntervalPara(uint8_t operationType, int16_t * paraValue );
//static char * DisplayModePara(uint8_t operationType, int16_t * paraValue );
static char * TimeHourPara( uint8_t operationType, int16_t * paraValue );
static char * TimeMinutePara( uint8_t operationType, int16_t * paraValue );
static char * TimeSecondPara( uint8_t operationType, int16_t * paraValue );
static char * DateYearPara( uint8_t operationType, int16_t * paraValue );
static char * DateMonthPara( uint8_t operationType, int16_t * paraValue );
static char * DateDayPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * TempValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * TempValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForPresAbsMinimumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForPresAbsMaximumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForPresDiffMinimumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForPresDiffMaximumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForTempMinimumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForTempMaximumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForHumidityMinimumPara( uint8_t operationType, int16_t * paraValue );
static char * OutputCountsForHumidityMaximumPara( uint8_t operationType, int16_t * paraValue );
static char * PressureAbsUnitPara( uint8_t operationType, int16_t * paraValue );
static char * PressureDiffUnitPara( uint8_t operationType, int16_t * paraValue );
static char * PressureDiff3UnitPara( uint8_t operationType, int16_t * paraValue );
static char * TemperatureUnitPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityUnitPara( uint8_t operationType, int16_t * paraValue );
//static char * PresAbsAreaPara( uint8_t operationType, int16_t * paraValue );
//static char * PresDiffAreaPara( uint8_t operationType, int16_t * paraValue );
//static char * PresDiff3AreaPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * Humidity2ZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresZeroRangePara(uint8_t operationType, int16_t * paraValue );
static char * DiffPresZeroRangePara(uint8_t operationType, int16_t * paraValue );
static char * DiffPres3ZeroRangePara(uint8_t operationType, int16_t * paraValue );
static char * AbsPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * DiffPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * DiffPres3ZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * TempZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * Temp2ZeroAdjustPara(uint8_t operationType, int16_t * paraValue );
static char * TempDiffAlarmPara(uint8_t operationType, int16_t * paraValue );
static char * TempFireAlmSetPara(uint8_t operationType, int16_t * paraValue );
static char * TempFireAlmTimePara(uint8_t operationType, int16_t * paraValue );
static char * ConfigPasswordPara(uint8_t operationType, int16_t * paraValue );
//static char * LogggingIntervalPara(uint8_t operationType, int16_t * paraValue );
static char * DeviceIdPara(uint8_t operationType, int16_t * paraValue );
static char * BroadcastIntervalPara(uint8_t operationType, int16_t * paraValue );
static char * SerialBaudratePara(uint8_t operationType, int16_t * paraValue );
static char * SerialDataStopParityPara(uint8_t operationType, int16_t * paraValue );
static char * PresAbsRange(uint8_t operationType, int16_t * paraValue );
static char * PresDiffRange(uint8_t operationType, int16_t * paraValue );
static char * PresDiff3Range(uint8_t operationType, int16_t * paraValue );
//static char * PresAbsSensMinCnt(uint8_t operationType, int16_t * paraValue );
//static char * PresAbsSensMaxCnt(uint8_t operationType, int16_t * paraValue );
//static char * PresDiffSensMinCnt(uint8_t operationType, int16_t * paraValue );
//static char * PresDiffSensMaxCnt(uint8_t operationType, int16_t * paraValue );
//static char * PresDiff3SensMinCnt(uint8_t operationType, int16_t * paraValue );
//static char * PresDiff3SensMaxCnt(uint8_t operationType, int16_t * paraValue );
static char * BuzzerOnPara( uint8_t operationType, int16_t * paraValue );
static char * BuzzerOffPara( uint8_t operationType, int16_t * paraValue );
static char * BuzzerDisablePara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsOutputTypePara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffOutputTypePara( uint8_t operationType, int16_t * paraValue );
static char * TempOutputTypePara( uint8_t operationType, int16_t * paraValue );
static char * HumidityOutputTypePara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsPIDSetpointPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffPIDSetpointPara( uint8_t operationType, int16_t * paraValue );
static char * TempPIDSetpointPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityPIDSetpointPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsPIDKpPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsPIDTiPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsPIDTdPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffPIDKpPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffPIDTiPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffPIDTdPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * TempPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue );
static char * TempPIDKpPara( uint8_t operationType, int16_t * paraValue );
static char * TempPIDTiPara( uint8_t operationType, int16_t * paraValue );
static char * TempPIDTdPara( uint8_t operationType, int16_t * paraValue );
static char * TempOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityPIDKpPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityPIDTiPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityPIDTdPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * TempRHSensTypePara( uint8_t operationType, int16_t * paraValue );
static char * TempRH2SensTypePara( uint8_t operationType, int16_t * paraValue );
//static char * ParticalAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * AHUIdPara(uint8_t operationType, int16_t * paraValue );
static char * AHUCFMPara(uint8_t operationType, int16_t * paraValue );
static char * AHUArea1Para(uint8_t operationType, int16_t * paraValue );
static char * AHUArea2Para(uint8_t operationType, int16_t * paraValue );
static char * AHUArea3Para(uint8_t operationType, int16_t * paraValue );

static char * DP1AreaTypePara( uint8_t operationType, int16_t * paraValue );
static char * DP2AreaTypePara( uint8_t operationType, int16_t * paraValue );
static char * DP3AreaTypePara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaLengthPara( uint8_t operationType, int16_t * paraValue );
static char * DP2AreaLengthPara( uint8_t operationType, int16_t * paraValue );
static char * DP3AreaLengthPara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaWidthPara( uint8_t operationType, int16_t * paraValue );
static char * DP2AreaWidthPara( uint8_t operationType, int16_t * paraValue );
static char * DP3AreaWidthPara( uint8_t operationType, int16_t * paraValue );
static char * DP1AreaRadiousPara( uint8_t operationType, int16_t * paraValue );
static char * DP2AreaRadiousPara( uint8_t operationType, int16_t * paraValue );
static char * DP3AreaRadiousPara( uint8_t operationType, int16_t * paraValue );

static char * PresAbsAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiff3AlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * TempAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityAlarmSetupPara( uint8_t operationType, int16_t * paraValue );
static char * RTCSetupPara( uint8_t operationType, int16_t * paraValue );
static char * PresAbsOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * PresDiffOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * TempOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityOutPolarityPara( uint8_t operationType, int16_t * paraValue );
static char * TempHighPIDRHCascadeFactorPara( uint8_t operationType, int16_t * paraValue );
static char * TempHighPIDRHCascadeStatePara( uint8_t operationType, int16_t * paraValue );
static char * TempLowPIDRHCascadeFactorPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowPIDRHCascadeStatePara( uint8_t operationType, int16_t * paraValue );
static char * HumidityHighPIDTempCascadeFactorPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityHighPIDTempCascadeStatePara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowPIDTempCascadeFactorPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowPIDTempCascadeStatePara( uint8_t operationType, int16_t * paraValue );
static char * TempHighPIDRHAdjustPara( uint8_t operationType, int16_t * paraValue );
static char * TempLowPIDRHAdjustPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityHighPIDTempAdjustPara( uint8_t operationType, int16_t * paraValue );
static char * HumidityLowPIDTempAdjustPara( uint8_t operationType, int16_t * paraValue );
//static char * TempRHScanTimePara(uint8_t operationType, int16_t * paraValue );
//static char * TempRHAverageCntPara(uint8_t operationType, int16_t * paraValue );
//static char * PresAverageCntPara(uint8_t operationType, int16_t * paraValue );
//static char * PresAbsSensTypePara(uint8_t operationType, int16_t * paraValue );
//static char * PresDiffSensTypePara(uint8_t operationType, int16_t * paraValue );
//static char * PresDiff3SensTypePara(uint8_t operationType, int16_t * paraValue );
//static char * ParticalSensTypePara(uint8_t operationType, int16_t * paraValue );
//static char * ParticalSizeTypePara(uint8_t operationType, int16_t * paraValue );

static char * VersionPara(uint8_t operationType, int16_t * paraValue );
static char * CustomerIDPara(uint8_t operationType, int16_t * paraValue );
static char * SerialNoPara(uint8_t operationType, int16_t * paraValue );
static char * SystemConfigPara(uint8_t operationType, int16_t * paraValue );


static void ParameterOperationsOnLowSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t minValue, int16_t maxValParaId, unsigned int eepromAddr );
static void ParameterOperationsOnHighSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t maxValue, int16_t minValParaId, unsigned int eepromAddr );
static char * ParameterOperationsTimeValues( uint8_t mask, uint8_t maskWriteVal, uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int8_t minValue, int8_t maxValue, uint8_t regAddr );
static char * PressureUnitProcessPara( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, unsigned int eepromAddr );
static char * ParaSetIntegerVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
static char * ParaSetIntegerVal1(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
static char * ParaOutputType( unsigned int eepromAddr, uint8_t operationType, int16_t * paraValue );
static char * AlarmSetupPara( uint8_t operationType, int16_t * paraValue, unsigned int eeAddress );
static void ParameterOperationsSettings( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltValue, int16_t maxValue, int16_t minValue );
static void ParameterOperationsSettingsRollover( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal );
//static void ParameterOperationsSettingsRolloverNoDefault( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t minVal, int16_t maxVal );
static char * SetOutPolarityPara( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal );

static char * Output1MappingPara( uint8_t operationType, int16_t * paraValue );
static char * Output2MappingPara( uint8_t operationType, int16_t * paraValue );
static char * Output3MappingPara( uint8_t operationType, int16_t * paraValue );
static char * Output4MappingPara( uint8_t operationType, int16_t * paraValue );
static char * OutputMappingPara( uint8_t operationType, int16_t * paraValue, int16_t dfltVal, unsigned int eeAddress );
static char * OutputMappingPara1( uint8_t operationType, int16_t * paraValue, int16_t dfltVal, unsigned int eeAddress );

//static void FormatAbsPressure( uint8_t unit, int16_t value);
//static void FormatDiffPressure( uint8_t unit, int16_t value);
//static void FormatDiff3Pressure( uint8_t unit, int16_t value);
static void FormatDP(uint8_t senType, uint8_t unit, int16_t value, int16_t dpsenResolution, int16_t range1, uint8_t AreaType, int16_t length, int16_t width, int16_t radious);
static void FormatTemperature( int16_t value);
static void FormatHumidity( int16_t value);
static void FormatFloat(int16_t value);
//static void FormatPartical( uint16_t value);


//static void FormatControlOutput( char * str, int value);
static void FormatError( int8_t error );
//static void FormatDispError( int8_t error );
static char * FormatInteger( int16_t value);
static char * FormatVersion();

static uint8_t IsMasterReset();
static uint8_t IsConfigPassword();
static uint8_t NotConfigPassword();

static uint8_t IsPresAbsOutEnabled();
static uint8_t IsPresAbsPIDEnabled();
static uint8_t IsPresDiffOutEnabled();
static uint8_t IsPresDiffPIDEnabled();
static uint8_t IsTempOutEnabled();
static uint8_t IsTempPIDEnabled();
static uint8_t IsRHOutEnabled();
static uint8_t IsRHPIDEnabled();
static uint8_t IsPresAbsOutActive();
static uint8_t IsPresDiffOutActive();
static uint8_t IsTempOutActive();
static uint8_t IsRHOutActive();
static uint8_t IsTempOrRHEnabled();
static uint8_t IsPresAbsAlarmEnabled();
static uint8_t IsPresDiffAlarmEnabled();
static uint8_t IsPresDiff3AlarmEnabled();
static uint8_t IsTempAlarmEnabled();
static uint8_t IsHumidityAlarmEnabled();
static uint8_t IsRTCEnabled();
//static uint8_t IsParticalAlarmEnabled();
static uint8_t IsTempRHPIDEnabled();
//static uint8_t IsPresEnabled();

static void SetAllParametersToDefault();

static uint8_t GetParaInfo( PARAMETER reqId, PARA_INFO * paraInfoVal );
static uint8_t GetParaInfoByIndex( uint8_t index, PARA_INFO * paraInfoVal );

unsigned char systemError = ERROR_OK;

static uint8_t displayPage = 0,displayPage1 = 0;

static unsigned char LCDControl = BLOCKED;

unsigned char systemXLoc;
unsigned char systemYLoc;

unsigned char presAbsYLoc_disp_mode_all_para;
unsigned char presDiffYLoc_disp_mode_all_para;
unsigned char presDiff3YLoc_disp_mode_all_para;
unsigned char tempYLoc_disp_mode_all_para;
unsigned char humidityYLoc_disp_mode_all_para;
unsigned char ParticalYLoc_disp_mode_all_para;

static uint8_t isMasterReset;

static int16_t configPassword;

static int tempUnit, humidityUnit;
//static uint8_t gu8_fontcolor=WHITE;
static char displayStr[30];
static char formatedValue[12];
static const char *formatedUnit;
//static const char * formatedtype;
char gu8_inOutput[16][10]={0};
int PreTd=0,PostTd=0,TDdiff=0;
uint8_t PreTdErr=0,PostTdErr=0,TDdiffErr=0;
	
static char prevKeys, currKeys, keys, keysPressed;

//static uint8_t timer, curDisplay;
static SENSOR_VALUE senVal,senVal2,senVal3;
//static ALARMS alarmOut;
//static sen5x_measured_value_t sen55;
//static uint8_t dispList[6], noOfSensors;

static OSSemaMutex UIMutex;

unsigned int keybrdIdleTimer;

#define USERTASK_STACK_SIZE   300
#define USERTASK_PRIORITY     3//3

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#ifdef OS_AVRX
static TimerControlBlock uioSleepTimer;
AVRX_GCC_TASKDEF(UserInterfaceTask, USERTASK_STACK_SIZE, USERTASK_PRIORITY)
#else
void UserInterfaceTask( void * taskPara );
void UserInterfaceTask( void * taskPara )
#endif
{
	static uint8_t DPautocale=0;
	
	wdt_reset();
	
	#ifdef OS_AVRX
	AvrXDelay(&uioSleepTimer, 2000);
	#else
	OSSleep(2000);
	#endif

	keybrdIdleTimer = 0;
	StartDisplaySensorValues();
   
	while (1)
	{
		//wdt_reset();			//Serve Watchdog Timer
	   
		#ifdef OS_AVRX
		AvrXDelay(&uioSleepTimer, KEYBOARD_LOOP_TIME);
		#else
		OSSleep(KEYBOARD_LOOP_TIME);
		#endif

		currKeys = (~KEYPAD_PORT_IN) & KEYS_ALL_MASK;
		keysPressed = keys;
		keys |= (currKeys & prevKeys);
		keys &= (currKeys | prevKeys);
		prevKeys = currKeys;

		keysPressed = (keys^keysPressed) & keys;

		if( keysPressed & MODE_SEL_KEY )
		{
			if( LCDControl == SENSOR_VALUES )
			{
				DisplayEditor( -1, -1 );
			}
			else if ( LCDControl == EDITOR )
			{
				if( IsLoggingEnabled() )
				{
					DisplayLog(-1, -1);
				}
				else
				{
					StartDisplaySensorValues();
				}
			}
			else
			{
				StartDisplaySensorValues();
			}
			keybrdIdleTimer = 0;
		}

		if( LCDControl == SENSOR_VALUES )
		{
			if(( keysPressed & UP_KEY ) && ( keysPressed & DOWN_KEY ))
			{
				SetBuzzerSuppressionReq();   // Suppress buzzer request (valid when displaying sensor values)
			}
			else if( !(KEYPAD_PORT_IN & (SETUP_KEY | DOWN_KEY) ))	//Auto DP calibration
			{
				if(!DPautocale)
				{
					int16_t ErrorCount;
					GetPareValue(DP1_VAL_INDEX, &senVal3);
					ErrorCount = (DP1_SENSOR_RESOLUTION_SM9543/2) - senVal3.rawValue;
					OSWriteEEPromWord((unsigned int *)EA_DP1_ZERO_ADJ, ErrorCount);
				
					GetPareValue(DP2_VAL_INDEX, &senVal3);
					ErrorCount = (DP2_SENSOR_RESOLUTION_SM9543/2) - senVal3.rawValue; 
					OSWriteEEPromWord((unsigned int *)EA_DP2_ZERO_ADJ, ErrorCount);
				
					GetPareValue(DP3_VAL_INDEX, &senVal3);
					ErrorCount = (DP3_SENSOR_RESOLUTION_SM9543/2) - senVal3.rawValue;
					OSWriteEEPromWord((unsigned int *)EA_DP3_ZERO_ADJ, ErrorCount);
				
					DPautocale = 1;
				}
				
				displayPage1 = 4;
			}
			else if( !(KEYPAD_PORT_IN & SETUP_KEY ))
			{
				displayPage1 = 1;
			}
			else if( !(KEYPAD_PORT_IN & UP_KEY ))
			{
				displayPage1 = 2;
			}
			else if( !(KEYPAD_PORT_IN & DOWN_KEY ))
			{
				displayPage1 = 3;
			}
			else
			{
				displayPage1 = 0;
				DPautocale = 0;
			}
			continue;
		}

		if( LCDControl == EDITOR )
		currKeys = KEYS_EDITOR_MASK;
		else if( LCDControl == SYSTEM_LOG )
		currKeys = (UP_KEY | DOWN_KEY );

		if(( keys & currKeys ) == 0)
		{
			keybrdIdleTimer++;
			if( keybrdIdleTimer >= KEYBOARD_IDLE_TIME/KEYBOARD_LOOP_TIME)
			StartDisplaySensorValues();
		}
		else
		{
			keybrdIdleTimer = 0;
			
			if( LCDControl == EDITOR )
				DisplayEditor( keysPressed & KEYS_EDITOR_MASK, keys & KEYS_EDITOR_MASK );
			else if( LCDControl == SYSTEM_LOG )
				DisplayLog( keysPressed & ( UP_KEY | DOWN_KEY ), keys & ( UP_KEY | DOWN_KEY ));
		}
	}
}


void UserInterfaceInit()
{
   isMasterReset = 0;
   configPassword = 0;

   KEYPAD_PORT_DIR  &= ~KEYS_ALL_MASK;
   KEYPAD_PORT |= KEYS_ALL_MASK;

   SETDFLTSW_DIR_IN;
   SETDFLTSW_HIGH;

   BUZZER_DIR_OP;
   BUZZER_OFF;

   prevKeys = 0;
   currKeys = 0;
   keys = 0;
   keysPressed = 0;

   if (IsLCDDisplay())
   {
      systemXLoc = 0;
      OSSemaCreateMutex(UIMutex);
      OSSemaGive(UIMutex);
      OSSemaTakeEver(UIMutex);
	  
      // Initialize the LCD
      uc1638_Init();	
	  
	  // Select a font
	  ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
	  // Print SAP India
	  locked_sprintf_P( displayStr, PSTR("info@sapindia.in"));
	  PrintLineCentered( 16, displayStr );
	  locked_sprintf_P( displayStr, PSTR("NIYAMA"));
	  PrintLineCentered( 48, displayStr );
	  locked_sprintf_P( displayStr, PSTR("Version: %s"), FormatVersion());
	  PrintLineCentered( 80, displayStr );
	
	  ks0xxx_DrawVertLine(0, 0, GLCD_PIXEL_Y-1, BLACK);
	  ks0xxx_DrawVertLine(GLCD_PIXEL_X-1, 0, GLCD_PIXEL_Y-1, BLACK);
	  
	  ks0xxx_DrawHoriLine(1, 0, GLCD_PIXEL_X-2, BLACK);
	  ks0xxx_DrawHoriLine(1, GLCD_PIXEL_Y-1, GLCD_PIXEL_X-2, BLACK);
	  
      systemXLoc = SCREEN_START_X_VAL;
      systemYLoc = SCREEN_START_Y_VAL;
	  
      if(IsMasterReset())
      {
         // Print Master Reset
         locked_sprintf_P( displayStr, PSTR("Master Reset"));
         PrintLineCentered( 112, displayStr );
         isMasterReset = 1;
         SetAllParametersToDefault();
      }
	  
      OSSemaGive(UIMutex);
	  
	  wdt_reset();
   }
#ifdef OS_AVRX
   AvrXRunTask(TCB(UserInterfaceTask));
#else
   /* Spawn the producer and consumer. */
   xTaskCreate( UserInterfaceTask, "UI", USERTASK_STACK_SIZE, ( void * ) 0, USERTASK_PRIORITY, NULL );
#endif
}

static void ShowFixText(void)
{
	//-------------------------------------------------------------------------
	ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, BLACK);
	PrintLineInBox(LEFT,4,80,8,"AHU:");
	PrintLineInBox(LEFT,84,160,8,"CFM:");
	PrintLineInBox(LEFT,164,GLCD_PIXEL_X-1,8,"ID:");
	
	if(!IsTemperature2Enabled())
	{
		PrintLineInBox(LEFT,4,80,144,"Area1:");
		PrintLineInBox(LEFT,84,160,144,"Area2:");
		PrintLineInBox(LEFT,164,GLCD_PIXEL_X-1,144,"Area3:");
	}
	
	//INPUT
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,40,"1-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,48,"2-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,56,"3-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,64,"4-");
	//PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,72,"DOOR:");
	//OUTPUT
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,104,"1-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,112,"2-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,120,"3-");
	PrintLineInBox(LEFT,165,GLCD_PIXEL_X-1,128,"4-");
	
	ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
	PrintLineInBoxWOClear(CENTER,160,GLCD_PIXEL_X-1,24,"INPUT");
	PrintLineInBoxWOClear(CENTER,160,GLCD_PIXEL_X-1,88,"OUTPUT");
}

//static void DrawVarText(void)
//{
	//ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
	//
	//PrintLineInBox(LEFT,4,80,40,"-999.9");
	//PrintLineInBox(LEFT,4,80,80,"-999.9");
	//PrintLineInBox(LEFT,4,80,120,"-999.9");
	//
	//PrintLineInBox(LEFT,84,160,40,"-999.9");
	//PrintLineInBox(LEFT,84,160,80,"-999.9");
	//PrintLineInBox(LEFT,84,160,120,"24:59:59");
//}

static void ShowSetValue(void)
{
	memset(displayStr,0,sizeof(displayStr));
	//---------------------------------------------------------------------
	ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, BLACK);
	PrintLineInBox(LEFT,32,80,8,FormatInteger (GetParameterValue( AHU_ID )));
	PrintLineInBox(LEFT,112,160,8,FormatInteger (GetParameterValue( AHU_CFM )));
	PrintLineInBox(LEFT,184,GLCD_PIXEL_X-1,8,FormatInteger (GetParameterValue( DEVICE_ID )));
	
	if(!IsTemperature2Enabled())
	{
		PrintLineInBox(LEFT,45,80,144,FormatInteger (GetParameterValue( AHU_AREA1 )));
		PrintLineInBox(LEFT,125,160,144,FormatInteger (GetParameterValue( AHU_AREA2 )));
		PrintLineInBox(LEFT,205,GLCD_PIXEL_X-1,144,FormatInteger (GetParameterValue( AHU_AREA3 )));
	}
}

static void ShowIOValue(void)
{ 
	uint8_t inputs,outputs; 
	
	GetInput(&inputs);
	GetOutput(&outputs);
	
	if(!(inputs & PIN0_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,40,&gu8_inOutput[0][0]);
	}
	else 
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,40,&gu8_inOutput[1][0]);
	}
	
	if(!(inputs & PIN1_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,48,&gu8_inOutput[2][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,48,&gu8_inOutput[3][0]);
	}
	
	if(!(inputs & PIN2_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,56,&gu8_inOutput[4][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,56,&gu8_inOutput[5][0]);
	}
	
	if(!(inputs & PIN3_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,64,&gu8_inOutput[6][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,64,&gu8_inOutput[7][0]);
	}
	
	if(!(outputs & PIN0_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,104,&gu8_inOutput[8][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,104,&gu8_inOutput[9][0]);
	}
	
	if(!(outputs & PIN1_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,112,&gu8_inOutput[10][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,112,&gu8_inOutput[11][0]);
	}
	
	if(!(outputs & PIN2_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,120,&gu8_inOutput[12][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,120,&gu8_inOutput[13][0]);
	}
	
	if(!(outputs & PIN3_bm))
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,128,&gu8_inOutput[14][0]);
	}
	else
	{
		PrintLineInBox(LEFT,180,GLCD_PIXEL_X-1,128,&gu8_inOutput[15][0]);
	}
	
	//if(CurrentDoorStatus()==CLOSE)
	//PrintLineInBox(LEFT,202,GLCD_PIXEL_X-1,72,"CLOSE");
	//else
	//PrintLineInBox(LEFT,202,GLCD_PIXEL_X-1,72,"OPEN");
}

static void DrawTable(void)
{
	//ks0xxx_DrawVertLine(0, 0, GLCD_PIXEL_Y, BLACK);
	//ks0xxx_DrawVertLine(GLCD_PIXEL_X-1, 0, GLCD_PIXEL_Y, BLACK);
	
	//ks0xxx_DrawHoriLine(1, 0, GLCD_PIXEL_X-1, BLACK);
	//ks0xxx_DrawHoriLine(1, GLCD_PIXEL_Y-1, GLCD_PIXEL_X-1, BLACK);
	
	//ks0xxx_DrawVertLine(80, 16, GLCD_PIXEL_Y-16, BLACK);
	//ks0xxx_DrawVertLine(160, 16, GLCD_PIXEL_Y-16, BLACK);

	ks0xxx_DrawHoriLine(0, 20, GLCD_PIXEL_X-1, BLACK);
	ks0xxx_DrawHoriLine(0, 60, GLCD_PIXEL_X-81, BLACK);
	ks0xxx_DrawHoriLine(0, 100, GLCD_PIXEL_X-81, BLACK);
	ks0xxx_DrawHoriLine(0, 140, GLCD_PIXEL_X-1, BLACK);
	
	ks0xxx_FillRect(0,20,GLCD_PIXEL_X-81,15,BLACK);
	ks0xxx_FillRect(0,60,GLCD_PIXEL_X-81,15,BLACK);
	ks0xxx_FillRect(0,100,GLCD_PIXEL_X-81,15,BLACK);
	ks0xxx_FillRect(160,20,GLCD_PIXEL_X-161,15,BLACK);
	ks0xxx_FillRect(160,82,GLCD_PIXEL_X-161,17,BLACK);
}

static void StartDisplaySensorValues(void)
{
	OSSemaTakeEver(UIMutex);
	uc1638_FillScreen(BLANK1);
	DrawTable();
	ShowFixText();
	OSSemaGive(UIMutex);
	
	LCDControl = SENSOR_VALUES;
}

void DisplaySensorValues(void)
{
	static char noSystemErrDisp = 0;
	uint8_t errorCode;
	int value1=0,PreTempvalue=0,PostTempvalue=0;
	int PreRHvalue=0,PostRHvalue=0;
	
	
	if (IsLCDDisplay())
	{
		if ( LCDControl != SENSOR_VALUES )
		return;
		
		if(u8_resetTimer)
		{
			uc1638_FillScreen(BLANK1);
			ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DFU MODE >");
			
			u8_resetTimer--;
			if(!u8_resetTimer)
			{
				//SoftReset ------------------
				CPU_CCP  = CCP_IOREG_gc;
				RST.CTRL = RST_SWRST_bm ;
			}

			return;
		}
		
		ALARMS alarmOut;
		static uint8_t toggle=0,clear=0;
		alarmOut.alarmByte = GetAlarms();
		if((alarmOut.alarm.fire==1) || (alarmOut.alarm.door==1))
		{
			uc1638_FillScreen(BLANK1);
			toggle ^= 1;
			if(toggle)
			{
				//ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
				ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
				if(alarmOut.alarm.fire==1) PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< FIRE DETECTED >");
				else if(alarmOut.alarm.door==1) PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DOOR OPEN >");
				else
				{
					PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,40,"< FIRE DETECTED >");
					PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"< DOOR OPEN >");
				}
			}
			
			if(GetParameterValue(OUTPUT1_MAPPING)==OUTPUT1_NORMAL)
			{
				OUTPUT1_HIGH;
			}
			else
			{
				OUTPUT1_LOW;
			}

			clear=1;
			return;
		}
		else
		{
			if(GetParameterValue(OUTPUT1_MAPPING)==OUTPUT1_NORMAL)
			{
				OUTPUT1_LOW;
			}
			else
			{
				OUTPUT1_HIGH;
			}
		}

		if(displayPage1==4)
		{
			uc1638_FillScreen(BLANK1);
			ks0xxx_SelectFont(Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			PrintLineInBoxWOClear(CENTER,0,GLCD_PIXEL_X-1,60,"DP AUTO CAL DONE");
			
			clear=1;
			return;
		}
		
		if(clear==1)
		{
			StartDisplaySensorValues();
			clear=0;
		}
		
		memset(displayStr,0,sizeof(displayStr));
		displayPage=displayPage1;

		//---------------------------------------------------------------------
		GetPareValue(DP1_VAL_INDEX, &senVal);//GetDP1( &senVal );
		if( abs(senVal.convertedValue-(DP1_SENSOR_RESOLUTION_SM9543 / 2)) < GetParameterValue( DP1_ZERO_RNG ))
		senVal.convertedValue = (DP1_SENSOR_RESOLUTION_SM9543/2);
		//noSystemErrDisp = DisplayDP1(senVal.errorCode, senVal.convertedValue);
		
		if(!displayPage)
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		else if(displayPage==1)
		{
			value1 = GetParameterValue( DP1_UPPER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else if(displayPage==2)
		{
			value1 = GetParameterValue( DP1_LOWER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else
		{
			value1 = GetParameterValue( DP1_PID_SET_VALUE );
			errorCode = ERROR_OK;
		}
		
		noSystemErrDisp = DisplayDP1(errorCode, value1);
		//---------------------------------------------------------------------
		GetPareValue(DP2_VAL_INDEX, &senVal);//GetDP2( &senVal );
		if( abs(senVal.convertedValue-(DP2_SENSOR_RESOLUTION_SM9543 / 2)) < GetParameterValue( DP2_ZERO_RNG ))
		senVal.convertedValue = (DP2_SENSOR_RESOLUTION_SM9543/2);
		//noSystemErrDisp = DisplayDP2(senVal.errorCode, senVal.convertedValue);
		
		if(!displayPage)
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		else if(displayPage==1)
		{
			value1 = GetParameterValue( DP2_UPPER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else if(displayPage==2)
		{
			value1 = GetParameterValue( DP2_LOWER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else
		{
			value1 = GetParameterValue( DP2_PID_SET_VALUE );
			errorCode = ERROR_OK;
		}
		
		noSystemErrDisp = DisplayDP2(errorCode, value1);
		//---------------------------------------------------------------------
		GetPareValue(DP3_VAL_INDEX, &senVal);//GetDP3( &senVal );
		if( abs(senVal.convertedValue-(DP3_SENSOR_RESOLUTION_SM9543 / 2)) < GetParameterValue( DP3_ZERO_RNG ))
		senVal.convertedValue = (DP3_SENSOR_RESOLUTION_SM9543/2);
		//noSystemErrDisp = DisplayDP3(senVal.errorCode, senVal.convertedValue);
		
		if(!displayPage)
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		else if(displayPage==1)
		{
			value1 = GetParameterValue( DP3_UPPER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else if(displayPage==2)
		{
			value1 = GetParameterValue( DP3_LOWER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		
		noSystemErrDisp = DisplayDP3(errorCode, value1);	
		//---------------------------------------------------------------------
		GetPareValue(HUMIDITY_VAL_INDEX, &senVal);//GetHumidity( &senVal );
		
		//noSystemErrDisp = DisplayHumidity(senVal.errorCode, senVal.convertedValue);
		PreRHvalue = senVal.convertedValue;
		
		if(!displayPage)
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		else if(displayPage==1)
		{
			value1 = GetParameterValue( RH_UPPER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else if(displayPage==2)
		{
			value1 = GetParameterValue( RH_LOWER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else
		{
			value1 = GetParameterValue( RH_PID_SET_VALUE );
			errorCode = ERROR_OK;
		}
		
		noSystemErrDisp = DisplayHumidity(errorCode, value1);
		//---------------------------------------------------------------------
		GetPareValue(TEMPERATURE_VAL_INDEX, &senVal);//GetTemperature( &senVal );
		//noSystemErrDisp = DisplayTemperature(senVal.errorCode, senVal.convertedValue);
		if(senVal.errorCode == ERROR_OK) PreTempvalue = senVal.convertedValue;
		
		if(!displayPage)
		{
			value1 = senVal.convertedValue;
			errorCode = senVal.errorCode;
		}
		else if(displayPage==1)
		{
			value1 = GetParameterValue( TEMP_UPPER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else if(displayPage==2)
		{
			value1 = GetParameterValue( TEMP_LOWER_ALARM_ON_LIMIT );
			errorCode = ERROR_OK;
		}
		else
		{
			value1 = GetParameterValue( TEMP_PID_SET_VALUE );
			errorCode = ERROR_OK;
		}
		
		noSystemErrDisp = DisplayTemperature(errorCode, value1);
		//---------------------------------------------------------------------
		if(IsTemperature2Enabled())
		{	
			ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, BLACK);
			
			if(senVal.errorCode != ERROR_OK)
			{
				PrintLineInBox(LEFT,80,166,144,"PrTd:Err");
				PreTdErr=1;
			}
			else
			{
				PreTdErr=0;
				PreTd=findDewPoint(PreTempvalue,PreRHvalue);
				FormatFloat(PreTd);
				locked_sprintf_P(displayStr, PSTR("PrTd:%s*C"), formatedValue);
				PrintLineInBox(LEFT,80,166,144,displayStr);
			}
			
			GetPareValue(TEMPERATURE2_VAL_INDEX, &senVal2);
			
			if(senVal2.errorCode != ERROR_OK)
			{
				PrintLineInBox(LEFT,4,80,144,"PoT:Err");
				PrintLineInBox(LEFT,4,80,152,"PoR:Err");
				PrintLineInBox(LEFT,80,166,152,"PoTd:Err");
				ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);	
				locked_sprintf_P(displayStr, PSTR("TD:Err"));
				PostTdErr=1;
				TDdiffErr=1;
			}
			else
			{
				PostTempvalue = senVal2.convertedValue;
				
				FormatFloat(PostTempvalue);
				locked_sprintf_P(displayStr, PSTR("PoT:%s*C"), formatedValue);
				PrintLineInBox(LEFT,4,80,144,displayStr);

				GetPareValue(HUMIDITY2_VAL_INDEX, &senVal2);
				PostRHvalue = senVal2.convertedValue;
				FormatFloat(PostRHvalue);
				locked_sprintf_P(displayStr, PSTR("PoR:%s%%"), formatedValue);
				PrintLineInBox(LEFT,4,80,152,displayStr);
				
				PostTd = findDewPoint(PostTempvalue,PostRHvalue);
				PostTdErr=0;
				FormatFloat(PostTd);
				locked_sprintf_P(displayStr, PSTR("PoTd:%s*C"), formatedValue);
				PrintLineInBox(LEFT,80,166,152,displayStr);

				ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);		
				if(senVal.errorCode == ERROR_OK)
				{	
					TDdiff = PreTd-PostTd;	
					TDdiffErr=0;
					if(TDdiff < GetParameterValue(TEMP_DIFF_ALARM_LIMIT))
					{	
						toggle ^= 1;
						if(toggle)
						{
							ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, WHITE);
						}
					}
					
					FormatFloat(TDdiff);
					locked_sprintf_P(displayStr, PSTR("TD:%s"), formatedValue);
				}
				else
				{
					locked_sprintf_P(displayStr, PSTR("TD:Err"));
					TDdiffErr=1;
				}
			}
			PrintLineInBox(LEFT,164,GLCD_PIXEL_X-1,144,displayStr);
		}
		//---------------------------------------------------------------------
		if(!noSystemErrDisp && (IsRTCEnabled() || (systemError!= ERROR_OK)))
		{
			DisplaySystemError(systemError);
		}
		//--------------------------------------------------------------------
		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, BLACK);	
		ShowSetValue();
		ShowIOValue();		
	}
}

int findDewPoint(int Tempvalue, int RHvalue)
{
	double n=0, d=0;
	double temp=(double)Tempvalue/100;
	double rh=(double)RHvalue/100;
	
	n = (log(rh / 100) + ((17.27 * temp) / (237.3 + temp))) / 17.27;
	d = (237.3 * n) / (1 - n);
	
	return (d*100);
}



static void DisplayLog(char keysPressed, char keys)
{
   LCDControl = SYSTEM_LOG;
   if (IsLCDDisplay())
   {
////
   }
}

// align = 0; left align
// align = 1; center align
static void PrintLine( uint8_t align, char * str )
{
    uint8_t len = ks0xxx_StringWidth(str);
    uint8_t scrWidth = (GLCD_PIXEL_X-1) - systemXLoc, start, spaceWidth = ks0xxx_CharWidth( ' ' ), i;
    if( align == 0 )
       start = 0;
    else
       start = ( scrWidth - len ) / 2;
    for( i = spaceWidth; i < start; i += spaceWidth )
       ks0xxx_PutChar( ' ' );
    ks0xxx_Puts(str);
    for( i = i+len; len < scrWidth; len += spaceWidth )
       ks0xxx_PutChar( ' ' );
}

static void PrintLineCentered( uint8_t yPos, char * str )
{
    uint8_t len;
    uint8_t start;
    uint8_t height = ks0xxx_FontHeight();

    while((len = ks0xxx_StringWidth(str)) > GLCD_PIXEL_X)
       str[strlen(str)-1] = 0;   // remove out of screen charaters

    start = (GLCD_PIXEL_X/2) - len/2;
    ks0xxx_FillRect(0, yPos, start, height, WHITE);
    uc1638_GotoXY(start,yPos);
    ks0xxx_Puts(str);
    ks0xxx_FillRect(start+len, yPos, GLCD_PIXEL_X-start-len, height, WHITE);
}

//static void PrintLineCenteredXY(uint8_t xPos, uint8_t yPos, char * str )
//{
	//uint8_t len;
	//uint8_t start;
	//uint8_t height = ks0xxx_FontHeight();
//
	//while((len = ks0xxx_StringWidth(str)) > (GLCD_PIXEL_X-xPos))
	//str[strlen(str)-1] = 0;   // remove out of screen charaters
//
	//start = xPos + (((GLCD_PIXEL_X-xPos)/2) - len/2);
	//ks0xxx_FillRect(xPos, yPos, start, height, WHITE);
	//uc1638_GotoXY(start,yPos);
	//ks0xxx_Puts(str);
	//ks0xxx_FillRect(start+len, yPos, GLCD_PIXEL_X-start-len, height, WHITE);
//}

static void PrintLineInBox(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str )
{
	uint8_t len = ks0xxx_StringWidth(str);
	uint8_t scrWidth = (xPosmax - xPosmin)-ks0xxx_CharWidth(' ');
	uint8_t spaceWidth = ks0xxx_CharWidth(' ');
	uint8_t start=0, i=0;
	
	if( side == LEFT )
	{
		start = xPosmin;
	}
	else
	{
		start = xPosmin + (( scrWidth - len ) / 2);
	}
	
	uc1638_GotoXY(start,yPos);
	for(i = (xPosmin+spaceWidth); i < start; i += spaceWidth)
	{
		ks0xxx_PutChar(' ');
	}
	
	ks0xxx_Puts(str);
	
	for(i = i+len; len < scrWidth; len += spaceWidth)
	{
		ks0xxx_PutChar(' ');
	}
	
	//-----------------------------------------------------------------------
	//uint8_t len = ks0xxx_StringWidth(str);
	//uint8_t scrWidth = (xPosmax - xPosmin);//-ks0xxx_CharWidth(' ');
	//uint8_t charWidth = ks0xxx_CharWidth(' ');
	//uint8_t start=0, i=0;
	//
	//if( side == LEFT )
	//{
		//start = xPosmin;
	//}
	//else
	//{
		//start = xPosmin + (( scrWidth - len ) / 2);
	//}
	//
	//uc1638_GotoXY(xPosmin,yPos);
	////for(i = (xPosmin+spaceWidth); i < start; i += spaceWidth)
	//for(i = (xPosmin); i <= start; i += charWidth)
	//{
		//ks0xxx_PutChar(' ');
	//}
	//uc1638_GotoXY(start,yPos);
	//ks0xxx_Puts(str);
	//
	//for(i = i+len; len < (scrWidth-len); len += charWidth)
	//{
		//ks0xxx_PutChar(' ');
	//}

	//uint8_t len = ks0xxx_StringWidth(str);
	//uint8_t start,width;
	//uint8_t height = ks0xxx_FontHeight();
//
	//while(len > (xPosmax-xPosmin))
	//{
		//str[strlen(str)-1] = 0;   // remove out of screen characters
	//}
	//
	//width = ((xPosmax-xPosmin)/2) - len/2;
	//start = width + xPosmin + 1;
	//
	//if(side == CENTER)
	//{
		//ks0xxx_FillRect(xPosmin, yPos, width, height, WHITE);
		//uc1638_GotoXY(start,yPos);
		//ks0xxx_Puts(str);
		//ks0xxx_FillRect(xPosmin+width+len, yPos, width, height, WHITE);
	//}
	//else
	//{
		//ks0xxx_FillRect(xPosmin, yPos, xPosmax-xPosmin, height, WHITE);
		//uc1638_GotoXY(xPosmin,yPos);
		//ks0xxx_Puts(str);
	//}
}

static void PrintLineInBoxWOClear(uint8_t side, uint8_t xPosmin, uint8_t xPosmax, uint8_t yPos, char * str )
{
	uint16_t len = ks0xxx_StringWidth(str);
	uint8_t start,width;

	//while(len > (xPosmax-xPosmin))
	if(len > (xPosmax-xPosmin))
	{
		str[strlen(str)-1] = 0;   // remove out of screen characters
	}
	
	width = ((xPosmax-xPosmin)/2) - len/2;
	start = width + xPosmin + 1;
	
	if(side == CENTER)
	{
		uc1638_GotoXY(start,yPos);
		ks0xxx_Puts(str);
	}
	else
	{
		uc1638_GotoXY(xPosmin,yPos);
		ks0xxx_Puts(str);
	}
}

void DisplaySystemError(void)
{
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);
		
		FormatSystemError( displayStr, systemError);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,84,160,120,displayStr);
		
		OSSemaGive(UIMutex);
	}
}

char DisplayDP1(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);
	  
		FormatDP(PRES_SENS_SM9543, GetParameterValue( DP1_UNIT ), value, DP1_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		  
		if(!displayPage)
		{
			locked_sprintf_P( displayStr, PSTR("D1(%S)"), formatedUnit);
		}
		else if(displayPage==1)
		{
			locked_sprintf_P( displayStr, PSTR("D1(%S)U"), formatedUnit);
		}
		else if(displayPage==2)
		{
			locked_sprintf_P( displayStr, PSTR("D1(%S)L"), formatedUnit);
		}
		else
		{
			locked_sprintf_P( displayStr, PSTR("D1(%S)P"), formatedUnit);
		}
		//locked_sprintf_P( displayStr, PSTR("DP1(%S)"), formatedUnit);
		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
		PrintLineInBox(LEFT,2,80,24,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,2,80,40,displayStr);

		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}

char DisplayDP2(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);

		FormatDP(PRES_SENS_SM9543, GetParameterValue( DP2_UNIT ), value, DP2_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP2_RANGE), GetParameterValue(DP2_AREA_TYPE), GetParameterValue(DP2_AREA_LENGTH), GetParameterValue(DP2_AREA_WIDTH), GetParameterValue(DP2_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		if(!displayPage)
		{
			locked_sprintf_P( displayStr, PSTR("D2(%S)"), formatedUnit);
		}
		else if(displayPage==1)
		{
			locked_sprintf_P( displayStr, PSTR("D2(%S)U"), formatedUnit);
		}
		else if(displayPage==2)
		{
			locked_sprintf_P( displayStr, PSTR("D2(%S)L"), formatedUnit);
		}
		else
		{
			locked_sprintf_P( displayStr, PSTR("D2(%S)P"), formatedUnit);
		}
		//locked_sprintf_P( displayStr, PSTR("DP2(%S)"), formatedUnit);
		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
		PrintLineInBox(LEFT,2,80,64,displayStr);

		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,2,80,80,displayStr);

		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}

char DisplayDP3(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);
	  
		FormatDP(PRES_SENS_SM9543, GetParameterValue( DP3_UNIT ), value, DP3_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP3_RANGE), GetParameterValue(DP3_AREA_TYPE), GetParameterValue(DP3_AREA_LENGTH), GetParameterValue(DP3_AREA_WIDTH), GetParameterValue(DP3_AREA_RADIOUS));
		formatedValue[strlen(formatedValue)-1] = '\0';  // one digit less resolution then setup
		if(formatedValue[strlen(formatedValue)-1] == '.' )
		formatedValue[strlen(formatedValue)-1] = '\0';  // if last digit '.' remove it
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		if(!displayPage)
		{
			locked_sprintf_P( displayStr, PSTR("D3(%S)"), formatedUnit);
		}
		else if(displayPage==1)
		{
			locked_sprintf_P( displayStr, PSTR("D3(%S)U"), formatedUnit);
		}
		else if(displayPage==2)
		{
			locked_sprintf_P( displayStr, PSTR("D3(%S)L"), formatedUnit);
		}
		else
		{
			locked_sprintf_P( displayStr, PSTR("D3(%S)"), formatedUnit);
		}
		//locked_sprintf_P( displayStr, PSTR("DP3(%S)"), formatedUnit);
		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
		PrintLineInBox(LEFT,2,80,104,displayStr);
		
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,2,80,120,displayStr);

		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}

char DisplayTemperature(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	//uint8_t Temp = IsTemperature2Enabled();
	
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);

		FormatTemperature( value );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		if(!displayPage)
		{
			locked_sprintf_P( displayStr, PSTR("PrTMP(%S)"), formatedUnit);
		}
		else if(displayPage==1)
		{
			locked_sprintf_P( displayStr, PSTR("PrTMP(%S)U"), formatedUnit);
		}
		else if(displayPage==2)
		{
			locked_sprintf_P( displayStr, PSTR("PrTMP(%S)L"), formatedUnit);
		}
		else
		{
			locked_sprintf_P( displayStr, PSTR("PrTMP(%S)P"), formatedUnit);
		}

		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
		PrintLineInBox(LEFT,84,160,24,displayStr);
      
		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,84,160,40,displayStr);
	  
		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}

char DisplayTemperature2(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);

		FormatTemperature( value );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		locked_sprintf_P( displayStr, PSTR("PTEMP(%S)=%s"), formatedUnit,formatedValue);
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,4,GLCD_PIXEL_X-1,144,displayStr);
		
		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}

char DisplayHumidity(uint8_t error, int16_t value)
{
	char noSystemErrDisp = 0;
	//uint8_t Temp = IsTemperature2Enabled();
	
	if (IsLCDDisplay())
	{
		OSSemaTakeEver(UIMutex);
		
		FormatHumidity( value );
		if( error != ERROR_OK )
		{
			FormatError( error );
		}
		
		//if(!displayPage)
		//{
			//locked_sprintf_P( displayStr, Temp? PSTR("PrRH(%S)"): PSTR("RH(%S)"), formatedUnit);
		//}
		//else if(displayPage==1)
		//{
			//locked_sprintf_P( displayStr, Temp? PSTR("PrRH(%S)U"): PSTR("RH(%S)U"), formatedUnit);
		//}
		//else if(displayPage==2)
		//{
			//locked_sprintf_P( displayStr, Temp? PSTR("PrRH(%S)L"): PSTR("RH(%S)L"), formatedUnit);
		//}
		//else
		//{
			//locked_sprintf_P( displayStr, Temp? PSTR("PrRH(%S)P"): PSTR("RH(%S)P"), formatedUnit);
		//}

		//if(IsTemperature2Enabled())
		//{
			//if(!displayPage)
			//{
				//locked_sprintf_P( displayStr, PSTR("RH(%S)"), formatedUnit);
			//}
			//else if(displayPage==1)
			//{
				//locked_sprintf_P( displayStr, PSTR("RH(%S)U"), formatedUnit);
			//}
			//else if(displayPage==2)
			//{
				//locked_sprintf_P( displayStr, PSTR("RH(%S)L"), formatedUnit);
			//}
			//else
			//{
				//locked_sprintf_P( displayStr, PSTR("RH(%S)P"), formatedUnit);
			//}
		//}
		//else
		//{
			if(!displayPage)
			{
				locked_sprintf_P( displayStr, PSTR("PrRH(%S)"), formatedUnit);
			}
			else if(displayPage==1)
			{
				locked_sprintf_P( displayStr, PSTR("PrRH(%S)U"), formatedUnit);
			}
			else if(displayPage==2)
			{
				locked_sprintf_P( displayStr, PSTR("PrRH(%S)L"), formatedUnit);
			}
			else
			{
				locked_sprintf_P( displayStr, PSTR("PrRH(%S)P"), formatedUnit);
			}
		//}

		ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
		PrintLineInBox(LEFT,84,160,64,displayStr);

		locked_sprintf_P( displayStr, PSTR("%s"), formatedValue );
		ks0xxx_SelectFont(ARIAL18BOLD, ks0xxx_ReadFontData, BLACK);
		PrintLineInBox(LEFT,84,160,80,displayStr);
		
		OSSemaGive(UIMutex);
	}
	return noSystemErrDisp;
}


int16_t DP1sensorResolution(void)
{
	return DP1_SENSOR_RESOLUTION_SM9543;
	
	//if(GetParameterValue(DP1_SENS_TYPE)==PRES_SENS_SM9543)
	//{
		//return DP1_SENSOR_RESOLUTION_SM9543;
	//}
	//else
	//{
		//return DP1_SENSOR_RESOLUTION_OTHER;
	//}
	
	//int16_t dpresolution;
	//
	//switch( GetParameterValue(DP1_SENS_TYPE))
	//{
		//case PRES_SENS_SM9543:    dpresolution = DP1_SENSOR_RESOLUTION_SM9543;		break;
		//case PRES_SENS_AMS5812:
		//case PRES_SENS_SM4331:
		//case PRES_SENS_NONE:
		//default:                  dpresolution = DP1_SENSOR_RESOLUTION;		break;
	//}
	//
	//return dpresolution;
}

int16_t DP2sensorResolution(void)
{
	return DP2_SENSOR_RESOLUTION_SM9543;
	
	//if(GetParameterValue(DP2_SENS_TYPE)==PRES_SENS_SM9543)
	//{
		//return DP2_SENSOR_RESOLUTION_SM9543;
	//}
	//else
	//{
		//return DP2_SENSOR_RESOLUTION_OTHER;
	//}
	
	//int16_t dpresolution;
	//
	//switch( GetParameterValue(DP2_SENS_TYPE))
	//{
		//case PRES_SENS_SM9543:    dpresolution = DP2_SENSOR_RESOLUTION_SM9543;		break;
		//case PRES_SENS_AMS5812:
		//case PRES_SENS_SM4331:
		//case PRES_SENS_NONE:
		//default:                  dpresolution = DP2_SENSOR_RESOLUTION;		break;
	//}
	//
	//return dpresolution;
}

int16_t DP3sensorResolution(void)
{
	return DP3_SENSOR_RESOLUTION_SM9543;
	
	//if(GetParameterValue(DP3_SENS_TYPE)==PRES_SENS_SM9543)
	//{
		//return DP3_SENSOR_RESOLUTION_SM9543;
	//}
	//else
	//{
		//return DP3_SENSOR_RESOLUTION;
	//}
	//int16_t dpresolution;
	 //
	//switch( GetParameterValue(DP3_SENS_TYPE))
	//{
		//case PRES_SENS_SM9543:    dpresolution = DP3_SENSOR_RESOLUTION_SM9543;		break;
		//case PRES_SENS_AMS5812:
		//case PRES_SENS_SM4331:
		//case PRES_SENS_NONE:
		//default:                  dpresolution = DP3_SENSOR_RESOLUTION;		break;
	//}
	//
	//return dpresolution;
}

//#pragma O0
/* Converts and formats the sting for output */
static void FormatDP(uint8_t senType, uint8_t unit, int16_t value, int16_t dpsenResolution, int16_t range1, uint8_t AreaType, int16_t length, int16_t width, int16_t radious)
{
	double result,range,area=1;
	uint8_t decimal;  // minimum value is 1
	
	range = range1;
	range /= 6894.7;	//(GetParameterValue(DP1_RANGE)/6895.7)  // Pa to PSI conversion
	
	if(senType==PRES_SENS_SM9543) value = value-(dpsenResolution / 2);
	
	if( unit == (uint8_t)PRESSURE_CFM )
	{
		if(AreaType==PRES_AREA_SQUARE)
		{
			//area = (length * width)/1000000;
			area = length;
			area *= width;
			area /= 92903.04;
		}
		else if(AreaType==PRES_AREA_ROUND)
		{
			//area = (3.14159265359 * radious * radious)/1000000;
			area = radious;
			area *= radious;
			area *= 3.14159265359;
			area /= 92903.04;
		}
		else
		{
			area = 1.0;
		}
	}
	
	if (IsLCDDisplay())
	{
		if( unit == (uint8_t)PRESSURE_MILIBAR )
		{
			result = value * (range*68.9476) / dpsenResolution;
			formatedUnit = PSTR("mBAR");
			decimal = 4;
		}
		else if( unit == (uint8_t)PRESSURE_PA )
		{
			result = value * (range*6894.76) / dpsenResolution;
			formatedUnit = PSTR("Pa");
			if( result > -0.1 && result < 0 )
			result = 0;
			decimal = 2;
		}
		else if( unit == (uint8_t)PRESSURE_HG_INCH )
		{
			result = value * (range*2.03602) / dpsenResolution;
			formatedUnit = PSTR("iHg");
			decimal = 5;
		}
		else if( unit == (uint8_t)PRESSURE_PSF )
		{
			result = value * (range*144.0) / dpsenResolution;
			formatedUnit = PSTR("PSF");
			decimal = 3;
		}
		else if( unit == (uint8_t)PRESSURE_H2O_MM )
		{
			result = value * (range*703.069578295612) / dpsenResolution;
			formatedUnit = PSTR("mmH2O");
			decimal = 3;
		}
		else if( unit == (uint8_t)PRESSURE_H2O_INCH )
		{
			result = value * (range*27.7076) / dpsenResolution;
			formatedUnit = PSTR("iH2O");
			decimal = 4;
		}
		else if( unit == (uint8_t)PRESSURE_CFM )
		{
			result = (4005.0 * sqrt(value * (range*27.7076) / dpsenResolution) * area);///100000;
			formatedUnit = PSTR("CFM");
			decimal = 1;//1;
		}
		else
		{
			result = value * range / dpsenResolution;
			formatedUnit = PSTR("PSI");
			decimal = 5;
		}

		dtostrf( result, sizeof(formatedValue)-1, decimal, formatedValue);
		strcpy( formatedValue, strtrim(formatedValue));
	}
	return;
}

int16_t ConvertToPascal(uint8_t senType, int16_t value, int16_t dpsenResolution, int16_t range1)
{
	double result,range;
	
	range = range1;
	range /= 6895.7;	//(GetParameterValue(DP1_RANGE)/6895.7)  // Pa to PSI conversion
	
	if(senType==PRES_SENS_SM9543) value = value-(dpsenResolution / 2);

	result = value * (range*6894.76) / dpsenResolution;
	if( result > -0.1 && result < 0 )
	result = 0;
	
	return 	(int16_t) (result*100);
}

//#pragma Os

/* Converts and formats the sting for output */
static void FormatTemperature(int16_t value)
{
   double result;
   tempUnit = GetParameterValue( TEMPERATURE_UNIT );

   if (IsLCDDisplay())
   {
      if( tempUnit == TEMPERATURE_DEG_F )
      {
         formatedUnit = PSTR("*F");
         result = ((double)value * 9 / 500) + 32;
      }
      else if( tempUnit == TEMPERATURE_DEG_K )
      {
         formatedUnit = PSTR("*K");
         result = value / 100.0 + 273.15;
      }
      else
      {
         formatedUnit = PSTR("*C");
         result = value / 100.0;
      }

      dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
   }
   return;
}

/* Converts and formats the sting for output */
static void FormatHumidity(int16_t value)
{
   double result;

   humidityUnit = GetParameterValue( HUMIDITY_UNIT );

   if (IsLCDDisplay())
   {
      formatedUnit = PSTR("%");
      result = value / 100.0;
      dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
   }
   return;
}

/* Converts and formats the sting for output */
static void FormatFloat(int16_t value)
{
	double result;

	result = value / 100.0;
	dtostrf( result, sizeof(formatedValue)-1, 1, formatedValue);
	strcpy( formatedValue, strtrim(formatedValue));

	return;
}

//static void FormatPartical( uint16_t value)
//{
	//double result;
//
	//if (IsLCDDisplay())
	//{
		////formatedUnit = PSTR("um");
		//result = value / 10.0;
		//dtostrf( result, sizeof(formatedValue)-1, 1, formatedValue);
		//strcpy( formatedValue, strtrim(formatedValue));
	//}
	//return;
//}

//static void FormatControlOutput( char * str, int value)
//{
   //locked_sprintf_P( str, PSTR("Output:%d.%d%%"), (int)(value/10), (int)(value%10));
//}

//-------------------------------
// Convert BCD 1 byte to HEX 1 byte
//-------------------------------
uint8_t BCD2HEX(uint8_t bcd)
{
	uint8_t temp=0;
	temp=((bcd>>4)*10);
	temp+=(bcd & 0x0F);
	
	return temp;
}

//-------------------------------
// Convert HEX 1 byte to BCD 1 byte
//-------------------------------
uint8_t HEX2BCD(uint8_t hex)
{
	uint8_t temp=0;
	temp = ((hex / 10)<<4);
	temp |= (hex % 10);
	return temp;
}

static TIME curTime; 
static char dispValSel = 0;
/* formats the sting for output */
char * FormatSystemError( char * str, int16_t value )
{
	static uint8_t lastsec=0, toggle=0, curhr=0, curhr1=0;
	
	if (IsLCDDisplay())
	{		
		if(dispValSel == (char)0) 
		{
			if(IsRTCEnabled())
			{
				if(RTCGetTime( &curTime ) == ERROR_OK)
				{
					//displayStr[0] = (curTime.hours >> 4) + '0';
					//displayStr[1] = (curTime.hours & 0xf ) + '0';
					//displayStr[2] = ' ';
					//displayStr[3] = (curTime.minutes >> 4) + '0';
					//displayStr[4] = (curTime.minutes & 0xf ) + '0';
					//displayStr[5] = ' ';
					//displayStr[6] = (curTime.seconds >> 4) + '0';
					//displayStr[7] = (curTime.seconds & 0xf ) + '0';
					//displayStr[8] = 0;
					
					
					if(lastsec != curTime.seconds)
					{
						lastsec = curTime.seconds;
						toggle ^= 1;
					}
					
					curhr = BCD2HEX(curTime.hours);
					if(curhr>=12)
					{
						if(curhr>12) curhr -=12;
						displayStr[6] = 'p';
						displayStr[7] = 'm';
					}
					else
					{
						displayStr[6] = 'a';
						displayStr[7] = 'm';
					}
					curhr1 = HEX2BCD(curhr);
					
					displayStr[0] = (curhr1 >> 4) + '0';
					displayStr[1] = (curhr1 & 0xf ) + '0';
					displayStr[2] = (toggle==0)?' ':':';
					displayStr[3] = (curTime.minutes >> 4) + '0';
					displayStr[4] = (curTime.minutes & 0xf ) + '0';
					displayStr[5] = ' ';
					
					displayStr[8] = 0;
				}
				else
				{
					locked_sprintf_P( str, PSTR("RTC ERR"));
				}
				
				ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
				PrintLineInBoxWOClear(LEFT,84,160,104,"CLOCK");
			}
			else
			{
				dispValSel++;
			}
		}
		
		if((dispValSel == (char)1) && ( value != ERROR_OK ))
		{
			ks0xxx_SelectFont(Font6x8, ks0xxx_ReadFontData, WHITE);
			PrintLineInBoxWOClear(LEFT,84,160,104,"SYS ERR");
			locked_sprintf_P( str, PSTR("%d"), value );
		}
		else
		{
			dispValSel++;
		}
		
		dispValSel = (dispValSel + 1) %2;
	}

	return str;
	
	   
   //if (IsLCDDisplay())
   //{
      //if( value == ERROR_OK )
	  //{
			//dispValSel = (dispValSel + 1) %2;
	  //}
      //else
	  //{
         //dispValSel = (dispValSel + 1) %3;
	  //}
//
      //if( dispValSel == (char)0)
      //{
         //if( RTCGetTime( &curTime ) == ERROR_OK )
         //{
            //displayStr[0] = ' ';
            //displayStr[1] = (curTime.hours >> 4) + '0';
            //displayStr[2] = (curTime.hours & 0xf ) + '0';
            //displayStr[3] = ':';
            //displayStr[4] = (curTime.minutes >> 4) + '0';
            //displayStr[5] = (curTime.minutes & 0xf ) + '0';
            //displayStr[6] = ':';
            //displayStr[7] = (curTime.seconds >> 4) + '0';
            //displayStr[8] = (curTime.seconds & 0xf ) + '0';
            //displayStr[9] = 0;
         //}
         //else
		 //{
            //dispValSel++;
		 //}
      //}
      //else if( dispValSel == (char)1)
      //{
	      //locked_sprintf_P( str, PSTR("www.sapindia.in"));
      //}
      //else if( dispValSel == (char)2)
	  //{
         //locked_sprintf_P( str, PSTR("SYSTEM ERR:%d"), value );
	  //}
   //}
   //return str;
}

///* formats the sting for output */
//static void FormatDispError( int8_t error )
//{
   //if (IsLCDDisplay())
      //locked_sprintf_P( formatedValue, PSTR("ER.%d"), error );
   //return;
//}

/* formats the sting for output */
static void FormatError( int8_t error )
{
   if (IsLCDDisplay())
      locked_sprintf_P( formatedValue, PSTR("E%d"), error );
   return;
}

/* formats the sting for output */
static char * FormatInteger( int16_t value)
{
   if (IsLCDDisplay())
   {
      locked_sprintf_P( displayStr, PSTR("%d"), value );
      return displayStr;
   }
   return NULL;
}

static char * FormatVersion()
{
    int major = SOFTWARE_VERSION/100;
    int minor = SOFTWARE_VERSION%100;
    if( minor < 10 )
       locked_sprintf_P( formatedValue, PSTR("%d.0%d"), major, minor );
    else
       locked_sprintf_P( formatedValue, PSTR("%d.%d"), major, minor );

    return formatedValue;
}


char * FormatDP1OneLine( char * str, uint8_t error, int16_t value )
{
   if( error != ERROR_OK )
   {
      FormatError( error );
   }
   else
   {
	   //FormatDP(GetParameterValue( DP1_SENS_TYPE ), GetParameterValue( DP1_UNIT ), value,  DP1sensorResolution(), GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
	   FormatDP(PRES_SENS_SM9543, GetParameterValue( DP1_UNIT ), value, DP1_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP1_RANGE), GetParameterValue(DP1_AREA_TYPE), GetParameterValue(DP1_AREA_LENGTH), GetParameterValue(DP1_AREA_WIDTH), GetParameterValue(DP1_AREA_RADIOUS));
   }
   locked_sprintf_P( str, PSTR("DP1(%S): %s"), formatedUnit, formatedValue );
   return str;
}

char * FormatDP2OneLine( char * str, uint8_t error, int16_t value )
{
	if( error != ERROR_OK )
	{
		FormatError( error );
	}
	else
	{
		//FormatDP(GetParameterValue( DP2_SENS_TYPE ), GetParameterValue( DP2_UNIT ), value,  DP2sensorResolution(), GetParameterValue(DP2_RANGE), GetParameterValue(DP2_AREA_TYPE), GetParameterValue(DP2_AREA_LENGTH), GetParameterValue(DP2_AREA_WIDTH), GetParameterValue(DP2_AREA_RADIOUS));
		FormatDP(PRES_SENS_SM9543, GetParameterValue( DP2_UNIT ), value, DP2_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP2_RANGE), GetParameterValue(DP2_AREA_TYPE), GetParameterValue(DP2_AREA_LENGTH), GetParameterValue(DP2_AREA_WIDTH), GetParameterValue(DP2_AREA_RADIOUS));
	}

	locked_sprintf_P( str, PSTR("DP2(%S): %s"), formatedUnit, formatedValue );
	return str;
}

char * FormatDP3OneLine( char * str, uint8_t error, int16_t value )
{
	if( error != ERROR_OK )
	{
		FormatError( error );
	}
	else
	{
		//FormatDP(GetParameterValue( DP3_SENS_TYPE ), GetParameterValue( DP3_UNIT ), value,  DP3sensorResolution(), GetParameterValue(DP3_RANGE), GetParameterValue(DP3_AREA_TYPE), GetParameterValue(DP3_AREA_LENGTH), GetParameterValue(DP3_AREA_WIDTH), GetParameterValue(DP3_AREA_RADIOUS));
		FormatDP(PRES_SENS_SM9543, GetParameterValue( DP3_UNIT ), value, DP3_SENSOR_RESOLUTION_SM9543, GetParameterValue(DP3_RANGE), GetParameterValue(DP3_AREA_TYPE), GetParameterValue(DP3_AREA_LENGTH), GetParameterValue(DP3_AREA_WIDTH), GetParameterValue(DP3_AREA_RADIOUS));
	}

	locked_sprintf_P( str, PSTR("DP3(%S): %s"), formatedUnit, formatedValue );
	return str;
}

char * FormatTemperatureOneLine( char * str, uint8_t error, int16_t value )
{
   FormatTemperature( value );
   if( error != ERROR_OK )
      FormatError( error );
   locked_sprintf_P( str, PSTR("TEMP(%S): %s"), formatedUnit, formatedValue );
   return str;
}

char * FormatHumidityOneLine( char * str, uint8_t error, int16_t value )
{
   FormatHumidity( value );
   if( error != ERROR_OK )
      FormatError( error );
   locked_sprintf_P( str, PSTR("HUMI(%S): %s"), formatedUnit, formatedValue );
   return str;
}

//char * FormatPartialStuffOneLine( char * str, uint8_t error, uint16_t value )
//{
	//if(error != ERROR_OK)
	//{
		//FormatError( error );
	//}
	//else
	//{
		//FormatPartical( value );
	//}
	//
	//locked_sprintf_P( str, PSTR("%s ppb"), formatedValue );
	//return str;
//}
//
//char * FormatParticalOneLine( char * str, uint8_t error, uint16_t value )
//{
	//if(error != ERROR_OK)
	//{
		//FormatError( error );
	//}
	//else
	//{
		//FormatPartical( value );
	//}
	//
	//switch(GetParameterValue( PARTICAL_SIZE_TYPE ))
	//{
		//case PARTICAL_SIZE_1_0:		formatedtype=PSTR("1.0");	 break;
		//case PARTICAL_SIZE_2_5:		formatedtype=PSTR("2.5");	 break;
		//case PARTICAL_SIZE_4_0:		formatedtype=PSTR("4.0");	 break;
		//case PARTICAL_SIZE_10_0:	formatedtype=PSTR("10.0");	 break;
	//}
	//
	//locked_sprintf_P( str, PSTR("PC%S(%S): %s"), formatedtype, PSTR("ug/m3"), formatedValue );
	//
	//return str;
//}

const PARA_INFO paraTableLCD [] PROGMEM =
{
	{ DEVICE_ID,                        IsSerialEnabled,    NULL,                   "Device ID",          DeviceIdPara  },
	//{ DISPLAY_INTERVAL,                 NULL,               NULL,                   "Display Interval",   DisplayIntervalPara },
	//{ DISPLAY_MODE,                     NULL,               NULL,                   "Display Mode",       DisplayModePara },	
	{ TEMP_FIRE_ALM_SET,                IsTemperatureEnabled,       NULL,           "FireAlmSet",   TempFireAlmSetPara  },
	{ TEMP_FIRE_ALM_TIME,                 IsTemperatureEnabled,       NULL,           "FireTimeSet(Sec)",  TempFireAlmTimePara  },
		
	{ DP1_UNIT,					   IsDP1Enabled,  NULL,           "DP1 Unit",           PressureAbsUnitPara  },
	{ DP1_AREA_TYPE,               IsMasterReset, IsDP1CFMEnabled,				"DP1 AREA TYPE",     DP1AreaTypePara  },
	{ DP1_AREA_LENGTH,             IsMasterReset, IsDP1CFMAreaSquare,				"DP1 AREA LENGTH",   DP1AreaLengthPara  },
	{ DP1_AREA_WIDTH,              IsMasterReset, IsDP1CFMAreaSquare,				"DP1 AREA WIDTH",    DP1AreaWidthPara  },
	{ DP1_AREA_RADIOUS,            IsMasterReset, IsDP1CFMAreaRound,				"DP1 AREA RAD",  DP1AreaRadiousPara  },
	//{ DP1_AREA,                    IsMasterReset, IsDP1CFMEnabled,				"DP1 AREA SQFT",     PresAbsAreaPara  },
	{ DP1_ZERO_RNG,                IsDP1Enabled,  NULL,								"DP1 ZeroRngCnt",   AbsPresZeroRangePara  },
	{ DP1_ZERO_ADJ,                IsDP1Enabled,  NULL,								"DP1 ZeroAdjCnt",   AbsPresZeroAdjustPara  },
	{ DP1_ALARM_SETUP,             IsAlarmEnabled,         IsDP1Enabled, "DP1 Alarm Setup",      PresAbsAlarmSetupPara },
	{ DP1_UPPER_ALARM_ON_LIMIT,    IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Upr Alarm On",     PresAbsUpperAlarmOnPara },
	{ DP1_UPPER_ALARM_OFF_LIMIT,   IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Upr Alarm Off",    PresAbsUpperAlarmOffPara  },
	{ DP1_LOWER_ALARM_ON_LIMIT,    IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Lwr Alarm On",     PresAbsLowerAlarmOnPara },
	{ DP1_LOWER_ALARM_OFF_LIMIT,   IsPresAbsAlarmEnabled,  IsDP1Enabled, "DP1 Lwr Alarm Off",    PresAbsLowerAlarmOffPara  },
	{ DP1_OUTPUT_TYPE,             IsAlalogOutEnabled,  IsPIDEnabled,          "DP1 Out Type",       PresAbsOutputTypePara  },
	{ DP1_OUT_LOW_PRES,            IsPresAbsOutEnabled, IsAlalogOutEnabled,    "DP1 MinOutput",     PresAbsValForMinimumOutputPara  },
	{ DP1_OUT_HIGH_PRES,           IsPresAbsOutEnabled, IsAlalogOutEnabled,    "DP1 MaxOutput",     PresAbsValForMaximumOutputPara  },
	{ DP1_PID_STARTUP_PERCENT,     IsPresAbsPIDEnabled, IsAlalogOutEnabled,    "DP1 PID Start %",    PresAbsPIDStartupPercentPara  },
	{ DP1_PID_KP,                  IsPresAbsPIDEnabled, IsAlalogOutEnabled,    "DP1 PID Kp",         PresAbsPIDKpPara  },
	{ DP1_PID_TI,                  IsPresAbsPIDEnabled, IsAlalogOutEnabled,    "DP1 PID Ti",         PresAbsPIDTiPara  },
	{ DP1_PID_TD,                  IsPresAbsPIDEnabled, IsAlalogOutEnabled,    "DP1 PID Td",         PresAbsPIDTdPara  },
	{ DP1_PID_SET_VALUE,           IsPresAbsPIDEnabled, IsAlalogOutEnabled,    "DP1 PIDSetPoint",   PresAbsPIDSetpointPara  },
	{ DP1_OUT_POLARITY,            IsPresAbsOutActive,  IsAlalogOutEnabled,    "DP1 OutPolarity",   PresAbsOutPolarityPara  },
	{ DP1_OUT_LOW_COUNT,           IsPresAbsOutActive,  IsAlalogOutEnabled,    "DP1 OutCntMin",    OutputCountsForPresAbsMinimumPara  },
	{ DP1_OUT_HIGH_COUNT,          IsPresAbsOutActive,  IsAlalogOutEnabled,    "DP1 OutCntMax",    OutputCountsForPresAbsMaximumPara  },
	
	{ DP2_UNIT,						IsDP2Enabled,      NULL,           "DP2 Unit",           PressureDiffUnitPara  },
	{ DP2_AREA_TYPE,               IsMasterReset, IsDP2CFMEnabled,				"DP2 AREA TYPE",     DP2AreaTypePara  },
	{ DP2_AREA_LENGTH,             IsMasterReset, IsDP2CFMAreaSquare,				"DP2 AREA LENGTH",   DP2AreaLengthPara  },
	{ DP2_AREA_WIDTH,              IsMasterReset, IsDP2CFMAreaSquare,				"DP2 AREA WIDTH",    DP2AreaWidthPara  },
	{ DP2_AREA_RADIOUS,            IsMasterReset, IsDP2CFMAreaRound,				"DP2 AREA RAD",  DP2AreaRadiousPara  },
	//{ DP2_AREA,                   IsMasterReset,    IsDP2CFMEnabled,			"DP2 AREA SQFT",     PresDiffAreaPara  },
	{ DP2_ZERO_RNG,               IsDP2Enabled,      NULL,					"DP2 ZeroRngCnt",   DiffPresZeroRangePara    },
	{ DP2_ZERO_ADJ,               IsDP2Enabled,      NULL,					"DP2 ZeroAdjCnt",   DiffPresZeroAdjustPara    },
	{ DP2_ALARM_SETUP,            IsAlarmEnabled,         IsDP2Enabled,     "DP2 Alarm Setup",      PresDiffAlarmSetupPara },
	{ DP2_UPPER_ALARM_ON_LIMIT,   IsPresDiffAlarmEnabled, IsDP2Enabled,     "DP2 Upr Alarm On",     PresDiffUpperAlarmOnPara  },
	{ DP2_UPPER_ALARM_OFF_LIMIT,  IsPresDiffAlarmEnabled, IsDP2Enabled,     "DP2 Upr Alarm Off",    PresDiffUpperAlarmOffPara },
	{ DP2_LOWER_ALARM_ON_LIMIT,   IsPresDiffAlarmEnabled, IsDP2Enabled,     "DP2 Lwr Alarm On",     PresDiffLowerAlarmOnPara  },
	{ DP2_LOWER_ALARM_OFF_LIMIT,  IsPresDiffAlarmEnabled, IsDP2Enabled,     "DP2 Lwr Alarm Off",    PresDiffLowerAlarmOffPara },
	{ DP2_OUTPUT_TYPE,            IsAlalogOutEnabled,   IsPIDEnabled,         "DP2 Out Type",       PresDiffOutputTypePara  },
	{ DP2_OUT_LOW_PRES,           IsPresDiffOutEnabled, IsAlalogOutEnabled,   "DP2 MinOutput",     PresDiffValForMinimumOutputPara  },
	{ DP2_OUT_HIGH_PRES,          IsPresDiffOutEnabled, IsAlalogOutEnabled,   "DP2 MaxOutput",     PresDiffValForMaximumOutputPara  },
	{ DP2_PID_STARTUP_PERCENT,          IsPresDiffPIDEnabled, IsAlalogOutEnabled,   "DP2 PID Start %",    PresDiffPIDStartupPercentPara  },
	{ DP2_PID_KP,                 IsPresDiffPIDEnabled, IsAlalogOutEnabled,   "DP2 PID Kp",         PresDiffPIDKpPara  },
	{ DP2_PID_TI,                 IsPresDiffPIDEnabled, IsAlalogOutEnabled,   "DP2 PID Ti",         PresDiffPIDTiPara  },
	{ DP2_PID_TD,                 IsPresDiffPIDEnabled, IsAlalogOutEnabled,   "DP2 PID Td",         PresDiffPIDTdPara  },
	{ DP2_PID_SET_VALUE,          IsPresDiffPIDEnabled, IsAlalogOutEnabled,   "DP2 PIDSetPoint",   PresDiffPIDSetpointPara  },
	{ DP2_OUT_POLARITY,           IsPresDiffOutActive,  IsAlalogOutEnabled,   "DP2 OutPolarity",   PresDiffOutPolarityPara  },
	{ DP2_OUT_LOW_COUNT,          IsPresDiffOutActive,  IsAlalogOutEnabled,   "DP2 OutCntMin",    OutputCountsForPresDiffMinimumPara  },
	{ DP2_OUT_HIGH_COUNT,         IsPresDiffOutActive,  IsAlalogOutEnabled,   "DP2 OutCntMax",    OutputCountsForPresDiffMaximumPara  },
	
	{ DP3_UNIT,					  IsDP3Enabled,     NULL,           "DP3 Unit",           PressureDiff3UnitPara  },
	{ DP3_AREA_TYPE,               IsMasterReset, IsDP3CFMEnabled,				"DP3 AREA TYPE",     DP3AreaTypePara  },
	{ DP3_AREA_LENGTH,             IsMasterReset, IsDP3CFMAreaSquare,				"DP3 AREA LENGTH",   DP3AreaLengthPara  },
	{ DP3_AREA_WIDTH,              IsMasterReset, IsDP3CFMAreaSquare,				"DP3 AREA WIDTH",    DP3AreaWidthPara  },
	{ DP3_AREA_RADIOUS,            IsMasterReset, IsDP3CFMAreaRound,				"DP3 AREA RAD",  DP3AreaRadiousPara  },
	//{ DP3_AREA,                   IsMasterReset,    IsDP3CFMEnabled,			"DP3 AREA SQFT",     PresDiff3AreaPara  },
	{ DP3_ZERO_RNG,              IsDP3Enabled,      NULL,          "DP3 ZeroRngCnt",   DiffPres3ZeroRangePara    },
	{ DP3_ZERO_ADJ,              IsDP3Enabled,      NULL,          "DP3 ZeroAdjCnt",   DiffPres3ZeroAdjustPara    },
	{ DP3_ALARM_SETUP,            IsAlarmEnabled,          IsDP3Enabled,     "DP3 Alarm Setup",      PresDiff3AlarmSetupPara },
	{ DP3_UPPER_ALARM_ON_LIMIT,   IsPresDiff3AlarmEnabled, IsDP3Enabled,     "DP3 Upr Alarm On",     PresDiff3UpperAlarmOnPara  },
	{ DP3_UPPER_ALARM_OFF_LIMIT,  IsPresDiff3AlarmEnabled, IsDP3Enabled,     "DP3 Upr Alarm Off",    PresDiff3UpperAlarmOffPara },
	{ DP3_LOWER_ALARM_ON_LIMIT,   IsPresDiff3AlarmEnabled, IsDP3Enabled,     "DP3 Lwr Alarm On",     PresDiff3LowerAlarmOnPara  },
	{ DP3_LOWER_ALARM_OFF_LIMIT,  IsPresDiff3AlarmEnabled, IsDP3Enabled,     "DP3 Lwr Alarm Off",    PresDiff3LowerAlarmOffPara },	   	   
	
	{ TEMPERATURE_UNIT,                 IsTemperatureEnabled,       NULL,           "Temperature Unit",   TemperatureUnitPara  },
	{ TEMP_ZERO_ADJ,                    IsTemperatureEnabled,       NULL,           "TempZeroAdjCnt",  TempZeroAdjustPara  },
	{ TEMP2_ZERO_ADJ,                    IsTemperature2Enabled,       NULL,          "Temp2ZeroAdjCnt",  Temp2ZeroAdjustPara  },
	{ TEMP_ALARM_SETUP,                 IsAlarmEnabled,         IsTemperatureEnabled,      "Temp Alarm Setup",      TempAlarmSetupPara },
	{ TEMP_UPPER_ALARM_ON_LIMIT,        IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Upr Alarm On",     TempUpperAlarmOnPara  },
	{ TEMP_UPPER_ALARM_OFF_LIMIT,       IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Upr Alarm Off",    TempUpperAlarmOffPara  },
	{ TEMP_LOWER_ALARM_ON_LIMIT,        IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Lwr Alarm On",     TempLowerAlarmOnPara  },
	{ TEMP_LOWER_ALARM_OFF_LIMIT,       IsTempAlarmEnabled,     IsTemperatureEnabled,      "Temp Lwr Alarm Off",    TempLowerAlarmOffPara  },
	{ TEMP_DIFF_ALARM_LIMIT,			IsTempAlarmEnabled,     IsTemperature2Enabled,     "Temp diff Alarm",    TempDiffAlarmPara  },
	{ TEMP_OUTPUT_TYPE,                 IsAlalogOutEnabled,  IsPIDEnabled,          "Temp Out Type",      TempOutputTypePara  },
	{ TEMP_OUT_LOW_TEMP,                IsTempOutEnabled,    IsAlalogOutEnabled,    "Temp MinOutput",    TempValForMinimumOutputPara  },
	{ TEMP_OUT_HIGH_TEMP,               IsTempOutEnabled,    IsAlalogOutEnabled,    "Temp MaxOutput",    TempValForMaximumOutputPara  },
	{ TEMP_PID_STARTUP_PERCENT,         IsTempPIDEnabled,    IsAlalogOutEnabled,    "Temp PID Start %",   TempPIDStartupPercentPara  },
	{ TEMP_PID_KP,                      IsTempPIDEnabled,    IsAlalogOutEnabled,    "Temp PID Kp",        TempPIDKpPara  },
	{ TEMP_PID_TI,                      IsTempPIDEnabled,    IsAlalogOutEnabled,    "Temp PID Ti",        TempPIDTiPara  },
	{ TEMP_PID_TD,                      IsTempPIDEnabled,    IsAlalogOutEnabled,    "Temp PID Td",        TempPIDTdPara  },
	{ TEMP_PID_SET_VALUE,               IsTempPIDEnabled,    IsAlalogOutEnabled,    "Temp PIDSetPoint",  TempPIDSetpointPara  },
	{ TEMP_HIGH_PID_RH_ADJUST,          IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiTempRHAdj",     TempHighPIDRHAdjustPara  },
	{ TEMP_HIGH_PID_RH_CASCADE_FACTOR,  IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiTempCasRHFct", TempHighPIDRHCascadeFactorPara  },
	{ TEMP_HIGH_PID_CASCADE_RH_STATE,   IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiTempCasOnRH",   TempHighPIDRHCascadeStatePara  },
	{ TEMP_LOW_PID_RH_ADJUST,           IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoTempRHAdj",     TempLowPIDRHAdjustPara  },
	{ TEMP_LOW_PID_RH_CASCADE_FACTOR,   IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoTempCasRHFct", TempLowPIDRHCascadeFactorPara  },
	{ TEMP_LOW_PID_CASCADE_RH_STATE,    IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoTempCasOnRH",   TempLowPIDRHCascadeStatePara  },
	{ TEMP_OUT_POLARITY,                IsTempOutActive,     IsAlalogOutEnabled,    "Temp OutPolarity",  TempOutPolarityPara  },
	{ TEMP_OUT_LOW_COUNT,               IsTempOutActive,     IsAlalogOutEnabled,    "Temp OutCntMin",   OutputCountsForTempMinimumPara  },
	{ TEMP_OUT_HIGH_COUNT,              IsTempOutActive,     IsAlalogOutEnabled,    "Temp OutCntTMax",   OutputCountsForTempMaximumPara  },

	{ HUMIDITY_UNIT,                    IsHumidityEnabled,          NULL,           "Humidity Unit",      HumidityUnitPara  },
	{ RH_ZERO_ADJ,                      IsHumidityEnabled,          NULL,           "RH ZeroAdjCnt",    HumidityZeroAdjustPara    },
	{ RH2_ZERO_ADJ,                     IsTemperature2Enabled,          NULL,           "RH2 ZeroAdjCnt",    Humidity2ZeroAdjustPara    },
	{ RH_ALARM_SETUP,                   IsAlarmEnabled,         IsHumidityEnabled,         "RH Alarm Setup",      HumidityAlarmSetupPara },
	{ RH_UPPER_ALARM_ON_LIMIT,          IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Upr Alarm On",     HumidityUpperAlarmOnPara  },
	{ RH_UPPER_ALARM_OFF_LIMIT,         IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Upr Alarm Off",    HumidityUpperAlarmOffPara  },
	{ RH_LOWER_ALARM_ON_LIMIT,          IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Lwr Alarm On",     HumidityLowerAlarmOnPara  },
	{ RH_LOWER_ALARM_OFF_LIMIT,         IsHumidityAlarmEnabled, IsHumidityEnabled,         "RH Lwr Alarm Off",    HumidityLowerAlarmOffPara  },
	{ RH_OUTPUT_TYPE,                   IsAlalogOutEnabled,  IsPIDEnabled,          "RH Out Type",        HumidityOutputTypePara  },
	{ RH_OUT_LOW_RH,                    IsRHOutEnabled,      IsAlalogOutEnabled,    "RH MinOutput",      HumidityValForMinimumOutputPara  },
	{ RH_OUT_HIGH_RH,                   IsRHOutEnabled,      IsAlalogOutEnabled,    "RH MaxOutput",      HumidityValForMaximumOutputPara  },
	{ RH_PID_STARTUP_PERCENT,           IsRHPIDEnabled,      IsAlalogOutEnabled,    "RH PID Start %",     HumidityPIDStartupPercentPara  },
	{ RH_PID_KP,                        IsRHPIDEnabled,      IsAlalogOutEnabled,    "RH PID Kp",    HumidityPIDKpPara  },
	{ RH_PID_TI,                        IsRHPIDEnabled,      IsAlalogOutEnabled,    "RH PID Ti",    HumidityPIDTiPara  },
	{ RH_PID_TD,                        IsRHPIDEnabled,      IsAlalogOutEnabled,    "RH PID Td",    HumidityPIDTdPara  },
	{ RH_PID_SET_VALUE,                 IsRHPIDEnabled,      IsAlalogOutEnabled,    "RH PID SetPoint",    HumidityPIDSetpointPara  },
	{ RH_HIGH_PID_TEMP_ADJUST,          IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiRHTempAdj",     HumidityHighPIDTempAdjustPara  },
	{ RH_HIGH_PID_TEMP_CASCADE_FACTOR,  IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiRHCasTempFct", HumidityHighPIDTempCascadeFactorPara },
	{ RH_HIGH_PID_CASCADE_TEMP_STATE,   IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDHiRHCas OnTemp",  HumidityHighPIDTempCascadeStatePara  },
	{ RH_LOW_PID_TEMP_ADJUST,           IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoRHTempAdj",     HumidityLowPIDTempAdjustPara  },
	{ RH_LOW_PID_TEMP_CASCADE_FACTOR,   IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoRHCasTempFct", HumidityLowPIDTempCascadeFactorPara  },
	{ RH_LOW_PID_CASCADE_TEMP_STATE,    IsTempRHPIDEnabled,  IsAlalogOutEnabled,    "PIDLoRHCasOnTemp",   HumidityLowPIDTempCascadeStatePara  },
	{ RH_OUT_POLARITY,                  IsRHOutActive,       IsAlalogOutEnabled,    "RH OutPolarity",    HumidityOutPolarityPara  },
	{ RH_OUT_LOW_COUNT,                 IsRHOutActive,       IsAlalogOutEnabled,    "RH OutCntMin",     OutputCountsForHumidityMinimumPara  },
	{ RH_OUT_HIGH_COUNT,                IsRHOutActive,       IsAlalogOutEnabled,    "RH OutCntMax",     OutputCountsForHumidityMaximumPara  },

	{ RTC_ENABLE_DISABLE,               NULL,               NULL,                   "Real Time Clock",    RTCSetupPara },
	{ TIME_HOURS,                       IsRTCEnabled,       NULL,                   "Time: Hour",         TimeHourPara  },
	{ TIME_MINUTES,                     IsRTCEnabled,       NULL,                   "Time: Minute",       TimeMinutePara  },
	{ TIME_SECONDS,                     IsRTCEnabled,       NULL,                   "Time: Second",       TimeSecondPara  },
	{ DATE_YEAR,                        IsRTCEnabled,       NULL,                   "Date: Year",         DateYearPara  },
	{ DATE_MONTH,                       IsRTCEnabled,       NULL,                   "Date: Month",        DateMonthPara  },
	{ DATE_DAY,                         IsRTCEnabled,       NULL,                   "Date: Day",          DateDayPara  },
	//  { LOGGING_INTERVAL,                 IsLoggingEnabled,   NULL,                   "Logging Interval",   LogggingIntervalPara },
	
	{ BROADCAST_INTERVAL,               IsSerialEnabled,    IsBroadcastEnabled,     "BroadcastInterval", BroadcastIntervalPara  },
		
	{ BUZZER_ON_TIME,                   IsAlarmEnabled,             NULL,           "BuzzerONTime",     BuzzerOnPara },
	{ BUZZER_OFF_TIME,                  IsAlarmEnabled,             NULL,           "BuzzerOFFTime",    BuzzerOffPara },     
	{ BUZZER_DISABLE_TIME,              IsAlarmEnabled,             NULL,           "BuzzerDISTime",    BuzzerDisablePara },     
	
	{ OUTPUT1_MAPPING,                  IsMasterReset,              NULL,           "OUTPUT1 MAPPING",   Output1MappingPara  },
	{ OUTPUT2_MAPPING,                  IsMasterReset,              NULL,           "OUTPUT2 MAPPING",   Output2MappingPara  },
	{ OUTPUT3_MAPPING,                  IsMasterReset,              NULL,           "OUTPUT3 MAPPING",   Output3MappingPara  },
	{ OUTPUT4_MAPPING,                  IsMasterReset,              NULL,           "OUTPUT4 MAPPING",   Output4MappingPara  },
	
	{ SOFT_VERSION,                     NULL,                       NULL,           "Version",            VersionPara },
	
	{ TEMP_RH_SENS_TYPE,                IsMasterReset,      IsTempOrRHEnabled,      "Temp/RH Sens type",  TempRHSensTypePara  },
	{ TEMP_RH2_SENS_TYPE,               IsMasterReset,      IsTempOrRHEnabled,      "Temp/RH2 Sens type",  TempRH2SensTypePara  },
	//{ TEMP_RH_SCAN_TIME,                IsMasterReset,      IsTempOrRHEnabled,      "Temp/RH Scan time",  TempRHScanTimePara  },
	//{ TEMP_RH_READING_AVERAGE,          IsMasterReset,      IsTempOrRHEnabled,      "Temp/RH Avg Cnt",	 TempRHAverageCntPara  },
	//{ PRES_READING_AVERAGE,             IsMasterReset,      IsPresEnabled,          "Pres Avg Cnt",       PresAverageCntPara  },
	
	{ SERIAL_BAUD_RATE_VAL,             IsMasterReset,      IsSerialEnabled,        "Serial Baudrate",    SerialBaudratePara  },
	{ SERIAL_DATA_STOP_PARITY_VAL,      IsMasterReset,      IsSerialEnabled,        "Ser D:St:Par Bits",  SerialDataStopParityPara },
		
	{ AHU_ID,                     NULL,                       NULL,           "AHU ID",            AHUIdPara },	
	{ AHU_CFM,                    NULL,                       NULL,           "AHU CFM",           AHUCFMPara },
	{ AHU_AREA1,                  NULL,                       NULL,           "AHU AREA1",         AHUArea1Para },		
	{ AHU_AREA2,                  NULL,                       NULL,           "AHU AREA2",         AHUArea2Para },
	{ AHU_AREA3,                  NULL,                       NULL,           "AHU AREA3",         AHUArea3Para },
	
	{ DP1_RANGE,                   IsMasterReset,      IsDP1Enabled, "DP1SenRng(Pa)",   PresAbsRange  },
	{ DP2_RANGE,                  IsMasterReset,      IsDP2Enabled,   "DP2SenRng(Pa)",     PresDiffRange  },
	{ DP3_RANGE,                 IsMasterReset,      IsDP3Enabled,   "DP3SenRng(Pa)",    PresDiff3Range  },
	//{ DP1_SENS_TYPE,               IsMasterReset,      IsDP1Enabled, "DP1SensType",     PresAbsSensTypePara  },
	//{ DP2_SENS_TYPE,              IsMasterReset,      IsDP2Enabled,  "DP2SensType",        PresDiffSensTypePara  },
	//{ DP3_SENS_TYPE,             IsMasterReset,      IsDP3Enabled,  "DP3SensType",       PresDiff3SensTypePara  },
	//{ DP1_SENS_MIN,                IsMasterReset,      IsDP1Enabled, "DP1SensMinCnt",   PresAbsSensMinCnt  },
	//{ DP1_SENS_MAX,                IsMasterReset,      IsDP1Enabled,  "DP1SensMaxCnt",  PresAbsSensMaxCnt  },
	//{ DP2_SENS_MIN,               IsMasterReset,      IsDP2Enabled, "DP2SensMinCnt",		 PresDiffSensMinCnt  },
	//{ DP2_SENS_MAX,               IsMasterReset,      IsDP2Enabled,  "DP2SensMaxCnt",      PresDiffSensMaxCnt  },
	//{ DP3_SENS_MIN,              IsMasterReset,      IsDP3Enabled, "DP3SensMinCnt",		 PresDiff3SensMinCnt  },
	//{ DP3_SENS_MAX,              IsMasterReset,      IsDP3Enabled,  "DP3SensMaxCnt",     PresDiff3SensMaxCnt  },
	{ CUSTOMER_ID,                      NULL,                       NULL,           "CustomerID",         CustomerIDPara  },
	{ SERIAL_NO,                        NULL,                       NULL,           "SerialNo",           SerialNoPara  },
	{ CONFIG_SYSTEM,                    NULL,                       NULL,           "SystemConfig",       SystemConfigPara  },
	{ CFG_PASSWORD,                     IsMasterReset,      NotConfigPassword,      "Config Password",    ConfigPasswordPara  },
};


static unsigned char editorState = 0;
static int activePara = 0;
static PARA_INFO activeParaInfo;
static int activeParaValue;
static unsigned int keyTimer;

static void DisplayEditor(char keysPressed, char keys)
{
   uint8_t paraCnt, paraUpReq;
   char * dispStr;

   LCDControl = EDITOR;
   if( keys == (char)-1 && keysPressed == (char)-1 )
   {
      /* Display Keyboard start message */
      OSSemaTakeEver(UIMutex);
      if (IsLCDDisplay())
      {
			uc1638_FillScreen(BLANK1);
			
			ks0xxx_SelectFont( Font_Verdana_10x24, ks0xxx_ReadFontData, BLACK);
			
			uc1638_GotoXY(systemXLoc, PARA_SETUP_Y_LOC);
			PrintLine(1, "PARAMETER EDITOR" );
			ks0xxx_DrawHoriLine(1, 24, 238, BLACK);
			uc1638_GotoXY(systemXLoc, 48);
			PrintLine(1, "Use PARA key to" );
			uc1638_GotoXY(systemXLoc, 72);
			PrintLine(1, "  select parameter" );
			uc1638_GotoXY(systemXLoc, 96);
			PrintLine(1, "UP/DN to change" );
			uc1638_GotoXY(systemXLoc, 120);
			PrintLine(1, "the parameter value" );
      }
      OSSemaGive(UIMutex);
      editorState = 1;
      activePara = -1;
      keyTimer = 0;
      return;
   }

   if( editorState == (unsigned char)1 )
   {  // wait for setup key to be pressed. Ignore up/down keys until then
      if(( keysPressed & SETUP_KEY) == 0 )
         return;
      OSSemaTakeEver(UIMutex);
      if (IsLCDDisplay())
      {
         uc1638_FillScreen(BLANK1);
         uc1638_GotoXY(systemXLoc, PARA_SETUP_Y_LOC);
         //ks0xxx_Puts_P(PSTR("PARAMETER EDITOR"));
		 PrintLine(1, "PARAMETER EDITOR" );
		 ks0xxx_DrawHoriLine(1, 24, 238, BLACK);
      }
      OSSemaGive(UIMutex);
   }

   if(( keys & SETUP_KEY) != 0 )
   {
      paraUpReq = 0;
      if(( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) == 0 )
         paraUpReq = 1;
      if(( keysPressed & DOWN_KEY ) != 0 && ( keys & UP_KEY ) == 0 )
      {  // treat as setup key
         keysPressed |= SETUP_KEY;
         keysPressed &= ~DOWN_KEY;
      }

      if(( keysPressed & SETUP_KEY) != 0 || paraUpReq )
      { 
		 // Setup key pressed select next parameter
         for( paraCnt = 0; paraCnt < sizeof(paraTableLCD)/sizeof(paraTableLCD[0]); paraCnt++ )
         {
            if( paraUpReq )
            {
               if( activePara )
                  activePara--;
               else
                  activePara = sizeof(paraTableLCD)/sizeof(paraTableLCD[0])-1;
            }
            else
            {
               activePara++;
               if(activePara >= sizeof(paraTableLCD)/sizeof(paraTableLCD[0]))
                  activePara = 0;
            }
   
            GetParaInfoByIndex( activePara, &activeParaInfo );
            if( activeParaInfo.priValidationFunction != NULL )
               if((*activeParaInfo.priValidationFunction)() == 0 )
                  continue;
            if( activeParaInfo.secValidationFunction != NULL )
               if((*activeParaInfo.secValidationFunction)() == 0 )
                  continue;
            break;
         }
         if( paraCnt == sizeof(paraTableLCD)/sizeof(paraTableLCD[0]))
         {  /* No parameter is required */
            StartDisplaySensorValues();
            return;
         }
         // Get and Display current Parameter value
         OSSemaTakeEver(UIMutex);
         if (IsLCDDisplay())
         {
            uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
            PrintLine(1, activeParaInfo.displayText );
            uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
			
            PrintLine(1, ( * activeParaInfo.parameterHandler)( PARA_READ, &activeParaValue ));
         }
         OSSemaGive(UIMutex);
   
         editorState = 2;
         return;
      }
      return;
   }

   if( editorState == (unsigned char)2 )
   {  
      // wait for at least one key is pressed
      if(( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) == 0 )
      {
         activeParaValue++;
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else if(( keys & UP_KEY ) == 0 && ( keysPressed & DOWN_KEY ) != 0 )
      {
         activeParaValue--;
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else if(( keysPressed & UP_KEY ) != 0 && ( keysPressed & DOWN_KEY ) != 0 )
      {
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_DEFAULT, &activeParaValue );  // Para default
      }
      else
         return;

      // Display current Parameter value
      if (IsLCDDisplay())
      {
         uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
         PrintLine( 1, activeParaInfo.displayText );
         uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
         PrintLine( 1, dispStr );
      }
      OSSemaGive(UIMutex);

      editorState = 3;
      return;
   }

   if( editorState == (unsigned char)3 )
   {
      if(( keysPressed & ( UP_KEY | DOWN_KEY )) != 0 )
         keyTimer = 0; // reset auto up/down timer counter

      if((( keysPressed & UP_KEY ) != 0 && ( keys & DOWN_KEY ) != 0 ) || (( keys & UP_KEY ) != 0 && ( keysPressed & DOWN_KEY ) != 0 ))
      {
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_DEFAULT, &activeParaValue );  // Para default
      }
      else if (( keys & DOWN_KEY ) == 0 && ( keys & UP_KEY ) == 0 )
         return;  // Do not do anything 
      else if (( keys & DOWN_KEY ) != 0 && ( keys & UP_KEY ) != 0 )
         return;  // Do not do anything 
      else if (( keysPressed & UP_KEY ) != 0)
      {
         activeParaValue++;
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else if(( keysPressed & DOWN_KEY ) != 0)
      {
         activeParaValue--;
         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue);  // Para value +
      }
      else
      {
         // Process timeouts and set new values
         // increment timer counter
         keyTimer++;
         if(( keyTimer % ( KEY_AUTO_CHG_TIME/KEYBOARD_LOOP_TIME )) != 0 )
            return;   /* 300 ms update time */
         if(( keyTimer / ( KEY_AUTO_CHG_FAST_TIME/KEYBOARD_LOOP_TIME )) == 0 )
            paraCnt = 1;
         else if(( keyTimer / ( KEY_AUTO_CHG_FAST_TIME/KEYBOARD_LOOP_TIME )) == 1 )
            paraCnt = KEY_AUTO_FAST_STEP;
         else
            paraCnt = KEY_AUTO_FAST_STEP * KEY_AUTO_FAST_STEP;

         if (( keys & DOWN_KEY ) != 0 )
            activeParaValue -= paraCnt;
         else
            activeParaValue += paraCnt;

         OSSemaTakeEver(UIMutex);
         dispStr = ( * activeParaInfo.parameterHandler)( PARA_WRITE, &activeParaValue );  // Para default
      }
      // Display current Parameter value
      if (IsLCDDisplay())
      {
         uc1638_GotoXY(systemXLoc, PARA_INFO_Y_LOC);
         PrintLine( 1, activeParaInfo.displayText );
         uc1638_GotoXY(systemXLoc, PARA_VAL_Y_LOC);
         PrintLine( 1, dispStr );
      }
      OSSemaGive(UIMutex);
   }
}

static void SetAllParametersToDefault(void)
{
	PARA_INFO paraInfoVal;
	uint8_t i;
	int16_t paraValue;

	for (i = 0; i < (uint8_t)(sizeof(paraTableLCD)/sizeof(paraTableLCD[0])); i++)
	{
		GetParaInfoByIndex( i, &paraInfoVal );
		(*paraInfoVal.parameterHandler)( PARA_DEFAULT, &paraValue );
	}
}


// returns error code as a function return and read value in paraValue
uint8_t GetParameter( char paraId, int *paraValue )
{
   PARA_INFO paraInfoVal;

   if( GetParaInfo( paraId, &paraInfoVal ) != 0 )
      return ERROR_PARA_INVALID;

	if( paraInfoVal.priValidationFunction != NULL && paraInfoVal.priValidationFunction != IsMasterReset)
      if((*paraInfoVal.priValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.secValidationFunction != NULL && paraInfoVal.secValidationFunction != IsMasterReset)
      if((*paraInfoVal.secValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
		 
		 
   //if( paraInfoVal.priValidationFunction != NULL )
      //if((*paraInfoVal.priValidationFunction)() == 0 )
         //return ERROR_PARA_NOT_SUPORTED;
   //if( paraInfoVal.secValidationFunction != NULL )
      //if((*paraInfoVal.secValidationFunction)() == 0 )
         //return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.parameterHandler != NULL )
      (* paraInfoVal.parameterHandler)( PARA_READ_VALUE_ONLY, paraValue );

   return ERROR_OK;
}

uint8_t GetParameterStr( uint8_t paraId, uint8_t *parastr )
{
	uint16_t eepromAddr=EA_IP1_HIGH_NAME;
	
	switch(paraId)
	{
		case IP1_HIGH_NAME: eepromAddr = EA_IP1_HIGH_NAME;	break;
		case IP1_LOW_NAME:  eepromAddr = EA_IP1_LOW_NAME;	break;
		case IP2_HIGH_NAME: eepromAddr = EA_IP2_HIGH_NAME;	break;
		case IP2_LOW_NAME:  eepromAddr = EA_IP2_LOW_NAME;	break;
		case IP3_HIGH_NAME: eepromAddr = EA_IP3_HIGH_NAME;	break;
		case IP3_LOW_NAME:  eepromAddr = EA_IP3_LOW_NAME;	break;
		case IP4_HIGH_NAME: eepromAddr = EA_IP4_HIGH_NAME;	break;
		case IP4_LOW_NAME:  eepromAddr = EA_IP4_LOW_NAME;	break;
		case OP1_HIGH_NAME: eepromAddr = EA_OP1_HIGH_NAME;	break;
		case OP1_LOW_NAME:  eepromAddr = EA_OP1_LOW_NAME;	break;
		case OP2_HIGH_NAME: eepromAddr = EA_OP2_HIGH_NAME;	break;
		case OP2_LOW_NAME:  eepromAddr = EA_OP2_LOW_NAME;	break;
		case OP3_HIGH_NAME: eepromAddr = EA_OP3_HIGH_NAME;	break;
		case OP3_LOW_NAME:  eepromAddr = EA_OP3_LOW_NAME;	break;
		case OP4_HIGH_NAME: eepromAddr = EA_OP4_HIGH_NAME;	break;
		case OP4_LOW_NAME:  eepromAddr = EA_OP4_LOW_NAME;	break;
		
		default:	return ERROR_PARA_INVALID;		break;
	}
	
	OSReadEEPromBlock((uint8_t *)eepromAddr, parastr, 10);
	
	return ERROR_OK;
}

// returns error code as a function return and actual set value in paraValue
uint8_t SetParameter( char paraId, int *paraValue )
{
   PARA_INFO paraInfoVal;

   if( GetParaInfo( paraId, &paraInfoVal ) != 0 )
      return ERROR_PARA_INVALID;

   if( paraInfoVal.priValidationFunction != NULL && paraInfoVal.priValidationFunction != IsMasterReset)
      if((*paraInfoVal.priValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.secValidationFunction != NULL && paraInfoVal.secValidationFunction != IsMasterReset)
      if((*paraInfoVal.secValidationFunction)() == 0 )
         return ERROR_PARA_NOT_SUPORTED;
   if( paraInfoVal.parameterHandler != NULL )
   {
//      OSSemaTakeEver(UIMutex);
      (* paraInfoVal.parameterHandler)( PARA_WRITE_VALUE_ONLY, paraValue );
//      OSSemaGive(UIMutex);
   }
   return ERROR_OK;
}

// returns error code as a function return and actual set value in paraValue
uint8_t SetParameterStr( uint8_t paraId, uint8_t *parastr )
{
	uint16_t eepromAddr=EA_IP1_HIGH_NAME;
	
	switch(paraId)
	{
		case IP1_HIGH_NAME: eepromAddr = EA_IP1_HIGH_NAME;	break;
		case IP1_LOW_NAME:  eepromAddr = EA_IP1_LOW_NAME;	break;
		case IP2_HIGH_NAME: eepromAddr = EA_IP2_HIGH_NAME;	break;
		case IP2_LOW_NAME:  eepromAddr = EA_IP2_LOW_NAME;	break;
		case IP3_HIGH_NAME: eepromAddr = EA_IP3_HIGH_NAME;	break;
		case IP3_LOW_NAME:  eepromAddr = EA_IP3_LOW_NAME;	break;
		case IP4_HIGH_NAME: eepromAddr = EA_IP4_HIGH_NAME;	break;
		case IP4_LOW_NAME:  eepromAddr = EA_IP4_LOW_NAME;	break;
		case OP1_HIGH_NAME: eepromAddr = EA_OP1_HIGH_NAME;	break;
		case OP1_LOW_NAME:  eepromAddr = EA_OP1_LOW_NAME;	break;
		case OP2_HIGH_NAME: eepromAddr = EA_OP2_HIGH_NAME;	break;
		case OP2_LOW_NAME:  eepromAddr = EA_OP2_LOW_NAME;	break;
		case OP3_HIGH_NAME: eepromAddr = EA_OP3_HIGH_NAME;	break;
		case OP3_LOW_NAME:  eepromAddr = EA_OP3_LOW_NAME;	break;
		case OP4_HIGH_NAME: eepromAddr = EA_OP4_HIGH_NAME;	break;
		case OP4_LOW_NAME:  eepromAddr = EA_OP4_LOW_NAME;	break;
		default:	return ERROR_PARA_INVALID;		break;
	}
	
	OSWriteEEPromBlock((uint8_t *)eepromAddr, parastr, 10);
	
	return ERROR_OK;
}

int GetParameterValue( PARAMETER paraId )
{
   PARA_INFO paraInfoVal;
   int16_t value = 0;

   if( GetParaInfo( paraId, &paraInfoVal ) == 0 )
      (* paraInfoVal.parameterHandler)( PARA_READ_VALUE_ONLY, &value );

   return value;
}

/* Returns 0 if no error else returns 1 */
static uint8_t GetParaInfo( PARAMETER reqId, PARA_INFO * paraInfoVal )
{
   uint8_t * start, * storePtr;
   uint8_t i, id;

   for( i = 0, start = (uint8_t *)paraTableLCD; i < (uint8_t)(sizeof (paraTableLCD) / sizeof( PARA_INFO)); i++, start += sizeof( PARA_INFO))
   {
      id = pgm_read_byte( start );
      if( id == (uint8_t)reqId)
      {
         storePtr = (uint8_t *)paraInfoVal;
         for( i = 0; i < sizeof(PARA_INFO); i++, storePtr++, start++)
            *storePtr = pgm_read_byte( start );
         return 0;
      }
   }
   return 1;
}

/* Returns 0 if no error else returns 1 */
static uint8_t GetParaInfoByIndex( uint8_t index, PARA_INFO * paraInfoVal )
{
   uint8_t * start = (uint8_t *)paraTableLCD, *storePtr;
   uint8_t i;

   if( index >= (uint8_t)(sizeof (paraTableLCD) / sizeof( PARA_INFO)))
      return 1;

   start += (sizeof( PARA_INFO) * index);
   storePtr = (uint8_t *)paraInfoVal;
   for( i = 0; i < sizeof(PARA_INFO); i++, storePtr++, start++)
      *storePtr = pgm_read_byte( start );
   return 0;
}


static void ParameterOperationsOnLowSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t minValue, int16_t maxValParaId, unsigned int eepromAddr )
{
   int16_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_READ )
   {
      *paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	  return;
   }
   if(operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltValue;
   }
   else
   {
      if ( *paraValue <= minValue)
         *paraValue = minValue;
      temp = GetParameterValue(maxValParaId);
      if ( *paraValue >= temp)
         *paraValue = temp-1;
   }
   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
}


static void ParameterOperationsOnHighSettings( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int16_t maxValue, int16_t minValParaId, unsigned int eepromAddr )
{
   int16_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_READ )
   {
      *paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	  return;
   }
   else if(operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltValue;
   }
   else
   {
      temp = GetParameterValue(minValParaId);
      if ( *paraValue <= temp)
         *paraValue = temp+1;
      if ( *paraValue > maxValue)
         *paraValue = maxValue;
   }
   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
}


//static char * PartMCUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnHighSettings(operationType,
	//paraValue,
	//D_PART_MC_UPPER_ALARM_ON_LIMIT,
	//MAX_PART_MC_UPPER_ALARM_ON_LIMIT,
	//PARTICAL_MC_UPPER_ALARM_OFF_LIMIT,
	//EA_PART_MC_UPPER_ALARM_ON_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatParticalOneLine( displayStr, ERROR_OK, *paraValue );
//}
//
//static char * PartMCUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnLowSettings(operationType,
	//paraValue,
	//D_PART_MC_UPPER_ALARM_OFF_LIMIT,
	//MIN_PART_MC_UPPER_ALARM_OFF_LIMIT,
	//PARTICAL_MC_UPPER_ALARM_ON_LIMIT,
	//EA_PART_MC_UPPER_ALARM_OFF_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatParticalOneLine( displayStr, ERROR_OK, *paraValue );
//}
//
//static char * PartVOCUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnHighSettings(operationType,
	//paraValue,
	//D_PART_VOC_UPPER_ALARM_ON_LIMIT,
	//MAX_PART_VOC_UPPER_ALARM_ON_LIMIT,
	//PARTICAL_VOC_UPPER_ALARM_OFF_LIMIT,
	//EA_PART_VOC_UPPER_ALARM_ON_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatPartialStuffOneLine( displayStr, ERROR_OK, *paraValue );
//}
//
//static char * PartVOCUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnLowSettings(operationType,
	//paraValue,
	//D_PART_VOC_UPPER_ALARM_OFF_LIMIT,
	//MIN_PART_VOC_UPPER_ALARM_OFF_LIMIT,
	//PARTICAL_VOC_UPPER_ALARM_ON_LIMIT,
	//EA_PART_VOC_UPPER_ALARM_OFF_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatPartialStuffOneLine( displayStr, ERROR_OK, *paraValue );
//}
//
//static char * PartNOXUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnHighSettings(operationType,
	//paraValue,
	//D_PART_NOX_UPPER_ALARM_ON_LIMIT,
	//MAX_PART_NOX_UPPER_ALARM_ON_LIMIT,
	//PARTICAL_NOX_UPPER_ALARM_OFF_LIMIT,
	//EA_PART_NOX_UPPER_ALARM_ON_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatPartialStuffOneLine( displayStr, ERROR_OK, *paraValue );
//}
//
//static char * PartNOXUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsOnLowSettings(operationType,
	//paraValue,
	//D_PART_NOX_UPPER_ALARM_OFF_LIMIT,
	//MIN_PART_NOX_UPPER_ALARM_OFF_LIMIT,
	//PARTICAL_NOX_UPPER_ALARM_ON_LIMIT,
	//EA_PART_NOX_UPPER_ALARM_OFF_LIMIT );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//return FormatPartialStuffOneLine( displayStr, ERROR_OK, *paraValue );
//}

static char * PresAbsUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP1_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_DP1_UPPER_ALARM_OFF_LIMIT, 
                                    DP1_UPPER_ALARM_ON_LIMIT, 
                                    EA_DP1_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP1_UPPER_ALARM_ON_LIMIT, 
                                     MAX_DP1_UPPER_ALARM_ON_LIMIT, 
                                     DP1_UPPER_ALARM_OFF_LIMIT, 
                                     EA_DP1_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_DP1_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_DP1_LOWER_ALARM_OFF_LIMIT, 
                                    DP1_LOWER_ALARM_ON_LIMIT, 
                                    EA_DP1_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_DP1_LOWER_ALARM_ON_LIMIT, 
                                     MIN_DP1_LOWER_ALARM_ON_LIMIT, 
                                     DP1_LOWER_ALARM_OFF_LIMIT, 
                                     EA_DP1_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}


static char * PresDiffUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP2_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_DP2_UPPER_ALARM_OFF_LIMIT, 
                                    DP2_UPPER_ALARM_ON_LIMIT, 
                                    EA_DP2_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP2_UPPER_ALARM_ON_LIMIT, 
                                     MAX_DP2_UPPER_ALARM_ON_LIMIT, 
                                     DP2_UPPER_ALARM_OFF_LIMIT, 
                                     EA_DP2_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_DP2_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_DP2_LOWER_ALARM_OFF_LIMIT, 
                                    DP2_LOWER_ALARM_ON_LIMIT, 
                                    EA_DP2_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_DP2_LOWER_ALARM_ON_LIMIT, 
                                     MIN_DP2_LOWER_ALARM_ON_LIMIT, 
                                     DP2_LOWER_ALARM_OFF_LIMIT, 
                                     EA_DP2_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiff3UpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnLowSettings(operationType,
	paraValue,
	D_DP3_UPPER_ALARM_OFF_LIMIT,
	MIN_DP3_UPPER_ALARM_OFF_LIMIT,
	DP3_UPPER_ALARM_ON_LIMIT,
	EA_DP3_UPPER_ALARM_OFF_LIMIT );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatDP3OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiff3UpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnHighSettings(operationType,
	paraValue,
	D_DP3_UPPER_ALARM_ON_LIMIT,
	MAX_DP3_UPPER_ALARM_ON_LIMIT,
	DP3_UPPER_ALARM_OFF_LIMIT,
	EA_DP3_UPPER_ALARM_ON_LIMIT );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatDP3OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiff3LowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnHighSettings(operationType,
	paraValue,
	D_DP3_LOWER_ALARM_OFF_LIMIT,
	MAX_DP3_LOWER_ALARM_OFF_LIMIT,
	DP3_LOWER_ALARM_ON_LIMIT,
	EA_DP3_LOWER_ALARM_OFF_LIMIT );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatDP3OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiff3LowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnLowSettings(operationType,
	paraValue,
	D_DP3_LOWER_ALARM_ON_LIMIT,
	MIN_DP3_LOWER_ALARM_ON_LIMIT,
	DP3_LOWER_ALARM_OFF_LIMIT,
	EA_DP3_LOWER_ALARM_ON_LIMIT );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatDP3OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_TEMP_UPPER_ALARM_OFF_LIMIT, 
                                    TEMP_UPPER_ALARM_ON_LIMIT, 
                                    EA_TEMP_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_UPPER_ALARM_ON_LIMIT, 
                                     MAX_TEMP_UPPER_ALARM_ON_LIMIT, 
                                     TEMP_UPPER_ALARM_OFF_LIMIT, 
                                     EA_TEMP_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}


static char * TempLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_TEMP_LOWER_ALARM_OFF_LIMIT, 
                                    TEMP_LOWER_ALARM_ON_LIMIT, 
                                    EA_TEMP_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempDiffAlarmPara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_TEMP_DIFF_ALARM_LIMIT,
	500,
	-2000,
	2000 );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;

	return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_LOWER_ALARM_ON_LIMIT, 
                                     MIN_TEMP_LOWER_ALARM_ON_LIMIT, 
                                     TEMP_LOWER_ALARM_OFF_LIMIT, 
                                     EA_TEMP_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityUpperAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_RH_UPPER_ALARM_OFF_LIMIT, 
                                    MIN_RH_UPPER_ALARM_OFF_LIMIT, 
                                    RH_UPPER_ALARM_ON_LIMIT, 
                                    EA_RH_UPPER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityUpperAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_RH_UPPER_ALARM_ON_LIMIT, 
                                     MAX_RH_UPPER_ALARM_ON_LIMIT, 
                                     RH_UPPER_ALARM_OFF_LIMIT, 
                                     EA_RH_UPPER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityLowerAlarmOffPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                    paraValue, 
                                    D_RH_LOWER_ALARM_OFF_LIMIT, 
                                    MAX_RH_LOWER_ALARM_OFF_LIMIT, 
                                    RH_LOWER_ALARM_ON_LIMIT, 
                                    EA_RH_LOWER_ALARM_OFF_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityLowerAlarmOnPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                     paraValue, 
                                     D_RH_LOWER_ALARM_ON_LIMIT, 
                                     MIN_RH_LOWER_ALARM_ON_LIMIT, 
                                     RH_LOWER_ALARM_OFF_LIMIT, 
                                     EA_RH_LOWER_ALARM_ON_LIMIT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP1_OUT_LOW_PRES, 
                                    MIN_DP1_OUT_LOW_PRES, 
                                    DP1_OUT_HIGH_PRES, 
                                    EA_DP1_OUT_LOW_PRES );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresAbsValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP1_OUT_HIGH_PRES, 
                                     MAX_DP1_OUT_HIGH_PRES, 
                                     DP1_OUT_LOW_PRES, 
                                     EA_DP1_OUT_HIGH_PRES );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP2_OUT_LOW_PRES, 
                                    MIN_DP2_OUT_LOW_PRES, 
                                    DP2_OUT_HIGH_PRES, 
                                    EA_DP2_OUT_LOW_PRES );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP2_OUT_HIGH_PRES, 
                                     MAX_DP2_OUT_HIGH_PRES, 
                                     DP2_OUT_LOW_PRES, 
                                     EA_DP2_OUT_HIGH_PRES );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_OUT_LOW_TEMP, 
                                    MIN_TEMP_OUT_LOW_TEMP, 
                                    TEMP_OUT_HIGH_TEMP, 
                                    EA_TEMP_OUT_LOW_TEMP );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_OUT_HIGH_TEMP, 
                                     MAX_TEMP_OUT_HIGH_TEMP, 
                                     TEMP_OUT_LOW_TEMP, 
                                     EA_TEMP_OUT_HIGH_TEMP );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityValForMinimumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_RH_OUT_LOW_RH, 
                                    MIN_RH_OUT_LOW_RH, 
                                    RH_OUT_HIGH_RH, 
                                    EA_RH_OUT_LOW_RH );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityValForMaximumOutputPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_RH_OUT_HIGH_RH, 
                                     MAX_RH_OUT_HIGH_RH, 
                                     RH_OUT_LOW_RH, 
                                     EA_RH_OUT_HIGH_RH );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * OutputCountsForPresAbsMinimumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP1_OUT_LOW_COUNT, 
                                    MIN_DP1_OUT_LOW_COUNT, 
                                    DP1_OUT_HIGH_COUNT, 
                                    EA_DP1_OUT_LOW_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForPresAbsMaximumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP1_OUT_HIGH_COUNT, 
                                     MAX_DP1_OUT_HIGH_COUNT, 
                                     DP1_OUT_LOW_COUNT, 
                                     EA_DP1_OUT_HIGH_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForPresDiffMinimumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_DP2_OUT_LOW_COUNT, 
                                    MIN_DP2_OUT_LOW_COUNT, 
                                    DP2_OUT_HIGH_COUNT, 
                                    EA_DP2_OUT_LOW_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForPresDiffMaximumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_DP2_OUT_HIGH_COUNT, 
                                     MAX_DP2_OUT_HIGH_COUNT, 
                                     DP2_OUT_LOW_COUNT, 
                                     EA_DP2_OUT_HIGH_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForTempMinimumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_TEMP_OUT_LOW_COUNT, 
                                    MIN_TEMP_OUT_LOW_COUNT, 
                                    TEMP_OUT_HIGH_COUNT, 
                                    EA_TEMP_OUT_LOW_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForTempMaximumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_TEMP_OUT_HIGH_COUNT, 
                                     MAX_TEMP_OUT_HIGH_COUNT, 
                                     TEMP_OUT_LOW_COUNT, 
                                     EA_TEMP_OUT_HIGH_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForHumidityMinimumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnLowSettings(operationType, 
                                    paraValue, 
                                    D_RH_OUT_LOW_COUNT, 
                                    MIN_RH_OUT_LOW_COUNT, 
                                    RH_OUT_HIGH_COUNT, 
                                    EA_RH_OUT_LOW_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * OutputCountsForHumidityMaximumPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsOnHighSettings(operationType, 
                                     paraValue, 
                                     D_RH_OUT_HIGH_COUNT, 
                                     MAX_RH_OUT_HIGH_COUNT, 
                                     RH_OUT_LOW_COUNT, 
                                     EA_RH_OUT_HIGH_COUNT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * PressureAbsUnitPara( uint8_t operationType, int16_t * paraValue )
{
   return PressureUnitProcessPara( operationType, paraValue, D_DP1_UNIT, EA_DP1_UNIT );
}

static char * PressureDiffUnitPara( uint8_t operationType, int16_t * paraValue )
{
   return PressureUnitProcessPara( operationType, paraValue, D_DP2_UNIT, EA_DP2_UNIT );
}

static char * PressureDiff3UnitPara( uint8_t operationType, int16_t * paraValue )
{
	return PressureUnitProcessPara( operationType, paraValue, D_DP3_UNIT, EA_DP3_UNIT );
}

static char * PressureUnitProcessPara( uint8_t operationType, int16_t * paraValue, int16_t dfltValue, unsigned int eepromAddr )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eepromAddr, dfltValue, PRESSURE_PSI, PRESSURE_CFM );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == PRESSURE_PSI )
         locked_sprintf_P( displayStr, PSTR("PSI"));
      else if( *paraValue == PRESSURE_MILIBAR )
         locked_sprintf_P( displayStr, PSTR("mBAR"));
      else if( *paraValue == PRESSURE_PA )
         locked_sprintf_P( displayStr, PSTR("Pa"));
      else if( *paraValue == PRESSURE_H2O_MM )
         locked_sprintf_P( displayStr, PSTR("mH2O"));
      else if( *paraValue == PRESSURE_H2O_INCH )
         locked_sprintf_P( displayStr, PSTR("iH2O"));
      else if( *paraValue == PRESSURE_HG_INCH )
         locked_sprintf_P( displayStr, PSTR("inHg"));
      else if( *paraValue == PRESSURE_PSF )
         locked_sprintf_P( displayStr, PSTR("PSF"));
      else if( *paraValue == PRESSURE_CFM )
         locked_sprintf_P( displayStr, PSTR("CFM"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * TemperatureUnitPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMPERATURE_UNIT, D_TEMPERATURE_UNIT, TEMPERATURE_DEG_C, TEMPERATURE_DEG_K );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == TEMPERATURE_DEG_C )
         locked_sprintf_P( displayStr, PSTR("*C"));
      else if( *paraValue == TEMPERATURE_DEG_F )
         locked_sprintf_P( displayStr, PSTR("*F"));
      else if( *paraValue == TEMPERATURE_DEG_K )
         locked_sprintf_P( displayStr, PSTR("*K"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * HumidityUnitPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_HUMIDITY_UNIT, D_HUMIDITY_UNIT, HUMIDITY_RH, HUMIDITY_RH );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == HUMIDITY_RH )
         locked_sprintf_P( displayStr, PSTR("%%RH"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}

static char * DP1AreaLengthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_LENGTH,
	D_DP1_AREA_LEN,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d mm", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP2AreaLengthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP2_AREA_LENGTH,
	D_DP2_AREA_LEN,
	MIN_DP2_AREA,
	MAX_DP2_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d mm", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP3AreaLengthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP3_AREA_LENGTH,
	D_DP3_AREA_LEN,
	MIN_DP3_AREA,
	MAX_DP3_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d mm", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP1AreaWidthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_WIDTH,
	D_DP1_AREA_WID,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP2AreaWidthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP2_AREA_WIDTH,
	D_DP2_AREA_WID,
	MIN_DP2_AREA,
	MAX_DP2_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP3AreaWidthPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP3_AREA_WIDTH,
	D_DP3_AREA_WID,
	MIN_DP3_AREA,
	MAX_DP3_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP1AreaRadiousPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP1_AREA_RADIOUS,
	D_DP1_AREA_RAD,
	MIN_DP1_AREA,
	MAX_DP1_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP2AreaRadiousPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP2_AREA_RADIOUS,
	D_DP2_AREA_RAD,
	MIN_DP2_AREA,
	MAX_DP2_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}

static char * DP3AreaRadiousPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettings(operationType,
	paraValue,
	EA_DP3_AREA_RADIOUS,
	D_DP3_AREA_RAD,
	MIN_DP3_AREA,
	MAX_DP3_AREA );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	sprintf( displayStr, "%d mm", *paraValue );
	return displayStr;
}


//static char * PresAbsAreaPara( uint8_t operationType, int16_t * paraValue )
//{
  //ParameterOperationsSettings(operationType, 
                               //paraValue, 
                               //EA_DP1_AREA,
                               //D_DP1_AREA, 
                               //MIN_DP1_AREA, 
                               //MAX_DP1_AREA );
   //if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      //return NULL;
   //sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
   //return displayStr;
//}
//
//
//static char * PresDiffAreaPara( uint8_t operationType, int16_t * paraValue )
//{
  //ParameterOperationsSettings(operationType, 
                               //paraValue, 
                               //EA_DP2_AREA,
                               //D_DP2_AREA, 
                               //MIN_DP2_AREA, 
                               //MAX_DP2_AREA );
   //if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      //return NULL;
   //sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
   //return displayStr;
//}
//
//static char * PresDiff3AreaPara( uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsSettings(operationType,
	//paraValue,
	//EA_DP3_AREA,
	//D_DP3_AREA,
	//MIN_DP3_AREA,
	//MAX_DP3_AREA );
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//sprintf( displayStr, "%d.%d", *paraValue / 1000, *paraValue % 1000 );
	//return displayStr;
//}

static char * SerialBaudratePara(uint8_t operationType, int16_t * paraValue )
{
   const char * baudDisp;

   ParameterOperationsSettingsRollover( operationType, paraValue, EA_SERIAL_BAUD_RATE_VAL, D_SERIAL_BAUD_RATE_VAL, MIN_SERIAL_BAUD_RATE_VAL, MAX_SERIAL_BAUD_RATE_VAL );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch( *paraValue )
      {
         case SERIAL_BAUD_1200: baudDisp = PSTR("1200");
                                break;
         case SERIAL_BAUD_2400: baudDisp = PSTR("2400");
                                break;
         case SERIAL_BAUD_4800: baudDisp = PSTR("4800");
                                break;
         case SERIAL_BAUD_14400: baudDisp = PSTR("14400");
                                break;
         case SERIAL_BAUD_19200: baudDisp = PSTR("19200");
                                break;
         case SERIAL_BAUD_28800: baudDisp = PSTR("28800");
                                break;
         case SERIAL_BAUD_38400: baudDisp = PSTR("38400");
                                break;
         case SERIAL_BAUD_57600: baudDisp = PSTR("57600");
                                break;
         case SERIAL_BAUD_9600: 
         default:               baudDisp = PSTR("9600");
                                break;

      }
      locked_sprintf_P( displayStr, baudDisp);
   }
   return displayStr;
}

static char * SerialDataStopParityPara(uint8_t operationType, int16_t * paraValue )
{
   const char * parity;

   ParameterOperationsSettingsRollover( operationType, paraValue, EA_SERIAL_DATA_STOP_PARITY_VAL, D_SERIAL_DATA_STOP_PARITY_VAL, MIN_SERIAL_DATA_STOP_PARITY_VAL, MAX_SERIAL_DATA_STOP_PARITY_VAL );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch(( *paraValue & SERIAL_PARITY_MASK ) >> SERIAL_PARITY_SHIFT )
      {
         case SERIAL_PARITY_EVEN: parity = PSTR("EVEN");
                                  break;
         case SERIAL_PARITY_ODD:  parity = PSTR("ODD");
                                  break;
         case SERIAL_PARITY_NONE: 
         default:                 parity = PSTR("NONE");
                                  break;
      }
      locked_sprintf_P( displayStr, PSTR( "%d:%d:%S" ), ((*paraValue&SERIAL_DATA_BITS_MASK)>>SERIAL_DATA_BITS_SHIFT) + 5,
                                                 ((*paraValue&SERIAL_STOP_BIT_MASK)>>SERIAL_STOP_BIT_SHIFT)+1,
                                                 parity );
   }
   return displayStr;
}


static char * PresAbsRange(uint8_t operationType, int16_t * paraValue )
{
   if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   {
      *paraValue = OSReadEEPromWord((unsigned int *)EA_DP1_RANGE);
	  return FormatInteger( *paraValue );
   }
   else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
   {
         if( *paraValue < 0 )
            *paraValue = 0; // roll over
   }
   else if( operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = D_DP1_RANGE;
   }
   
   OSWriteEEPromWord((unsigned int *)EA_DP1_RANGE, *paraValue);
   
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

//static char * PresAbsSensMinCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP1_SENS_MIN, 8, 1, 50 );
//}
//
//static char * PresAbsSensMaxCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP1_SENS_MAX, 52, 51, 300);
//}
//
//static char * PresDiffSensMinCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP2_SENS_MIN, 8, 1, 50 );
//}
//
//static char * PresDiffSensMaxCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP2_SENS_MAX, 52, 51, 300);
//}
//
//static char * PresDiff3SensMinCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP3_SENS_MIN, 8, 1, 50 );
//}
//
//static char * PresDiff3SensMaxCnt(uint8_t operationType, int16_t * paraValue )
//{
	//return ParaSetIntegerVal( operationType, paraValue, EA_DP3_SENS_MAX, 52, 51, 300);
//}

static char * PresDiffRange(uint8_t operationType, int16_t * paraValue )
{
	if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
	{
		*paraValue = OSReadEEPromWord((unsigned int *)EA_DP2_RANGE);
		return FormatInteger( *paraValue );
	}
	else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
	{
		if( *paraValue < 0 )
		*paraValue = 0; // roll over
	}
	else if( operationType == (uint8_t)PARA_DEFAULT )
	{
		*paraValue = D_DP2_RANGE;
	}
	
	OSWriteEEPromWord((unsigned int *)EA_DP2_RANGE, *paraValue);
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatInteger( *paraValue );
}

static char * PresDiff3Range(uint8_t operationType, int16_t * paraValue )
{
	if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
	{
		*paraValue = OSReadEEPromWord((unsigned int *)EA_DP3_RANGE);
		return FormatInteger( *paraValue );
	}
	else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
	{
		if( *paraValue < 0 )
		*paraValue = 0; // roll over
	}
	else if( operationType == (uint8_t)PARA_DEFAULT )
	{
		*paraValue = D_DP3_RANGE;
	}
	
	OSWriteEEPromWord((unsigned int *)EA_DP3_RANGE, *paraValue);
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatInteger( *paraValue );
}

static char * TimeHourPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x3f, 0x0, operationType, paraValue, D_TIME_HOURS, MIN_TIME_HOURS, MAX_TIME_HOURS, RTC_TIMEHOUR_REG );
}

static char * TimeMinutePara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x7f, 0, operationType, paraValue, D_TIME_MINUTES, MIN_TIME_MINUTES, MAX_TIME_MINUTES, RTC_TIMEMIN_REG );
}

static char * TimeSecondPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x7f, 0, operationType, paraValue, D_TIME_SECONDS, MIN_TIME_SECONDS, MAX_TIME_SECONDS, RTC_TIMESEC_REG );
}

static char * DateYearPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0xff, 0x00, operationType, paraValue, D_DATE_YEAR, MIN_DATE_YEAR, MAX_DATE_YEAR, RTC_DATE_YEAR_REG );
}

static char * DateMonthPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x1f, 0x0, operationType, paraValue, D_DATE_MONTH, MIN_DATE_MONTH, MAX_DATE_MONTH, RTC_DATE_MONTH_REG );
}

static char * DateDayPara( uint8_t operationType, int16_t * paraValue )
{
   return ParameterOperationsTimeValues( 0x3f, 0x0, operationType, paraValue, D_DATE_DAY, MIN_DATE_DAY, MAX_DATE_DAY, RTC_DATE_DATE_REG );
}


static char * ParameterOperationsTimeValues( uint8_t mask, uint8_t maskWriteVal, uint8_t operationType, int16_t * paraValue, int16_t dfltValue, int8_t minValue, int8_t maxValue, uint8_t regAddr )
{
   uint8_t temp;

   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = dfltValue;
       if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
       {
          temp = temp & mask;
          *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
       }
   }
   else if( operationType == (uint8_t)PARA_READ )
   {
      *paraValue = dfltValue;
       if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
       {
          temp = temp & mask;
          *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
       }
   }
   else
   {
      if( operationType == (uint8_t)PARA_DEFAULT )
         *paraValue = dfltValue;

      if ( *paraValue < minValue)
         *paraValue = maxValue;   // Roll over
      if ( *paraValue > maxValue)
         *paraValue = minValue;   // Roll over

      temp = *paraValue;
      temp = ((temp / 10) << 4 ) + temp % 10;
      RTCWriteTimeDateRegister(regAddr, temp | maskWriteVal);
      *paraValue = dfltValue;
      if( RTCReadTimeDateRegister(regAddr, &temp ) == ERROR_OK )
      {
         temp = temp & mask;
         *paraValue = (temp & 0xf) + ( temp >> 4 ) * 10;
      }
   }
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}


static char * ConfigPasswordPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = configPassword;
   }
   else if( operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
      configPassword = *paraValue;
   else 
      *paraValue = configPassword;

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * CustomerIDPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CUSTOMER_ID);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
      OSWriteEEPromWord((unsigned int *)EA_CUSTOMER_ID, ~(*paraValue));
   else
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CUSTOMER_ID);

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

static char * SerialNoPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_SERIAL_NO);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
      OSWriteEEPromWord((unsigned int *)EA_SERIAL_NO, ~(*paraValue));
   else
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_SERIAL_NO);

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}


static char * SystemConfigPara(uint8_t operationType, int16_t * paraValue )
{
   if( operationType == (uint8_t)PARA_READ_VALUE_ONLY )
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM);
   }
   else if( IsConfigPassword() && (operationType == (uint8_t)PARA_WRITE || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY))
   {
      OSWriteEEPromWord((unsigned int *)EA_CONFIG_SYSTEM, ~(*paraValue));
   }
   else
   {
      *paraValue = ~OSReadEEPromWord((unsigned int *)EA_CONFIG_SYSTEM);
   }

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatInteger( *paraValue );
}

//static char * DisplayIntervalPara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_DISPLAY_INTERVAL, D_DISPLAY_INTERVAL, MIN_DISPLAY_INTERVAL, MAX_DISPLAY_INTERVAL );
//}
//
//static char * DisplayModePara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_DISPLAY_MODE, 1, 0, 1 );
//}

//static char * LogggingIntervalPara(uint8_t operationType, int16_t * paraValue )
//{
//   return ParaSetIntegerVal( operationType, paraValue, EA_LOGGING_INTERVAL, D_LOGGING_INTERVAL, MIN_LOGGING_INTERVAL, MAX_LOGGING_INTERVAL );
//}

static char * AHUIdPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_ID, 0, 0, 1000 );
}

static char * AHUCFMPara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_CFM, 0, 0, 32000 );
}

static char * AHUArea1Para(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_AREA1, 0, 0, 32000 );
}

static char * AHUArea2Para(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_AREA2, 0, 0, 32000 );
}

static char * AHUArea3Para(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal1( operationType, paraValue, EA_AHU_AREA3, 0, 0, 32000 );
}

static char * DeviceIdPara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_DEVICE_ID, D_DEVICE_ID, MIN_DEVICE_ID, MAX_DEVICE_ID );
}

static char * BroadcastIntervalPara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BROADCAST_INTERVAL, D_BROADCAST_INTERVAL, MIN_BROADCAST_INTERVAL, MAX_BROADCAST_INTERVAL );
}

static char * AbsPresZeroRangePara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_DP1_ZERO_RNG, D_DP1_ZERO_RNG, MIN_DP1_ZERO_RNG, MAX_DP1_ZERO_RNG );
}

static char * DiffPresZeroRangePara(uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_DP2_ZERO_RNG, D_DP2_ZERO_RNG, MIN_DP2_ZERO_RNG, MAX_DP2_ZERO_RNG );
}

static char * DiffPres3ZeroRangePara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal( operationType, paraValue, EA_DP3_ZERO_RNG, D_DP3_ZERO_RNG, MIN_DP3_ZERO_RNG, MAX_DP3_ZERO_RNG );
}
static char * AbsPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_DP1_ZERO_ADJ, D_DP1_ZERO_ADJ, MIN_DP1_ZERO_ADJ, MAX_DP1_ZERO_ADJ );
}

static char * DiffPresZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_DP2_ZERO_ADJ, D_DP2_ZERO_ADJ, MIN_DP2_ZERO_ADJ, MAX_DP2_ZERO_ADJ );
}

static char * DiffPres3ZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
	//if( operationType == PARA_DEFAULT )
	//return FormatInteger( *paraValue );
	return ParaSetIntegerVal( operationType, paraValue, EA_DP3_ZERO_ADJ, D_DP3_ZERO_ADJ, MIN_DP3_ZERO_ADJ, MAX_DP3_ZERO_ADJ );
}

static char * HumidityZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_RH_ZERO_ADJ, D_RH_ZERO_ADJ, MIN_RH_ZERO_ADJ, MAX_RH_ZERO_ADJ );
}

static char * Humidity2ZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
	//if( operationType == PARA_DEFAULT )
	//return FormatInteger( *paraValue );
	return ParaSetIntegerVal( operationType, paraValue, EA_RH2_ZERO_ADJ, D_RH_ZERO_ADJ, MIN_RH_ZERO_ADJ, MAX_RH_ZERO_ADJ );
}

static char * TempZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
   //if( operationType == PARA_DEFAULT )
      //return FormatInteger( *paraValue );
   return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_ZERO_ADJ, D_TEMP_ZERO_ADJ, MIN_TEMP_ZERO_ADJ, MAX_TEMP_ZERO_ADJ );
}

static char * Temp2ZeroAdjustPara(uint8_t operationType, int16_t * paraValue )
{
	//if( operationType == PARA_DEFAULT )
	//return FormatInteger( *paraValue );
	return ParaSetIntegerVal( operationType, paraValue, EA_TEMP2_ZERO_ADJ, D_TEMP_ZERO_ADJ, MIN_TEMP_ZERO_ADJ, MAX_TEMP_ZERO_ADJ );
}

static char * TempFireAlmSetPara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsOnHighSettings(operationType,
	paraValue,
	5000,
	10000,
	TEMP_UPPER_ALARM_ON_LIMIT,
	EA_TEMP_FIRE_ALM_SET );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempFireAlmTimePara(uint8_t operationType, int16_t * paraValue )
{
	return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_FIRE_TIME_SET, 60, 15, 3600 );
}

static char * VersionPara(uint8_t operationType, int16_t * paraValue )
{
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
   {
      *paraValue = SOFTWARE_VERSION;
      return NULL;
   }
   return FormatVersion();
}

static char * BuzzerOnPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_ON_TIME, D_BUZZER_ON_TIME, MIN_BUZZER_ON_TIME, MAX_BUZZER_ON_TIME );
}


static char * BuzzerOffPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_OFF_TIME, D_BUZZER_OFF_TIME, MIN_BUZZER_OFF_TIME, MAX_BUZZER_OFF_TIME );
}

static char * BuzzerDisablePara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal( operationType, paraValue, EA_BUZZER_DISABLE_TIME, D_BUZZER_DISABLE_TIME, MIN_BUZZER_DISABLE_TIME, MAX_BUZZER_DISABLE_TIME );
}

static char * PresAbsOutputTypePara( uint8_t operationType, int16_t * paraValue )
{
    return ParaOutputType( EA_DP1_OUTPUT_TYPE, operationType, paraValue );
}

static char * PresDiffOutputTypePara( uint8_t operationType, int16_t * paraValue )
{
    return ParaOutputType( EA_DP2_OUTPUT_TYPE, operationType, paraValue );
}

static char * TempOutputTypePara( uint8_t operationType, int16_t * paraValue )
{
    return ParaOutputType( EA_TEMP_OUTPUT_TYPE, operationType, paraValue );
}

static char * HumidityOutputTypePara( uint8_t operationType, int16_t * paraValue )
{
    return ParaOutputType( EA_RH_OUTPUT_TYPE, operationType, paraValue );
}

static char * PresAbsPIDSetpointPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP1_PID_SET_VALUE,
                               D_DP1_PID_SET_VALUE, 
                               MIN_DP1_PID_SET_VALUE, 
                               MAX_DP1_PID_SET_VALUE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatDP1OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * PresDiffPIDSetpointPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP2_PID_SET_VALUE,
                               D_DP2_PID_SET_VALUE, 
                               MIN_DP2_PID_SET_VALUE, 
                               MAX_DP2_PID_SET_VALUE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatDP2OneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempPIDSetpointPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_TEMP_PID_SET_VALUE,
                               D_TEMP_PID_SET_VALUE, 
                               MIN_TEMP_PID_SET_VALUE, 
                               MAX_TEMP_PID_SET_VALUE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * HumidityPIDSetpointPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_RH_PID_SET_VALUE,
                               D_RH_PID_SET_VALUE, 
                               MIN_RH_PID_SET_VALUE, 
                               MAX_RH_PID_SET_VALUE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;

   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}





static char * PresAbsPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP1_PID_STARTUP_PERCENT,
                               D_DP1_PID_STARTUP_PERCENT, 
                               MIN_DP1_PID_STARTUP_PERCENT, 
                               MAX_DP1_PID_STARTUP_PERCENT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * PresAbsPIDKpPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal(operationType, 
                            paraValue, 
                            EA_DP1_PID_KP,
                            D_DP1_PID_KP, 
                            MIN_DP1_PID_KP, 
                            MAX_DP1_PID_KP );
}

static char * PresAbsPIDTiPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP1_PID_TI,
                               D_DP1_PID_TI, 
                               MIN_DP1_PID_TI, 
                               MAX_DP1_PID_TI );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * PresAbsPIDTdPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP1_PID_TD,
                               D_DP1_PID_TD, 
                               MIN_DP1_PID_TD, 
                               MAX_DP1_PID_TD );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 100, *paraValue % 100 );
   return displayStr;
}


static char * PresDiffPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP2_PID_STARTUP_PERCENT,
                               D_DP2_PID_STARTUP_PERCENT, 
                               MIN_DP2_PID_STARTUP_PERCENT, 
                               MAX_DP2_PID_STARTUP_PERCENT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * PresDiffPIDKpPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal(operationType, 
                            paraValue, 
                            EA_DP2_PID_KP,
                            D_DP2_PID_KP, 
                            MIN_DP2_PID_KP, 
                            MAX_DP2_PID_KP );
}

static char * PresDiffPIDTiPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP2_PID_TI,
                               D_DP2_PID_TI, 
                               MIN_DP2_PID_TI, 
                               MAX_DP2_PID_TI );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * PresDiffPIDTdPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_DP2_PID_TD,
                               D_DP2_PID_TD, 
                               MIN_DP2_PID_TD, 
                               MAX_DP2_PID_TD );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 100, *paraValue % 100 );
   return displayStr;
}


static char * TempPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_TEMP_PID_STARTUP_PERCENT,
                               D_TEMP_PID_STARTUP_PERCENT, 
                               MIN_TEMP_PID_STARTUP_PERCENT, 
                               MAX_TEMP_PID_STARTUP_PERCENT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * TempPIDKpPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal(operationType, 
                            paraValue, 
                            EA_TEMP_PID_KP,
                            D_TEMP_PID_KP, 
                            MIN_TEMP_PID_KP, 
                            MAX_TEMP_PID_KP );
}

static char * TempPIDTiPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_TEMP_PID_TI,
                               D_TEMP_PID_TI, 
                               MIN_TEMP_PID_TI, 
                               MAX_TEMP_PID_TI );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * TempPIDTdPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_TEMP_PID_TD,
                               D_TEMP_PID_TD, 
                               MIN_TEMP_PID_TD, 
                               MAX_TEMP_PID_TD );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 100, *paraValue % 100 );
   return displayStr;
}


static char * HumidityPIDStartupPercentPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_RH_PID_STARTUP_PERCENT,
                               D_RH_PID_STARTUP_PERCENT, 
                               MIN_RH_PID_STARTUP_PERCENT, 
                               MAX_RH_PID_STARTUP_PERCENT );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * HumidityPIDKpPara( uint8_t operationType, int16_t * paraValue )
{
   return ParaSetIntegerVal(operationType, 
                            paraValue, 
                            EA_RH_PID_KP,
                            D_RH_PID_KP, 
                            MIN_RH_PID_KP, 
                            MAX_RH_PID_KP );
}

static char * HumidityPIDTiPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_RH_PID_TI,
                               D_RH_PID_TI, 
                               MIN_RH_PID_TI, 
                               MAX_RH_PID_TI );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 10, *paraValue % 10 );
   return displayStr;
}

static char * HumidityPIDTdPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings(operationType, 
                               paraValue, 
                               EA_RH_PID_TD,
                               D_RH_PID_TD, 
                               MIN_RH_PID_TD, 
                               MAX_RH_PID_TD );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   sprintf( displayStr, "%d.%d", *paraValue / 100, *paraValue % 100 );
   return displayStr;
}


static char * TempRHSensTypePara( uint8_t operationType, int16_t * paraValue )
{
   //ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_TEMP_RH_SENS_TYPE, 0, 3 );
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMP_RH_SENS_TYPE, TEMP_RH_SENS_SHT25, 0, 2 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
       switch( *paraValue )
       {
          case TEMP_RH_SENS_SHT25: locked_sprintf_P( displayStr, PSTR("SHT25"));
                                   break;
          //case TEMP_RH_SENS_SHT35: locked_sprintf_P( displayStr, PSTR("SHT35"));
                                   //break;
          case TEMP_RH_SENS_IDT_HS3100: 
                                   locked_sprintf_P( displayStr, PSTR("IDT3100"));
                                   break;
          case TEMP_RH_SENS_NONE: 
          default:                 locked_sprintf_P( displayStr, PSTR("NONE"));
                                   break;
       }
   }
   return displayStr;
}

static char * TempRH2SensTypePara( uint8_t operationType, int16_t * paraValue )
{
	//ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_TEMP_RH_SENS_TYPE, 0, 3 );
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMP_RH2_SENS_TYPE, TEMP_RH_SENS_SHT25, 0, 2 );

	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case TEMP_RH_SENS_SHT25: locked_sprintf_P( displayStr, PSTR("SHT25"));
			break;
			//case TEMP_RH_SENS_SHT35: locked_sprintf_P( displayStr, PSTR("SHT35"));
			//break;
			case TEMP_RH_SENS_IDT_HS3100:
			locked_sprintf_P( displayStr, PSTR("IDT3100"));
			break;
			case TEMP_RH_SENS_NONE:
			default:                 locked_sprintf_P( displayStr, PSTR("NONE"));
			break;
		}
	}
	return displayStr;
}

//static char * ParticalSizeTypePara(uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_PARTICAL_SIZE_TYPE, 0, 3 );
//
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//if (IsLCDDisplay())
	//{
		//switch( *paraValue )
		//{
			//case PARTICAL_SIZE_1_0:    locked_sprintf_P( displayStr, PSTR("PM 1.0"));
			//break;
			//case PARTICAL_SIZE_2_5:  locked_sprintf_P( displayStr, PSTR("PM 2.5"));
			//break;
			//case PARTICAL_SIZE_4_0:  locked_sprintf_P( displayStr, PSTR("PM 4.0"));
			//break;
			//case PARTICAL_SIZE_10_0:
			//default:                  locked_sprintf_P( displayStr, PSTR("PM 10.0"));
			//break;
		//}
	//}
	//return displayStr;
//}
//
//static char * ParticalSensTypePara(uint8_t operationType, int16_t * paraValue )
//{
	//ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_PARTICAL_SENS_TYPE, 0, 3 );
//
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//if (IsLCDDisplay())
	//{
		//switch( *paraValue )
		//{
			//case PARTICAL_SENS_SEN55:    locked_sprintf_P( displayStr, PSTR("SEN55"));
			//break;
			//case PARTICAL_SENS_ISP7100:  locked_sprintf_P( displayStr, PSTR("ISP7100"));
			//break;
			//case PARTICAL_SENS_ISP5100:  locked_sprintf_P( displayStr, PSTR("ISP5100"));
			//break;
			//case PARTICAL_SENS_NONE:
			//default:                  locked_sprintf_P( displayStr, PSTR("NONE"));
			//break;
		//}
	//}
	//return displayStr;
//}
//

//static char * PresAbsSensTypePara(uint8_t operationType, int16_t * paraValue )
//{
   ////ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_DP1_SENS_TYPE, 0, 3 );
	//ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP1_SENS_TYPE, PRES_SENS_SM9543, 0, 3 );
	//
   //if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      //return NULL;
   //if (IsLCDDisplay())
   //{
       //switch( *paraValue )
       //{                  
          //case PRES_SENS_SM4331:    locked_sprintf_P( displayStr, PSTR("SM4331"));          
                                    //break;                                               
          //case PRES_SENS_SM9543:    locked_sprintf_P( displayStr, PSTR("SM9543"));
                                    //break;
          //case PRES_SENS_AMS5812:   locked_sprintf_P( displayStr, PSTR("AMS5812"));
                                    //break;
          //case PRES_SENS_NONE: 
          //default:                  locked_sprintf_P( displayStr, PSTR("NONE"));
                                    //break;
       //}
   //}
   //return displayStr;
//}
//
//static char * PresDiffSensTypePara(uint8_t operationType, int16_t * paraValue )
//{
    ////ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_DP2_SENS_TYPE, 0, 3 );
	//ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP2_SENS_TYPE, PRES_SENS_SM9543, 0, 3 );
	//
    //if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
       //return NULL;
    //if (IsLCDDisplay())
    //{
        //switch( *paraValue )
        //{                                               
           //case PRES_SENS_SM4331:    locked_sprintf_P( displayStr, PSTR("SM4331"));          
                                     //break;                                               
           //case PRES_SENS_SM9543:    locked_sprintf_P( displayStr, PSTR("SM9543"));
                                     //break;
           //case PRES_SENS_AMS5812:   locked_sprintf_P( displayStr, PSTR("AMS5812"));
                                     //break;
           //case PRES_SENS_NONE: 
           //default:                  locked_sprintf_P( displayStr, PSTR("NONE"));
                                     //break;
        //}
    //}
    //return displayStr;
//}
//
//static char * PresDiff3SensTypePara(uint8_t operationType, int16_t * paraValue )
//{
	////ParameterOperationsSettingsRolloverNoDefault( operationType, paraValue, EA_DP3_SENS_TYPE, 0, 3 );
	//ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP3_SENS_TYPE, PRES_SENS_SM9543, 0, 3 );
	//
	//if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	//return NULL;
	//if (IsLCDDisplay())
	//{
		//switch( *paraValue )
		//{
			//case PRES_SENS_SM4331:    locked_sprintf_P( displayStr, PSTR("SM4331"));
			//break;
			//case PRES_SENS_SM9543:    locked_sprintf_P( displayStr, PSTR("SM9543"));
			//break;
			//case PRES_SENS_AMS5812:   locked_sprintf_P( displayStr, PSTR("AMS5812"));
			//break;
			//case PRES_SENS_NONE:
			//default:                  locked_sprintf_P( displayStr, PSTR("NONE"));
			//break;
		//}
	//}
	//return displayStr;
//}

static char * DP1AreaTypePara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP1_AREA_TYPE, PRES_AREA_SQUARE, 0, 1 );
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case PRES_AREA_SQUARE:    locked_sprintf_P( displayStr, PSTR("SQUARE"));
			break;
			case PRES_AREA_ROUND:     locked_sprintf_P( displayStr, PSTR("ROUND"));
			break;
			default:                  locked_sprintf_P( displayStr, PSTR("UNKNOWN"));
			break;
		}
	}
	return displayStr;
}

static char * DP2AreaTypePara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP2_AREA_TYPE, PRES_AREA_SQUARE, 0, 1 );
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case PRES_AREA_SQUARE:    locked_sprintf_P( displayStr, PSTR("SQUARE"));
			break;
			case PRES_AREA_ROUND:     locked_sprintf_P( displayStr, PSTR("ROUND"));
			break;
			default:                  locked_sprintf_P( displayStr, PSTR("UNKNOWN"));
			break;
		}
	}
	return displayStr;
}

static char * DP3AreaTypePara(uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_DP1_AREA_TYPE, PRES_AREA_SQUARE, 0, 1 );
	
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case PRES_AREA_SQUARE:    locked_sprintf_P( displayStr, PSTR("SQUARE"));
			break;
			case PRES_AREA_ROUND:     locked_sprintf_P( displayStr, PSTR("ROUND"));
			break;
			default:                  locked_sprintf_P( displayStr, PSTR("UNKNOWN"));
			break;
		}
	}
	return displayStr;
}

//static char * ParticalAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
//{
	//return AlarmSetupPara( operationType, paraValue, EA_PARTICAL_ALARM_SETUP );
//}

static char * PresAbsAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_DP1_ALARM_SETUP );
}

static char * PresDiffAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_DP2_ALARM_SETUP );
}

static char * PresDiff3AlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
	return AlarmSetupPara( operationType, paraValue, EA_DP3_ALARM_SETUP );
}

static char * TempAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_TEMP_ALARM_SETUP );
}

static char * HumidityAlarmSetupPara( uint8_t operationType, int16_t * paraValue )
{
   return AlarmSetupPara( operationType, paraValue, EA_RH_ALARM_SETUP );
}

static char * RTCSetupPara( uint8_t operationType, int16_t * paraValue )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, EA_RTC_SETUP, RTC_ENABLED, 0, 1 );

	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case RTC_DISABLED: locked_sprintf_P( displayStr, PSTR("DISABLED"));
			break;
			case RTC_ENABLED: locked_sprintf_P( displayStr, PSTR("ENABLED"));
			break;
			default:          locked_sprintf_P( displayStr, PSTR("UNKOWN"));
			break;
		}
	}
	return displayStr;
}

static char * AlarmSetupPara( uint8_t operationType, int16_t * paraValue, unsigned int eeAddress )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eeAddress, ALARM_ENB_BUZ_ON, 0, 2 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      switch( *paraValue )
      {
         case ALARM_DISABLED: locked_sprintf_P( displayStr, PSTR("DISABLED"));
                                  break;
         case ALARM_ENB_BUZ_OFF: locked_sprintf_P( displayStr, PSTR("ENB_BUZ_OFF"));
                                  break;
         case ALARM_ENB_BUZ_ON:  locked_sprintf_P( displayStr, PSTR("ENB_BUZ_ON"));
                                  break;
         default:                 locked_sprintf_P( displayStr, PSTR("UNKOWN"));
                                  break;
      }
   }
   return displayStr;

}

static char * PresAbsOutPolarityPara( uint8_t operationType, int16_t * paraValue )
{
   return SetOutPolarityPara( operationType, paraValue, EA_DP1_OUT_POLARITY, D_DP1_OUT_POLARITY );
}

static char * PresDiffOutPolarityPara( uint8_t operationType, int16_t * paraValue )
{
   return SetOutPolarityPara( operationType, paraValue, EA_DP2_OUT_POLARITY, D_DP2_OUT_POLARITY );
}

static char * TempOutPolarityPara( uint8_t operationType, int16_t * paraValue )
{
   return SetOutPolarityPara( operationType, paraValue, EA_TEMP_OUT_POLARITY, D_TEMP_OUT_POLARITY );
}

static char * HumidityOutPolarityPara( uint8_t operationType, int16_t * paraValue )
{
   return SetOutPolarityPara( operationType, paraValue, EA_RH_OUT_POLARITY, D_RH_OUT_POLARITY );
}


static char * TempHighPIDRHAdjustPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings( operationType, paraValue, EA_TEMP_HIGH_PID_RH_ADJUST, D_TEMP_HIGH_PID_RH_ADJUST, MIN_TEMP_HIGH_PID_RH_ADJUST, MAX_TEMP_HIGH_PID_RH_ADJUST );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}

static char * TempLowPIDRHAdjustPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings( operationType, paraValue, EA_TEMP_LOW_PID_RH_ADJUST, D_TEMP_LOW_PID_RH_ADJUST, MIN_TEMP_LOW_PID_RH_ADJUST, MAX_TEMP_LOW_PID_RH_ADJUST );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatTemperatureOneLine( displayStr, ERROR_OK, *paraValue );
}


static char * TempHighPIDRHCascadeFactorPara( uint8_t operationType, int16_t * paraValue )
{
   double result;

   ParameterOperationsSettings( operationType, paraValue, EA_TEMP_HIGH_PID_RH_CASCADE_FACTOR, D_TEMP_HIGH_PID_RH_CASCADE_FACTOR, MIN_TEMP_HIGH_PID_RH_CASCADE_FACTOR, MAX_TEMP_HIGH_PID_RH_CASCADE_FACTOR );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {  
	  result = (*paraValue) / 100.0;
	  dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
	  strcpy( formatedValue, strtrim(formatedValue));
	  sprintf(displayStr, formatedValue);
   }
   return displayStr;
}

static char * TempLowPIDRHCascadeFactorPara( uint8_t operationType, int16_t * paraValue )
{
   double result;

   ParameterOperationsSettings( operationType, paraValue, EA_TEMP_LOW_PID_RH_CASCADE_FACTOR, D_TEMP_LOW_PID_RH_CASCADE_FACTOR, MIN_TEMP_LOW_PID_RH_CASCADE_FACTOR, MAX_TEMP_LOW_PID_RH_CASCADE_FACTOR );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      result = (*paraValue) / 100.0;
      dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
      sprintf(displayStr, formatedValue);
   }
   return displayStr;
}



static char * HumidityHighPIDTempAdjustPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings( operationType, paraValue, EA_RH_HIGH_PID_TEMP_ADJUST, D_RH_HIGH_PID_TEMP_ADJUST, MIN_RH_HIGH_PID_TEMP_ADJUST, MAX_RH_HIGH_PID_TEMP_ADJUST );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}


static char * HumidityLowPIDTempAdjustPara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettings( operationType, paraValue, EA_RH_LOW_PID_TEMP_ADJUST, D_RH_LOW_PID_TEMP_ADJUST, MIN_RH_LOW_PID_TEMP_ADJUST, MAX_RH_LOW_PID_TEMP_ADJUST );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   return FormatHumidityOneLine( displayStr, ERROR_OK, *paraValue );
}


static char * HumidityHighPIDTempCascadeFactorPara( uint8_t operationType, int16_t * paraValue )
{
   double result;

   ParameterOperationsSettings( operationType, paraValue, EA_RH_HIGH_PID_TEMP_CASCADE_FACTOR, D_RH_HIGH_PID_TEMP_CASCADE_FACTOR, MIN_RH_HIGH_PID_TEMP_CASCADE_FACTOR, MAX_RH_HIGH_PID_TEMP_CASCADE_FACTOR );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      result = (*paraValue) / 100.0;
      dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
      sprintf(displayStr, formatedValue);
   }
   return displayStr;
}


static char * HumidityLowPIDTempCascadeFactorPara( uint8_t operationType, int16_t * paraValue )
{
   double result;

   ParameterOperationsSettings( operationType, paraValue, EA_RH_LOW_PID_TEMP_CASCADE_FACTOR, D_RH_LOW_PID_TEMP_CASCADE_FACTOR, MIN_RH_LOW_PID_TEMP_CASCADE_FACTOR, MAX_RH_LOW_PID_TEMP_CASCADE_FACTOR );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      result = (*paraValue) / 100.0;
      dtostrf( result, sizeof(formatedValue)-1, 2, formatedValue);
      strcpy( formatedValue, strtrim(formatedValue));
      sprintf(displayStr, formatedValue);
   }
   return displayStr;
}


static char * TempHighPIDRHCascadeStatePara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMP_HIGH_PID_CASCADE_RH_STATE, D_TEMP_HIGH_PID_CASCADE_RH_STATE, MIN_TEMP_HIGH_PID_CASCADE_RH_STATE, MAX_TEMP_HIGH_PID_CASCADE_RH_STATE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if ((*paraValue) < 0 )
      locked_sprintf_P( displayStr, PSTR("LOW"));
   else if ((*paraValue) > 0 )
      locked_sprintf_P( displayStr, PSTR("HI"));
   else if ((*paraValue) == 0 )
      locked_sprintf_P( displayStr, PSTR("ANY"));

   return displayStr;
}


static char * TempLowPIDRHCascadeStatePara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_TEMP_LOW_PID_CASCADE_RH_STATE, D_TEMP_LOW_PID_CASCADE_RH_STATE, MIN_TEMP_LOW_PID_CASCADE_RH_STATE, MAX_TEMP_LOW_PID_CASCADE_RH_STATE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if ((*paraValue) < 0 )
      locked_sprintf_P( displayStr, PSTR("LOW"));
   else if ((*paraValue) > 0 )
      locked_sprintf_P( displayStr, PSTR("HI"));
   else if ((*paraValue) == 0 )
      locked_sprintf_P( displayStr, PSTR("ANY"));

   return displayStr;
}


static char * HumidityHighPIDTempCascadeStatePara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_RH_HIGH_PID_CASCADE_TEMP_STATE, D_RH_HIGH_PID_CASCADE_TEMP_STATE, MIN_RH_HIGH_PID_CASCADE_TEMP_STATE, MAX_RH_HIGH_PID_CASCADE_TEMP_STATE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if ((*paraValue) < 0 )
      locked_sprintf_P( displayStr, PSTR("LOW"));
   else if ((*paraValue) > 0 )
      locked_sprintf_P( displayStr, PSTR("HI"));
   else if ((*paraValue) == 0 )
      locked_sprintf_P( displayStr, PSTR("ANY"));

   return displayStr;
}


static char * HumidityLowPIDTempCascadeStatePara( uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, EA_RH_LOW_PID_CASCADE_TEMP_STATE, D_RH_LOW_PID_CASCADE_TEMP_STATE, MIN_RH_LOW_PID_CASCADE_TEMP_STATE, MAX_RH_LOW_PID_CASCADE_TEMP_STATE );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if ((*paraValue) < 0 )
      locked_sprintf_P( displayStr, PSTR("LOW"));
   else if ((*paraValue) > 0 )
      locked_sprintf_P( displayStr, PSTR("HI"));
   else if ((*paraValue) == 0 )
      locked_sprintf_P( displayStr, PSTR("ANY"));

   return displayStr;
}


static char * SetOutPolarityPara( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eepromAddr, dfltVal, 0, 1 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue )
         locked_sprintf_P( displayStr, PSTR("NORMAL"));
      else
         locked_sprintf_P( displayStr, PSTR("REVERSE"));
   }
   return displayStr;
   
}


static char * Output1MappingPara( uint8_t operationType, int16_t * paraValue )
{
   return OutputMappingPara1( operationType, paraValue, OUTPUT1_NORMAL, EA_OUTPUT1_MAPPING );
}

static char * Output2MappingPara( uint8_t operationType, int16_t * paraValue )
{
   return OutputMappingPara( operationType, paraValue, OUTPUT_INPUT2, EA_OUTPUT2_MAPPING );
}

static char * Output3MappingPara( uint8_t operationType, int16_t * paraValue )
{
   return OutputMappingPara( operationType, paraValue, OUTPUT_INPUT3, EA_OUTPUT3_MAPPING );
}

static char * Output4MappingPara( uint8_t operationType, int16_t * paraValue )
{
   return OutputMappingPara( operationType, paraValue, OUTPUT_INPUT4, EA_OUTPUT4_MAPPING );
}


static char * OutputMappingPara( uint8_t operationType, int16_t * paraValue, int16_t dfltVal, unsigned int eeAddress )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eeAddress, dfltVal, 0, OUTPUT_LAST_NO );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
      return NULL;
   if (IsLCDDisplay())
   {
      switch( *paraValue )
      {
         case OUTPUT_COMPUTER: locked_sprintf_P( displayStr, PSTR("COMPUTER"));
                               break;
         case OUTPUT_INPUT1:   locked_sprintf_P( displayStr, PSTR("INPUT1"));
                               break;
         case OUTPUT_INPUT1REV:locked_sprintf_P( displayStr, PSTR("INPUT1REV"));
                               break;
         case OUTPUT_INPUT2:   locked_sprintf_P( displayStr, PSTR("INPUT2"));
                               break;
         case OUTPUT_INPUT2REV:locked_sprintf_P( displayStr, PSTR("INPUT2REV"));
                               break;
         case OUTPUT_INPUT3:   locked_sprintf_P( displayStr, PSTR("INPUT3"));
                               break;
         case OUTPUT_INPUT3REV:locked_sprintf_P( displayStr, PSTR("INPUT3REV"));
                               break;
         case OUTPUT_INPUT4:   locked_sprintf_P( displayStr, PSTR("INPUT4"));
                               break;
         case OUTPUT_INPUT4REV:locked_sprintf_P( displayStr, PSTR("INPUT4REV"));
                               break;
		 case OUTPUT_INPUT5:   locked_sprintf_P( displayStr, PSTR("INPUT5"));
							   break;
		 case OUTPUT_INPUT5REV:locked_sprintf_P( displayStr, PSTR("INPUT5REV"));
							   break;
		 //case OUTPUT_DOOR_INPUT:   locked_sprintf_P( displayStr, PSTR("DOOR OPEN"));
							   //break;
		 //case OUTPUT_DOOR_INPUT_REV:locked_sprintf_P( displayStr, PSTR("DOOR CLOSE"));
							   //break;
         default:              locked_sprintf_P( displayStr, PSTR("UNKOWN"));
                               break;
      }
   }
   return displayStr;
}

static char * OutputMappingPara1( uint8_t operationType, int16_t * paraValue, int16_t dfltVal, unsigned int eeAddress )
{
	ParameterOperationsSettingsRollover( operationType, paraValue, eeAddress, dfltVal, 0, OUTPUT1_LAST_NO );

	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY)
	return NULL;
	if (IsLCDDisplay())
	{
		switch( *paraValue )
		{
			case OUTPUT1_NORMAL: locked_sprintf_P( displayStr, PSTR("NORMAL"));
			break;
			case OUTPUT1_REVERSE:   locked_sprintf_P( displayStr, PSTR("REVERSE"));
			break;
			default:              locked_sprintf_P( displayStr, PSTR("UNKOWN"));
			break;
		}
	}
	return displayStr;
}

//static char * TempRHScanTimePara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_RH_SCAN_TIME, 5, 1, 30 );
//}
//
//
//static char * TempRHAverageCntPara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_TEMP_RH_READING_AVERAGE, 10, 1, 20 );
//}
//
//
//static char * PresAverageCntPara(uint8_t operationType, int16_t * paraValue )
//{
   //return ParaSetIntegerVal( operationType, paraValue, EA_PRES_READING_AVERAGE, 5, 1, 30 );
//}

 
static void ParameterOperationsSettings( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   {
      *paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	  return;
   }
   else if( operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltVal;
   }
   else
   {
      if( *paraValue < minVal )
      *paraValue = minVal;
      else if ( *paraValue > maxVal )
      *paraValue = maxVal;
   }
   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
}


static void ParameterOperationsSettingsRollover( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
   {
      *paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	  return;
   }
   else if( operationType == (uint8_t)PARA_DEFAULT )
   {
	   *paraValue = dfltVal;
   }
   else
   {
      if( *paraValue < minVal )
         *paraValue = maxVal; 
      else if ( *paraValue > maxVal )
         *paraValue = minVal;
   }
   OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
}

//static void ParameterOperationsSettingsRolloverNoDefault( uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t minVal, int16_t maxVal )
//{
	//if(( operationType == (uint8_t)PARA_READ_VALUE_ONLY ) || ( operationType == (uint8_t)PARA_READ ))
	//{
		//*paraValue = OSReadEEPromWord((unsigned int *)eepromAddr);
	//}
	//else
	//{
		//if( *paraValue < minVal )
		//*paraValue = maxVal;
		//else if ( *paraValue > maxVal )
		//*paraValue = minVal;
//
		//OSWriteEEPromWord((unsigned int *)eepromAddr, *paraValue);
	//}
//}

static char * ParaOutputType( unsigned int eepromAddr, uint8_t operationType, int16_t * paraValue )
{
   ParameterOperationsSettingsRollover( operationType, paraValue, eepromAddr, 1, 0, 2 );

   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
   if (IsLCDDisplay())
   {
      if( *paraValue == 0 )
         locked_sprintf_P( displayStr, PSTR("NONE"));
      else if( *paraValue == 1 )
         locked_sprintf_P( displayStr, PSTR("SENS"));
      else if( *paraValue == 2 )
         locked_sprintf_P( displayStr, PSTR("PID"));
      else
         locked_sprintf_P( displayStr, PSTR("Unknown"));
   }
   return displayStr;
}


static char * ParaSetIntegerVal(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
   ParameterOperationsSettings( operationType, paraValue, eepromAddr, dfltVal, minVal, maxVal );
   if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
      return NULL;
	  
   if( operationType == (uint8_t)PARA_DEFAULT )
	*paraValue = dfltVal;
	
   return FormatInteger(*paraValue);
}

static char * ParaSetIntegerVal1(uint8_t operationType, int16_t * paraValue, unsigned int eepromAddr, int16_t dfltVal, int16_t minVal, int16_t maxVal )
{
	ParameterOperationsSettings( operationType, paraValue, eepromAddr, dfltVal, minVal, maxVal );
	if(operationType == (uint8_t)PARA_READ_VALUE_ONLY || operationType == (uint8_t)PARA_WRITE_VALUE_ONLY )
	return NULL;
	
	if( operationType == (uint8_t)PARA_DEFAULT )
	*paraValue = dfltVal;
	
	if (IsLCDDisplay())
	{
		if( *paraValue == 0 )
			locked_sprintf_P( displayStr, PSTR("NONE"));
		else
			FormatInteger(*paraValue);
	}

	return displayStr;
}

static uint8_t IsMasterReset()
{
   return (SETDFLTSW_IN == 0)? (uint8_t)1 : (uint8_t)0;
}


static uint8_t IsConfigPassword()
{
   if( configPassword == MASTER_CONFIG_PASSWORD )
      return 1;
   
   // check zero or 0xffff
   if( GetParameterValue( CONFIG_SYSTEM ) == 0 && configPassword == CONFIG_PASSWORD )
      return 1;
   if( GetParameterValue( CONFIG_SYSTEM ) == -1 && configPassword == CONFIG_PASSWORD )
      return 1;
   return  0;
}


static uint8_t NotConfigPassword()
{
   if( configPassword == MASTER_CONFIG_PASSWORD )
      return 0;
   
   return (configPassword != CONFIG_PASSWORD)? 1 : 0;
}


void UIResourceLock()
{
   OSSemaTakeEver(UIMutex);
}

void UIResourceUnlock()
{
   OSSemaGive(UIMutex);
}


void BuzzerOn()
{
   BUZZER_ON;
}

void BuzzerOff()
{
   BUZZER_OFF;
}

int d_pres_abs_upper_alarm_off_limit()
{ 
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 14.9) / GetParameterValue(DP1_RANGE)));
	//return ((DP1_SENSOR_RESOLUTION_SM9543/2) + 245);
}
int d_pres_abs_upper_alarm_on_limit()       
{  
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 15.1) / GetParameterValue(DP1_RANGE)));
	//return ((DP1_SENSOR_RESOLUTION_SM9543/2) + 262);
}
             
int d_pres_abs_lower_alarm_off_limit()      
{  
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 5.1) / GetParameterValue(DP1_RANGE)));
	//return ((DP1_SENSOR_RESOLUTION_SM9543/2) + 82);
}

int d_pres_abs_lower_alarm_on_limit()       
{  
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 4.9) / GetParameterValue(DP1_RANGE)));
	//return ((DP1_SENSOR_RESOLUTION_SM9543/2) + 65);
}
             
int d_pres_diff_upper_alarm_off_limit()     
{  
   return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 14.9) / GetParameterValue(DP2_RANGE)));
   //return ((DP2_SENSOR_RESOLUTION_SM9543/2) + 245);
}
             
int d_pres_diff_upper_alarm_on_limit()      
{  
   return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 15.1) / GetParameterValue(DP2_RANGE)));
   //return ((DP2_SENSOR_RESOLUTION_SM9543/2) + 262);
}
             
int d_pres_diff_lower_alarm_off_limit()     
{  
   return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 5.1) / GetParameterValue(DP2_RANGE)));
   //return ((DP2_SENSOR_RESOLUTION_SM9543/2) + 82);
}
            
int d_pres_diff_lower_alarm_on_limit()      
{  
   return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 4.9) / GetParameterValue(DP2_RANGE)));
   //return ((DP2_SENSOR_RESOLUTION_SM9543/2) + 65);
}

int d_pres_diff3_upper_alarm_off_limit()
{
	return ((DP3_SENSOR_RESOLUTION_SM9543/2) + ((DP3_SENSOR_RESOLUTION_SM9543 * 14.9) / GetParameterValue(DP3_RANGE)));
	//return ((DP3_SENSOR_RESOLUTION_SM9543/2) + 245);
}

int d_pres_diff3_upper_alarm_on_limit()
{
	return ((DP3_SENSOR_RESOLUTION_SM9543/2) + ((DP3_SENSOR_RESOLUTION_SM9543 * 15.1) / GetParameterValue(DP3_RANGE)));
	//return ((DP3_SENSOR_RESOLUTION_SM9543/2) + 262);
}

int d_pres_diff3_lower_alarm_off_limit()
{
	return ((DP3_SENSOR_RESOLUTION_SM9543/2) + ((DP3_SENSOR_RESOLUTION_SM9543 * 5.1) / GetParameterValue(DP3_RANGE)));
	//return ((DP3_SENSOR_RESOLUTION_SM9543/2) + 82);
}

int d_pres_diff3_lower_alarm_on_limit()
{
	return ((DP3_SENSOR_RESOLUTION_SM9543/2) + ((DP3_SENSOR_RESOLUTION_SM9543 * 4.9) / GetParameterValue(DP3_RANGE)));
	//return ((DP3_SENSOR_RESOLUTION_SM9543/2) + 65);
}

int d_pres_abs_pid_setpoint_limit()
{
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 5.0) / GetParameterValue(DP1_RANGE)));
}

int d_pres_diff_pid_setpoint_limit()
{
	return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 5.0) / GetParameterValue(DP2_RANGE)));
}

            
int d_temp_upper_alarm_off_limit()          
{  
   return 2500;
}
             
int d_temp_upper_alarm_on_limit()           
{  
   return 2501;
}
             
int d_temp_lower_alarm_off_limit()          
{  
   return 2100;
}
             
int d_temp_lower_alarm_on_limit()           
{  
   return 2099;
}
             
int d_rh_upper_alarm_off_limit()            
{  
   return 5500;
}
             
int d_rh_upper_alarm_on_limit()             
{  
   return 5501;
}
             
int d_rh_lower_alarm_off_limit()            
{  
   return 4500;
}
             
int d_rh_lower_alarm_on_limit()             
{  
   return 4499;
}

int d_part_mc_upper_alarm_off_limit()
{
	return 4000;
}

int d_part_mc_upper_alarm_on_limit()
{
	return 5000;
}

int d_part_voc_upper_alarm_off_limit()
{
	return 3500;
}

int d_part_voc_upper_alarm_on_limit()
{
	return 4000;
}   

int d_part_nox_upper_alarm_off_limit()
{
	return 20;
}

int d_part_nox_upper_alarm_on_limit()
{
	return 25;
}           

int d_pres_abs_out_low_pres()               
{  
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 5.0) / GetParameterValue(DP1_RANGE)));
   //return 0;//return -DP1_SENSOR_RESOLUTION_SM9543;
}
             
int d_pres_abs_out_high_pres()              
{
	return ((DP1_SENSOR_RESOLUTION_SM9543/2) + ((DP1_SENSOR_RESOLUTION_SM9543 * 10.0) / GetParameterValue(DP1_RANGE)));  
   //return DP1_SENSOR_RESOLUTION_SM9543;
}
             
int d_pres_diff_out_low_pres()             
{  
	return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 5.0) / GetParameterValue(DP2_RANGE)));
   //return 0;//return -DP2_SENSOR_RESOLUTION_SM9543;
}
             
int d_pres_diff_out_high_pres()             
{  
	return ((DP2_SENSOR_RESOLUTION_SM9543/2) + ((DP2_SENSOR_RESOLUTION_SM9543 * 10.0) / GetParameterValue(DP2_RANGE)));
   //return DP2_SENSOR_RESOLUTION_SM9543;
}
             
int d_temp_out_low_temp()                   
{  
   return 2100;
}
             
int d_temp_out_high_temp()                  
{  
   return 2500;
}
             
int d_rh_out_low_rh()                       
{  
   return 4500;
}
             
int d_rh_out_high_rh()                      
{  
   return 5500;
}
             

int min_pres_abs_upper_alarm_off_limit()    
{  
   return 0;
}
    
int min_pres_abs_lower_alarm_on_limit()     
{  
   return 0;
}
    
int min_pres_diff_upper_alarm_off_limit()   
{  
   return 0;
}

int min_pres_diff_lower_alarm_on_limit()    
{  
   return 0;
}

int min_pres_diff3_upper_alarm_off_limit()
{
	return 0;
}

int min_pres_diff3_lower_alarm_on_limit()
{
	return 0;
}

int min_temp_upper_alarm_off_limit()        
{  
   return -4000;
}
    
int min_temp_lower_alarm_on_limit()         
{  
   return -4000;
}
    
int min_rh_upper_alarm_off_limit()          
{  
   return 0;
}
    
int min_rh_lower_alarm_on_limit()           
{  
   return 0;
}

int min_part_mc_upper_alarm_off_limit()
{
	return 0;
}

int min_part_voc_upper_alarm_off_limit()
{
	return 0;
}

int min_part_nox_upper_alarm_off_limit()
{
	return 0;
}

int min_abs_pres_zero_adj()                 
{  
   return -DP1_SENSOR_RESOLUTION_SM9543;
}
                
int min_diff_pres_zero_adj()                
{  
   return -DP2_SENSOR_RESOLUTION_SM9543;
}

int min_diff_pres3_zero_adj()
{
	return -DP3_SENSOR_RESOLUTION_SM9543;
}
                
int min_temp_zero_adj()                     
{  
   return -2000;
}
                
int min_rh_zero_adj()                       
{  
   return -2000;
}
                

int min_pres_abs_out_low_pres()             
{  
   return 0;
}
    
int min_pres_diff_out_low_pres()            
{  
   return 0;
}
    
int min_temp_out_low_temp()                 
{  
   return -4000;
}
    
int min_rh_out_low_rh()                     
{  
   return 0;
}
    
int max_pres_abs_upper_alarm_on_limit()     
{  
   return DP1_SENSOR_RESOLUTION_SM9543;
}

int max_pres_abs_lower_alarm_off_limit()    
{  
   return DP1_SENSOR_RESOLUTION_SM9543;
}

int max_pres_diff_upper_alarm_on_limit()    
{  
   return DP2_SENSOR_RESOLUTION_SM9543;
}

int max_pres_diff_lower_alarm_off_limit()   
{  
   return DP2_SENSOR_RESOLUTION_SM9543;
}

int max_pres_diff3_upper_alarm_on_limit()
{
	return DP3_SENSOR_RESOLUTION_SM9543;
}

int max_pres_diff3_lower_alarm_off_limit()
{
	return DP3_SENSOR_RESOLUTION_SM9543;
}

int max_temp_upper_alarm_on_limit()         
{  
   return 12380;
}

int max_temp_lower_alarm_off_limit()        
{  
   return 12380;
}

int max_rh_upper_alarm_on_limit()           
{  
   return 10000;
}

int max_rh_lower_alarm_off_limit()          
{  
   return 10000;
}

int max_part_mc_upper_alarm_on_limit()
{
	return 10000;
}

int max_part_voc_upper_alarm_on_limit()
{
	return 5000;
}

int max_part_nox_upper_alarm_on_limit()
{
	return 50;
}

int max_abs_pres_zero_rng()                 
{  
   return DP1_SENSOR_RESOLUTION_SM9543;
}
                
int max_diff_pres_zero_rng()                
{  
   return DP2_SENSOR_RESOLUTION_SM9543;
}
                
int max_diff_pres3_zero_rng()
{
	return DP3_SENSOR_RESOLUTION_SM9543;
}
 
int max_abs_pres_zero_adj()                 
{  
   return DP1_SENSOR_RESOLUTION_SM9543;
}
                
int max_diff_pres_zero_adj()                
{  
   return DP2_SENSOR_RESOLUTION_SM9543;
}

int max_diff_pres3_zero_adj()
{
	return DP3_SENSOR_RESOLUTION_SM9543;
}
                
int max_temp_zero_adj()                     
{  
   return 2000;
}
                
int max_rh_zero_adj()                       
{  
   return 2000;
}
                

int max_pres_abs_out_high_pres()            
{  
   return DP1_SENSOR_RESOLUTION_SM9543;
}

int max_pres_diff_out_high_pres()           
{  
   return DP2_SENSOR_RESOLUTION_SM9543;
}

int max_temp_out_high_temp()                
{  
   return 12380;
}

int max_rh_out_high_rh()                    
{  
   return 10000;
}

//static uint8_t IsPresEnabled()
//{
   //return IsDP1Enabled() || IsDP2Enabled()  || IsDP3Enabled();
//}


static uint8_t IsPresAbsOutEnabled()
{
    if( !IsDP1Enabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return TRUE;
    return (GetParameterValue(DP1_OUTPUT_TYPE) == 1);
}


static uint8_t IsPresAbsPIDEnabled()
{
    if( !IsDP1Enabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return FALSE;
    return (GetParameterValue(DP1_OUTPUT_TYPE) == 2);
}


static uint8_t IsPresDiffOutEnabled()
{
    if( !IsDP2Enabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return TRUE;
    return (GetParameterValue(DP2_OUTPUT_TYPE) == 1);
}

static uint8_t IsPresDiffPIDEnabled()
{
    if( !IsDP2Enabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return FALSE;
    return (GetParameterValue(DP2_OUTPUT_TYPE) == 2);
}

static uint8_t IsTempOutEnabled()
{
    if( !IsTemperatureEnabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return TRUE;
    return (GetParameterValue(TEMP_OUTPUT_TYPE) == 1);
}


static uint8_t IsTempPIDEnabled()
{
    if( !IsTemperatureEnabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return FALSE;
    return (GetParameterValue(TEMP_OUTPUT_TYPE) == 2);
}

static uint8_t IsRHOutEnabled()
{
    if( !IsHumidityEnabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return TRUE;
    return (GetParameterValue(RH_OUTPUT_TYPE) == 1);
}


static uint8_t IsRHPIDEnabled()
{
    if( !IsHumidityEnabled())
       return FALSE;
    if( ! IsPIDEnabled())
       return FALSE;
    return (GetParameterValue(RH_OUTPUT_TYPE) == 2);
}


static uint8_t IsTempRHPIDEnabled()
{
    return IsRHPIDEnabled() && IsTempPIDEnabled();
}


static uint8_t IsPresAbsOutActive()
{
    return IsPresAbsOutEnabled() || IsPresAbsPIDEnabled();
}


static uint8_t IsPresDiffOutActive()
{
    return IsPresDiffOutEnabled() || IsPresDiffPIDEnabled();
}

static uint8_t IsTempOutActive()
{
    return IsTempOutEnabled() || IsTempPIDEnabled();
}

static uint8_t IsRHOutActive()
{
    return IsRHOutEnabled() || IsRHPIDEnabled();
}


static uint8_t IsTempOrRHEnabled()
{
    return(IsTemperatureEnabled() || IsHumidityEnabled());
}

//static uint8_t IsParticalAlarmEnabled()
//{
	//return IsAlarmEnabled() && GetParameterValue(PARTICAL_ALARM_SETUP) != ALARM_DISABLED;
//}

static uint8_t IsPresAbsAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(DP1_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsPresDiffAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(DP2_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsPresDiff3AlarmEnabled()
{
	return IsAlarmEnabled() && GetParameterValue(DP3_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsTempAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(TEMP_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsHumidityAlarmEnabled()
{
   return IsAlarmEnabled() && GetParameterValue(RH_ALARM_SETUP) != ALARM_DISABLED;
}

static uint8_t IsRTCEnabled()
{
	return GetParameterValue(RTC_ENABLE_DISABLE) != RTC_DISABLED;
}

//uint8_t GetDisplayPara(void)
//{
	//return 0;
//}

//#pragma GCC pop_options



