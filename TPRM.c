//#define _SFR_ASM_COMPAT 1
#include <avr/io.h>        // This defers to avr/io.h for GCC
#include <avr/interrupt.h>    // This defers to avr/signal.h for GCC
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "os_api.h"
#include "locked_printf.h"
#include "SW_I2C1Interface.h"
#include "SW_I2C2Interface.h"
#include "SW_I2C3Interface.h"
#include "SW_I2C4Interface.h"
#include "UserInterface.h"
#include "SerialInterface.h"
#include "DeviceIO.h"
#include "PeriodicProcess.h"
#include "HardwareInfo.h"
#include "rtc.h"
#include "eepromAdr.h"
#include <util/delay.h>

// Convenience macro for enabling pull-ups on specified pins on any port.
#define __PORT_PULLUP(port, mask) { \
	PORTCFG.MPCMASK = mask ; \
	port.PIN0CTRL = PORT_OPC_PULLUP_gc; \
}

void InitSystemClock(void)
{		
	//Set Clock to Internal 32MHz
	OSC.CTRL = OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));
	
	//Write CCP to change clock option otherwise change will not happens -----------------------------
	CCP = CCP_IOREG_gc;	
	asm("LDI R24,0x01"); //For switching to 32MHz internal RC. Modify 0x01 as required
	asm("STS 0x0040,R24");
	//------------------------------------------------------------------------------------------------
	
	OSC.CTRL &= ~OSC_RC2MEN_bm;
			
	//Select Clock Source for RTC to External 32.768 KHz
	CLK.RTCCTRL = (CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm);
}

void DisableUnusedModules(void)
{
	// the power reduction settings are specific for each device
	// PRGEN general power reduction register
	//
	// disable
	//	- AES
	//  - DMA
	//  - event system
	//  - external bus interface
	//
	PR.PRGEN |= PR_AES_bm | PR_DMA_bm | PR_EVSYS_bm | PR_EBI_bm;
	//PR_PRGEN = 0b11010011;

	// PRPA, PRPB power reduction port A/B register
	//
	// disable
	//	- analog comparator
	//	- ADC on port B
	//  - DAC
	//
	// leave ADCA active !
	//
	PR.PRPA |= PR_AC_bm | PR_ADC_bm | PR_DAC_bm;
	PR.PRPB |= PR_AC_bm | PR_DAC_bm;
	//PR_PRPA = 0b00000111;
	//PR_PRPB = 0b00000101;
	
	// PRPC/D/E/F Power Reduction Port C/D/E/F Register
	//
	// disable unused timers
	//
	// - high resolution extension
	// - TCD1, TCD1       on, used for nested intérrupt tests
	// - TCC1, TCD0, TCE0 (if not simulating serial input)
	//
	// TCC0 is the tick timer, always on!
	//
	PR.PRPC |= PR_HIRES_bm;
	PR.PRPE |= PR_HIRES_bm;
	//PR_PRPC = 0b01001110;
	//PR_PRPE = 0b01001110;
	
	// enable eeprom and flash power reduction
	//
	//NVM.CTRLB |= NVM_EPRM_bm | NVM_FPRM_bm;

	//DISABLE_JTAG();
}

void InitGPIO(void)
{
	//==============
	//  Port A
	//==============
	// PA0 is OUTPUT1
	// PA1 is OUTPUT2
	// PA2 is OUTPUT3
	// PA3 is OUTPUT4
	// PA4 is ALARM7
	// PA5 is NC
	// PA6 is KEY5
	// PA7 is KEY6
	
	PORTA_DIR = 0x3F;
	PORTA_OUT = 0xFF;      // set high
	
	PORTA.DIRCLR = PIN6_bm; // pin6 is input
	__PORT_PULLUP(PORTA, PIN6_bm); // Enable Pull up on KEY3
	
	PORTA.DIRCLR = PIN7_bm; // pin7 is input
	__PORT_PULLUP(PORTA, PIN7_bm); // Enable Pull up on KEY4
	
	//==============
	//  Port B
	//==============
	// PB0 is Analog Input1
	// PB1 is Analog Input2
	// PB2 is Analog Input3
	// PB3 is Door Status
	// PB4 is TMS
	// PB5 is TDI
	// PB6 is TCK
	// PB7 is TDO
	
	PORTB_DIR = 0x00;
	
	PORTB.DIRCLR = PIN3_bm; // pin3 DoorStatus is input
	
	//==============
	//  Port C
	//==============
	// PC0 is SDA1
	// PC1 is SCL1
	// PC2 is RXD1
	// PC3 is TXD1
	// PC4 is DIR1
	// PC5 is DIR2
	// PC6 is RXD2
	// PC7 is TXD2
	
	PORTC.DIRSET = PIN0_bm; // pin0 is output
	PORTC.OUTSET = PIN0_bm; // set SDA1=1
	
	PORTC.DIRSET = PIN1_bm; // pin1 is output
	PORTC.OUTSET = PIN1_bm; // set SCL1=1
	
	PORTC.DIRCLR = PIN2_bm; // pin2 RXD1 is input
	
	PORTC.DIRSET = PIN3_bm; // pin3 TXD1 is output
	
	PORTC.DIRSET = PIN4_bm; // pin4 is output
	PORTC.OUTCLR = PIN4_bm; // DE1=0, set the RS485 driver to receive mode
	
	PORTC.DIRSET = PIN5_bm; // pin5 is output
	PORTC.OUTCLR = PIN5_bm; // DE2=0, set the RS485 driver to receive mode
	
	PORTC.DIRCLR = PIN6_bm; // pin2 RXD2 is input
	
	PORTC.DIRSET = PIN7_bm; // pin3 TXD2 is output
	
	//==============
	//  Port D
	//==============
	// PD0 is SDA2
	// PD1 is SCL2
	// PD2 is RXD3
	// PD3 is TXD3
	// PD4 is KEY1
	// PD5 is KEY2
	// PD6 is KEY3
	// PD7 is KEY4
	
	PORTD.DIRSET = PIN0_bm; // pin0 is output
	PORTD.OUTSET = PIN0_bm; // set SDA2=1
	
	PORTD.DIRSET = PIN1_bm; // pin1 is output
	PORTD.OUTSET = PIN1_bm; // set SCL2=1
	
	PORTD.DIRCLR = PIN2_bm; // pin2 RXD3 is input
	
	PORTD.DIRSET = PIN3_bm; // pin3 TXD3 is output
	
	PORTD.DIRCLR = PIN4_bm; // pin4 is input
	__PORT_PULLUP(PORTD, PIN4_bm); // Enable Pull up on KEY1
	
	PORTD.DIRCLR = PIN5_bm; // pin5 is input
	__PORT_PULLUP(PORTD, PIN5_bm); // Enable Pull up on KEY2
	
	PORTD.DIRCLR = PIN6_bm; // pin6 is input
	__PORT_PULLUP(PORTD, PIN6_bm); // Enable Pull up on KEY3
		
	PORTD.DIRCLR = PIN7_bm; // pin7 is input
	__PORT_PULLUP(PORTD, PIN7_bm); // Enable Pull up on KEY4	
	
	//==============
	//  Port E
	//==============
	// PE0 is SDA3
	// PE1 is SCL3
	// PE2 is RXD4
	// PE3 is TXD4
	// PE4 is CS
	// PE5 is SDI
	// PE6 is SDO
	// PE7 is SCLK
	
	PORTE.DIRSET = PIN0_bm; // pin0 is output
	PORTE.OUTSET = PIN0_bm; // set SDA3=1
	
	PORTE.DIRSET = PIN1_bm; // pin1 is output
	PORTE.OUTSET = PIN1_bm; // set SCL3=1
	
	PORTE.DIRCLR = PIN2_bm; // pin2 RXD4 is input
	
	PORTE.DIRSET = PIN3_bm; // pin3 TXD4 is output
	
	PORTE.DIRSET = PIN4_bm; // pin4 is output
	PORTE.OUTSET = PIN4_bm; // set CS=1
	
	PORTE.DIRSET = PIN5_bm; // pin5 is output
	PORTE.OUTSET = PIN5_bm; // set SDI=1
	
	PORTE.DIRCLR = PIN6_bm; // pin6 is input
	PORTE.OUTSET = PIN6_bm; // set SDO=1
	
	PORTE.DIRSET = PIN7_bm; // pin7 is output
	PORTE.OUTSET = PIN7_bm; // set SCLK=1
	
	//==============
	//  Port F
	//==============
	// PF0 is SDA4
	// PF1 is SCL4
	// PF2 is ALARM LED1
	// PF3 is OK LED1
	// PF4 is ALARM LED2
	// PF5 is OK LED2
	// PF6 is ALARM LED2
	// PF7 is OK LED2
	
	PORTF.DIRSET = PIN0_bm; // pin0 is output
	PORTF.OUTSET = PIN0_bm; // set SDA4=1
	
	PORTF.DIRSET = PIN1_bm; // pin1 is output
	PORTF.OUTSET = PIN1_bm; // set SCL4=1
	
	PORTF.DIRSET = PIN2_bm; // pin2 is output
	PORTF.OUTSET = PIN2_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN3_bm; // pin3 is output
	PORTF.OUTSET = PIN3_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN4_bm; // pin4 is output
	PORTF.OUTSET = PIN4_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN5_bm; // pin5 is output
	PORTF.OUTSET = PIN5_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN6_bm; // pin6 is output
	PORTF.OUTSET = PIN6_bm; // set LED=1 for off
	
	PORTF.DIRSET = PIN7_bm; // pin7 is output
	PORTF.OUTSET = PIN7_bm; // set LED=1 for off
	
	//==============
	//  Port H
	//==============
	// PH0 is BUZZER
	// PH1 is RST1
	// PH2 is XBEE RESET
	// PH3 is LCD RS
	// PH4 is LCD RB0
	// PH5 is LCD RB1
	// PH6 is LCD RST
	// PH7 is LCD CS
	
	PORTH.DIRSET = PIN0_bm; // pin0 is output
	PORTH.OUTCLR = PIN0_bm; // set BUZZER OFF
		
	PORTH.DIRSET = PIN1_bm; // pin1 is output
	PORTH.OUTSET = PIN1_bm; // set high
	
	PORTH.DIRSET = PIN2_bm; // pin2 is output
	PORTH.OUTSET = PIN2_bm; // set high
	
	PORTH.DIRSET = PIN3_bm; // pin3 is output
	PORTH.OUTSET = PIN3_bm; // set LCD RS to HIGH
	
	PORTH.DIRSET = PIN4_bm; // pin4 is output
	PORTH.OUTSET = PIN4_bm; // set LCD RB0 to HIGH
	
	PORTH.DIRSET = PIN5_bm; // pin5 is output
	PORTH.OUTSET = PIN5_bm; // set LCD RB1 to HIGH
	
	PORTH.DIRSET = PIN6_bm; // pin6 is output
	PORTH.OUTSET = PIN6_bm; // set LCD RST to HIGH
	
	PORTH.DIRSET = PIN7_bm; // pin7 is output
	PORTH.OUTSET = PIN7_bm; // set LCD CS to HIGH
	
	//==============
	//  Port J
	//==============
	// PJ0 is LCD D0
	// PJ1 is LCD D1
	// PJ2 is LCD D2
	// PJ3 is LCD D3
	// PJ4 is LCD D4
	// PJ5 is LCD D5
	// PJ6 is LCD D6
	// PJ7 is LCD D7
	
	PORTJ_DIR = 0xFF;		// Set all pins output
	PORTJ_OUT = 0x00;		// set low

	//==============
	//  Port K
	//==============
	// PK0 is INPUT1
	// PK1 is INPUT2
	// PK2 is INPUT3
	// PK3 is INPUT4
	// PK4 is INPUT5
	// PK5 is ALARM4
	// PK6 is ALARM5
	// PK7 is ALARM6
	
	PORTK_DIR = 0xE0;		
	__PORT_PULLUP(PORTK, PIN0_bm);
	__PORT_PULLUP(PORTK, PIN1_bm);
	__PORT_PULLUP(PORTK, PIN2_bm);
	__PORT_PULLUP(PORTK, PIN3_bm);
	__PORT_PULLUP(PORTK, PIN4_bm);
	
	PORTK.DIRSET = PIN5_bm; // pin5 is output
	PORTK.OUTCLR = PIN5_bm; // set ALARM4 to LOW
	
	PORTK.DIRSET = PIN6_bm; // pin6 is output
	PORTK.OUTCLR = PIN6_bm; // set ALARM5 to LOW
	
	PORTK.DIRSET = PIN7_bm; // pin7 is output
	PORTK.OUTCLR = PIN7_bm; // set ALARM6 to LOW
	
	//==============
	//  Port Q
	//==============
	// PQ0 is XTAL1
	// PQ1 is XTAL2
	// PQ2 is ALARM LED4
	// PQ3 is OK LED4
	
	PORTQ.DIRSET = PIN2_bm; // pin2 is output
	PORTQ.OUTSET = PIN2_bm; // set LED=1 for off
	
	PORTQ.DIRSET = PIN3_bm; // pin3 is output
	PORTQ.OUTSET = PIN3_bm; // set LED=1 for off
}

void Init_InternalRTC(void)
{
	while(RTC.STATUS & RTC_SYNCBUSY_bm);
	
	RTC.PER = 511;
	RTC.CNT = 0;
	RTC.COMP = 0;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC_INTCTRL = RTC_OVFINTLVL_LO_gc;
}

void SetDefault(void)
{	
	if(eeprom_read_word((unsigned int *)EA_DEFAULT_SAVE)!=0xCBCD)
	{
		OSWriteEEPromWord((unsigned int *)EA_CONFIG_SYSTEM, ~27007);	//10623 //2429
		//OSWriteEEPromWord((unsigned int *)EA_CONFIG_SYSTEM, ~10623);	//10623 //2429
		//OSWriteEEPromWord((unsigned int *)EA_CONFIG_SYSTEM, ~27000);	//10623 //2429
		
		OSWriteEEPromWord((unsigned int *)EA_TEMP_RH_READING_AVERAGE, 20);
		OSWriteEEPromWord((unsigned int *)EA_PRES_READING_AVERAGE, 20);
		
		OSWriteEEPromWord((unsigned int *)EA_DP1_SENS_TYPE, PRES_SENS_SM9543);
		OSWriteEEPromWord((unsigned int *)EA_DP2_SENS_TYPE, PRES_SENS_SM9543);
		OSWriteEEPromWord((unsigned int *)EA_DP3_SENS_TYPE, PRES_SENS_SM9543);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_RH_SENS_TYPE, TEMP_RH_SENS_SHT25);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_RH2_SENS_TYPE, TEMP_RH_SENS_SHT25);
		
		OSWriteEEPromWord((unsigned int *)EA_DP1_UNIT, PRESSURE_PA);
		OSWriteEEPromWord((unsigned int *)EA_DP2_UNIT, PRESSURE_PA);
		OSWriteEEPromWord((unsigned int *)EA_DP3_UNIT, PRESSURE_PA);
		OSWriteEEPromWord((unsigned int *)EA_TEMPERATURE_UNIT, TEMPERATURE_DEG_C);
		OSWriteEEPromWord((unsigned int *)EA_HUMIDITY_UNIT, HUMIDITY_RH);
		
		OSWriteEEPromWord((unsigned int *)EA_DISPLAY_MODE, DISP_MODE_ALL_PARA);
		
		OSWriteEEPromWord((unsigned int *)EA_DP1_RANGE, 1000);
		OSWriteEEPromWord((unsigned int *)EA_DP2_RANGE, 1000);
		OSWriteEEPromWord((unsigned int *)EA_DP3_RANGE, 1000);
		
		OSWriteEEPromWord((unsigned int *)EA_DEVICE_ID, 1);
		OSWriteEEPromWord((unsigned int *)EA_DP1_ZERO_ADJ, 0);
		OSWriteEEPromWord((unsigned int *)EA_DP2_ZERO_ADJ, 0);
		OSWriteEEPromWord((unsigned int *)EA_DP3_ZERO_ADJ, 0);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_ZERO_ADJ, 0);
		OSWriteEEPromWord((unsigned int *)EA_TEMP2_ZERO_ADJ, 0);
		OSWriteEEPromWord((unsigned int *)EA_RH_ZERO_ADJ, 0);
		
		OSWriteEEPromWord((unsigned int *)EA_DP1_AREA_TYPE, PRES_AREA_SQUARE);
		OSWriteEEPromWord((unsigned int *)EA_DP1_ALARM_SETUP, ALARM_DISABLED);
		OSWriteEEPromWord((unsigned int *)EA_DP1_UPPER_ALARM_ON_LIMIT, (DP1_SENSOR_RESOLUTION_SM9543/2)+327);
		OSWriteEEPromWord((unsigned int *)EA_DP1_UPPER_ALARM_OFF_LIMIT, (DP1_SENSOR_RESOLUTION_SM9543/2)+311);
		OSWriteEEPromWord((unsigned int *)EA_DP1_LOWER_ALARM_OFF_LIMIT, (DP1_SENSOR_RESOLUTION_SM9543/2)-163);
		OSWriteEEPromWord((unsigned int *)EA_DP1_LOWER_ALARM_ON_LIMIT, (DP1_SENSOR_RESOLUTION_SM9543/2)-147);
		
		OSWriteEEPromWord((unsigned int *)EA_DP2_AREA_TYPE, PRES_AREA_SQUARE);
		OSWriteEEPromWord((unsigned int *)EA_DP2_ALARM_SETUP, ALARM_DISABLED);
		OSWriteEEPromWord((unsigned int *)EA_DP2_UPPER_ALARM_ON_LIMIT, (DP2_SENSOR_RESOLUTION_SM9543/2)+327);
		OSWriteEEPromWord((unsigned int *)EA_DP2_UPPER_ALARM_OFF_LIMIT, (DP2_SENSOR_RESOLUTION_SM9543/2)+311);
		OSWriteEEPromWord((unsigned int *)EA_DP2_LOWER_ALARM_OFF_LIMIT, (DP2_SENSOR_RESOLUTION_SM9543/2)-163);
		OSWriteEEPromWord((unsigned int *)EA_DP2_LOWER_ALARM_ON_LIMIT, (DP2_SENSOR_RESOLUTION_SM9543/2)-147);
		
		OSWriteEEPromWord((unsigned int *)EA_DP3_AREA_TYPE, PRES_AREA_SQUARE);
		OSWriteEEPromWord((unsigned int *)EA_DP3_ALARM_SETUP, ALARM_DISABLED);
		OSWriteEEPromWord((unsigned int *)EA_DP3_UPPER_ALARM_ON_LIMIT, (DP3_SENSOR_RESOLUTION_SM9543/2)+327);
		OSWriteEEPromWord((unsigned int *)EA_DP3_UPPER_ALARM_OFF_LIMIT, (DP3_SENSOR_RESOLUTION_SM9543/2)+311);
		OSWriteEEPromWord((unsigned int *)EA_DP3_LOWER_ALARM_OFF_LIMIT, (DP3_SENSOR_RESOLUTION_SM9543/2)-163);
		OSWriteEEPromWord((unsigned int *)EA_DP3_LOWER_ALARM_ON_LIMIT, (DP3_SENSOR_RESOLUTION_SM9543/2)-147);
		
		OSWriteEEPromWord((unsigned int *)EA_TEMP_ALARM_SETUP, ALARM_DISABLED);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_UPPER_ALARM_ON_LIMIT, 2510);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_UPPER_ALARM_OFF_LIMIT, 2500);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_LOWER_ALARM_OFF_LIMIT, 2110);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_LOWER_ALARM_ON_LIMIT, 2100);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_FIRE_ALM_SET, 7500);
		
		OSWriteEEPromWord((unsigned int *)EA_RH_ALARM_SETUP, ALARM_DISABLED);
		OSWriteEEPromWord((unsigned int *)EA_RH_UPPER_ALARM_ON_LIMIT, 6010);
		OSWriteEEPromWord((unsigned int *)EA_RH_UPPER_ALARM_OFF_LIMIT, 6000);
		OSWriteEEPromWord((unsigned int *)EA_RH_LOWER_ALARM_OFF_LIMIT, 5010);
		OSWriteEEPromWord((unsigned int *)EA_RH_LOWER_ALARM_ON_LIMIT, 5000);
		
		OSWriteEEPromWord((unsigned int *)EA_DP1_OUTPUT_TYPE, 0);
		OSWriteEEPromWord((unsigned int *)EA_DP2_OUTPUT_TYPE, 0);
		OSWriteEEPromWord((unsigned int *)EA_TEMP_OUTPUT_TYPE, 0);
		OSWriteEEPromWord((unsigned int *)EA_RH_OUTPUT_TYPE, 0);
		
		OSWriteEEPromWord((unsigned int *)EA_RTC_SETUP, RTC_DISABLED);
		
		OSWriteEEPromWord((unsigned int *)EA_BUZZER_ON_TIME, 1);
		OSWriteEEPromWord((unsigned int *)EA_BUZZER_OFF_TIME, 3);
		OSWriteEEPromWord((unsigned int *)EA_BUZZER_DISABLE_TIME, 600);
		
		OSWriteEEPromWord((unsigned int *)EA_SERIAL_BAUD_RATE_VAL, SERIAL_BAUD_57600);
		OSWriteEEPromWord((unsigned int *)EA_SERIAL_DATA_STOP_PARITY_VAL, (3<<SERIAL_DATA_BITS_SHIFT) | (0<<SERIAL_STOP_BIT_SHIFT) | (SERIAL_PARITY_NONE << SERIAL_PARITY_SHIFT));

		memset(&gu8_inOutput[0][0],0,160);
		strcpy(&gu8_inOutput[0][0],"MANUAL");
		strcpy(&gu8_inOutput[1][0],"AUTO");
		strcpy(&gu8_inOutput[2][0],"MANUAL");
		strcpy(&gu8_inOutput[3][0],"AUTO");
		strcpy(&gu8_inOutput[4][0],"MANUAL");
		strcpy(&gu8_inOutput[5][0],"AUTO");
		strcpy(&gu8_inOutput[6][0],"MANUAL");
		strcpy(&gu8_inOutput[7][0],"AUTO");
		strcpy(&gu8_inOutput[8][0],"MANUAL");
		strcpy(&gu8_inOutput[9][0],"AUTO");
		strcpy(&gu8_inOutput[10][0],"MANUAL");
		strcpy(&gu8_inOutput[11][0],"AUTO");
		strcpy(&gu8_inOutput[12][0],"MANUAL");
		strcpy(&gu8_inOutput[13][0],"AUTO");
		strcpy(&gu8_inOutput[14][0],"MANUAL");
		strcpy(&gu8_inOutput[15][0],"AUTO");
		
		OSWriteEEPromBlock((uint8_t*)EA_IP1_HIGH_NAME,(uint8_t*)&gu8_inOutput[0][0],160);
		
		OSWriteEEPromWord((unsigned int *)EA_DEFAULT_SAVE, 0xCBCD);
	}
	else
	{
		OSReadEEPromBlock((uint8_t*)EA_IP1_HIGH_NAME,(uint8_t*)&gu8_inOutput[0][0],160);
	}
}


int main(void)                 // Main runs under the AvrX Stack
{
	//// software reset ?
	//if( RST.STATUS & RST_SRF_bm )
	//{
		//opstr("Software reset\r");
		//// reset this bit
		//RST.STATUS = RST_SRF_bm;
	//}
	//// power on reset ?
	//else if( RST.STATUS & RST_PORF_bm)
	//{
		//opstr("Power on reset\r");
		//// reset this bit
		//RST.STATUS = RST_PORF_bm;
	//}
	//// debugger reset ?
	//else if( RST.STATUS & RST_PDIRF_bm)
	//{
		//opstr("Debugger reset\r");
		//// reset this bit
		//RST.STATUS = RST_PDIRF_bm;
	//}
	//// external reset ?
	//else if( RST.STATUS & RST_EXTRF_bm)
	//{
		//opstr("External reset\r");
		//// reset this bit
		//RST.STATUS = RST_EXTRF_bm;
	//}
	//// brown out reset ?
	//else if( RST.STATUS & RST_BORF_bm)
	//{
		//opstr("Brown out reset\r");
		//// reset this bit
		//RST.STATUS = RST_BORF_bm;
	//}
	//// watch dog reset ?
	//else if( RST.STATUS & RST_WDRF_bm)
	//{
		//opstr("watch dog reset\r");
		//// reset this bit
		//RST.STATUS = RST_WDRF_bm;
	//}
	//// spike detection reset ?
	//else if( RST.STATUS & RST_SDRF_bm)
	//{
		//opstr("spike detection reset\r");
		//// reset this bit
		//RST.STATUS = RST_SDRF_bm;
	//}
	//else
	//{
		//opstr("Other reset\r");
	//}
	
	//Global Interrupt Disable
   cli();
	
   OSInit(); 
   
   InitSystemClock();	
   DisableUnusedModules();
   InitGPIO();	
   Init_InternalRTC();
   InitLockedPrintf();
   
   SW_I2C1InterfaceInit();	//PORTC	=> DAC/Display
   SW_I2C2InterfaceInit();	//PORTD => TempRH/DP1/RTC
   SW_I2C3InterfaceInit();	//PORTE	=> DP3/PostTemp
   SW_I2C4InterfaceInit();	//PORTF => DP2/ParticalSensor
   
   InitRTC();
   SetDefault();
   if(IsSerialEnabled()) SerialInterfaceInit();
   DeviceIOInit();
   PeriodicProcessInit();
   UserInterfaceInit();

   PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
   sei();		//Global Interrupt Enable

   OSStart();                   // Start OS switching
   
   while(1);
}


