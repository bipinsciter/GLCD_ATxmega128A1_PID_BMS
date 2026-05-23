/*
 * Copyright:      Fabian Maximilian Thiele  mailto:me@apetech.de
 * Author:         Fabian Maximilian Thiele
 * Remarks:        this Copyright must be included
 * known Problems: none
 * Version:        1.1
 * Description:    Graphic Library for KS0108- (and compatible) based LCDs
 * 
 */

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

#ifndef	KS0108_H
#define KS0108_H

//For UC1638 -----------------------------------------------------------------------
#define RST_HIGH    	PORTH_OUTSET = PIN6_bm
#define RST_LOW     	PORTH_OUTCLR = PIN6_bm
#define RST_DIR_OUTPUT  PORTH_DIRSET = PIN6_bm

#define RS_HIGH	     	PORTH_OUTSET = PIN3_bm
#define RS_LOW	     	PORTH_OUTCLR = PIN3_bm
#define RS_DIR_OUTPUT   PORTH_DIRSET = PIN3_bm

#define CS_HIGH	     	PORTH_OUTSET = PIN7_bm
#define CS_LOW	     	PORTH_OUTCLR = PIN7_bm
#define CS_DIR_OUTPUT   PORTH_DIRSET = PIN7_bm

#define RDB_HIGH     	PORTH_OUTSET = PIN5_bm
#define RDB_LOW     	PORTH_OUTCLR = PIN5_bm
#define RDB_DIR_OUTPUT  PORTH_DIRSET = PIN5_bm

#define WRD_HIGH     	PORTH_OUTSET = PIN4_bm
#define WRD_LOW     	PORTH_OUTCLR = PIN4_bm
#define WRD_DIR_OUTPUT  PORTH_DIRSET = PIN4_bm

#define LCD_DATA_IN		PORTJ_IN		
#define LCD_DATA_OUT	PORTJ_OUT

#define LCD_DATA_DIR_OUT	PORTJ_DIR=0xFF		// Data Direction Register for Data Port
#define LCD_DATA_DIR_IN		PORTJ_DIR=0x00		// Data Direction Register for Data Port


#define BLANK1               0u                  /**< Glcd Blank.*/
#define FILL                (!BLANK1)            /**< Glcd Fill.*/

#define GLCD_PIXEL_X        240                 /**< Pixels in X-Direction.*/
#define GLCD_PIXEL_Y        160                  /**< Pixels in Y-Direction.*/
#define GLCD_MAX_PAGE       20                   /**< Maximum Page Size.*/
#define GLCD_MAX_X_POS      240                  /**< Maximum X Position.*/

//-----------------------------------------------------------------------


// Commands
#define KS0108_LCD_ON		 	    0x3F
#define KS0108_LCD_OFF				0x3E
#define KS0108_LCD_SET_ADD			0x40
#define KS0108_LCD_SET_PAGE		    0xB8
#define KS0108_LCD_DISP_START		0xC0

/// new commands
#define KS0713_LCD_ON				0xAF
#define KS0713_LCD_OFF				0xAE
#define KS0713_LCD_SET_ADD_HIGH	    0x10
#define KS0713_LCD_SET_ADD_LOW	    0x0
#define KS0713_LCD_SET_PAGE		    0xB0
#define KS0713_LCD_DISP_START		0x40


// Colors
#define BLACK				0xFF
#define WHITE				0x00

// Font Indices
#define FONT_LENGTH			0
#define FONT_FIXED_WIDTH	2
#define FONT_HEIGHT			3
#define FONT_FIRST_CHAR		4
#define FONT_CHAR_COUNT		5
#define FONT_WIDTH_TABLE	6

// Uncomment for slow drawing
// #define DEBUG

typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t page;
} lcdCoord;

typedef uint8_t (*ks0xxx_FontCallback)(const uint8_t*);

//LCD Types
#define KS0108   1
#define KS0713   2
#define UC1638   3

//
// Function Prototypes
//
uint8_t ks0xxx_lcdType();


// Graphic Functions
void ks0xxx_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void ks0xxx_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void ks0xxx_DrawRoundRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color);
void ks0xxx_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
void ks0xxx_InvertRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
void ks0xxx_SetInverted(uint8_t invert);
void ks0xxx_SetDot(uint8_t x, uint8_t y, uint8_t color);

#define ks0xxx_DrawVertLine(x, y, length, color) {ks0xxx_FillRect(x, y, 0, length, color);}
#define ks0xxx_DrawHoriLine(x, y, length, color) {ks0xxx_FillRect(x, y, length, 0, color);}
#define ks0xxx_DrawCircle(xCenter, yCenter, radius, color) {ks0xxx_DrawRoundRect(xCenter-radius, yCenter-radius, 2*radius, 2*radius, radius, color);}
#define ks0xxx_ClearScreen() {ks0xxx_FillRect(0, 0, GLCD_PIXEL_X-1, GLCD_PIXEL_Y-1, WHITE);}

void ks0xxx_Circle(uint8_t xcenter, uint8_t ycenter, uint8_t radius, uint8_t color);
void LoadBitmap(unsigned char *bitmap);

// Font Functions
uint8_t ks0xxx_ReadFontData(const uint8_t* ptr);     //Standard Read Callback
void ks0xxx_SelectFont(const uint8_t* font, ks0xxx_FontCallback callback, uint8_t color);
uint8_t ks0xxx_FontHeight();
int ks0xxx_PutChar(char c);
void ks0xxx_Puts(char* str);
void ks0xxx_Puts_P(PGM_P str);
uint8_t ks0xxx_CharWidth(char c);
uint16_t ks0xxx_StringWidth(char* str);
uint16_t ks0xxx_StringWidth_P(PGM_P str);

// Control Functions
void delay1(void);
void uc1638_Init(void);
void uc1638_GotoXY(uint8_t x, uint8_t y);
void uc1638WriteCommand(uint8_t cmd);
void uc1638WriteData(uint8_t cmd);
void uc1638WriteScreenData(uint8_t data);
uint8_t uc1638ReadData(void);
uint8_t uc1638ReadScreenData(void);
void uc1638_Set_X(uint8_t);
void uc1638_Set_Y(uint8_t);
void uc1638_FillScreen(uint8_t pattern);
void Glcd_V_Line ( unsigned char x_pos, unsigned char y_start, unsigned char y_end, unsigned char color);
void Glcd_H_Line( unsigned char x_start, unsigned char y_pos, unsigned char x_end, unsigned char color);

#endif
