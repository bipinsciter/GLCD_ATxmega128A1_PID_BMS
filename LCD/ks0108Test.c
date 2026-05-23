/*
 * Copyright:      Fabian Maximilian Thiele  mailto:me@apetech.de
 * Author:         Fabian Maximilian Thiele
 * Remarks:        this Copyright must be included
 * known Problems: none
 * Version:        1.1
 * Description:    KS0108 Library Demo Program 
 * 
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "ks0108.h"
#include "arial_bold_14.h"
#include "corsiva_12.h"

int main(void) {
	
	// Wait a little while the display starts up
	for(volatile uint16_t i=0; i<15000; i++);
	
	// Initialize the LCD
	ks0108Init(0);
	
	// Select a font
	ks0108SelectFont(Arial_Bold_14, ks0108ReadFontData, BLACK);
	// Set a position
	ks0108GotoXY(15,10);
	// Print some text
	ks0108Puts_P(PSTR("KS0108-Treiber"));
	// a nice little round rect
	ks0108DrawRoundRect(5, 5, 117, 20, 8, BLACK);

	// Once again :)
	// Select a font
	ks0108SelectFont(Corsiva_12, ks0108ReadFontData, BLACK);
	// Set a position
	ks0108GotoXY(5,30);
	// Print some text
	ks0108Puts_P(PSTR("http://www.apetech.de\nmailto:me@apetech.de"));

	while(1);
}
