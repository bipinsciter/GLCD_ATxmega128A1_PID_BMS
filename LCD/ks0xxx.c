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
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "ks0xxx.h"

lcdCoord            ks0xxx_Coord;
uint8_t             ks0xxx_Inverted=0;
ks0xxx_FontCallback  ks0xxx_FontRead;
uint8_t             ks0xxx_FontColor;
const uint8_t*      ks0xxx_Font;

uint8_t ks0xxx_ReadData(void);
uint8_t ks0xxx_DoReadData();

void ks0xxx_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    uint8_t length, i, y, yAlt, xTmp, yTmp;
    int16_t m;

    //
    // vertical line
    //
    if( x1 == x2 )
    {
        // x1|y1 must be the upper point
        if( y1 > y2 )
        {
            yTmp = y1;
            y1 = y2;
            y2 = yTmp;
        }
        ks0xxx_DrawVertLine(x1, y1, y2-y1, color);

        //
        // horizontal line
        //
    }
    else if( y1 == y2 )
    {
        // x1|y1 must be the left point
        if( x1 > x2 )
        {
            xTmp = x1;
            x1 = x2;
            x2 = xTmp;
        }
        ks0xxx_DrawHoriLine(x1, y1, x2-x1, color);

        //
        // schiefe line :)
        //
    }
    else
    {
        // angle >= 45°
        if( (y2-y1) >= (x2-x1) || (y1-y2) >= (x2-x1) )
        {
            // x1 must be smaller than x2
            if( x1 > x2 )
            {
                xTmp = x1;
                yTmp = y1;
                x1 = x2;
                y1 = y2;
                x2 = xTmp;
                y2 = yTmp;
            }

            length = x2-x1;     // not really the length :)
            m = ((y2-y1)*200)/length;
            yAlt = y1;

            for( i=0; i<=length; i++ )
            {
                y = ((m*i)/200)+y1;

                if( (m*i)%200 >= 100 )
                    y++;
                else if( (m*i)%200 <= -100 )
                    y--;

                ks0xxx_DrawLine(x1+i, yAlt, x1+i, y, color);

                if( length <= (y2-y1) && y1 < y2 )
                    yAlt = y+1;
                else if( length <= (y1-y2) && y1 > y2 )
                    yAlt = y-1;
                else
                    yAlt = y;
            }

            // angle < 45°
        }
        else
        {
            // y1 must be smaller than y2
            if( y1 > y2 )
            {
                xTmp = x1;
                yTmp = y1;
                x1 = x2;
                y1 = y2;
                x2 = xTmp;
                y2 = yTmp;
            }

            length = y2-y1;
            m = ((x2-x1)*200)/length;
            yAlt = x1;

            for( i=0; i<=length; i++ )
            {
                y = ((m*i)/200)+x1;

                if( (m*i)%200 >= 100 )
                    y++;
                else if( (m*i)%200 <= -100 )
                    y--;

                ks0xxx_DrawLine(yAlt, y1+i, y, y1+i, color);
                if( length <= (x2-x1) && x1 < x2 )
                    yAlt = y+1;
                else if( length <= (x1-x2) && x1 > x2 )
                    yAlt = y-1;
                else
                    yAlt = y;
            }
        }
    }
}

void ks0xxx_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color)
{
    ks0xxx_DrawHoriLine(x, y, width, color);             // top
    ks0xxx_DrawHoriLine(x, y+height, width, color);      // bottom
    ks0xxx_DrawVertLine(x, y, height, color);            // left
    ks0xxx_DrawVertLine(x+width, y, height, color);      // right
}

void ks0xxx_DrawRoundRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t radius, uint8_t color)
{
    int16_t tSwitch, x1 = 0, y1 = radius;
    tSwitch = 3 - 2 * radius;

    while( x1 <= y1 )
    {
        ks0xxx_SetDot(x+radius - x1, y+radius - y1, color);
        ks0xxx_SetDot(x+radius - y1, y+radius - x1, color);

        ks0xxx_SetDot(x+width-radius + x1, y+radius - y1, color);
        ks0xxx_SetDot(x+width-radius + y1, y+radius - x1, color);

        ks0xxx_SetDot(x+width-radius + x1, y+height-radius + y1, color);
        ks0xxx_SetDot(x+width-radius + y1, y+height-radius + x1, color);

        ks0xxx_SetDot(x+radius - x1, y+height-radius + y1, color);
        ks0xxx_SetDot(x+radius - y1, y+height-radius + x1, color);

        if( tSwitch < 0 )
        {
            tSwitch += (4 * x1 + 6);
        }
        else
        {
            tSwitch += (4 * (x1 - y1) + 10);
            y1--;
        }
        x1++;
    }

    ks0xxx_DrawHoriLine(x+radius, y, width-(2*radius), color);           // top
    ks0xxx_DrawHoriLine(x+radius, y+height, width-(2*radius), color);    // bottom
    ks0xxx_DrawVertLine(x, y+radius, height-(2*radius), color);          // left
    ks0xxx_DrawVertLine(x+width, y+radius, height-(2*radius), color);    // right
}

void Glcd_H_Line( unsigned char x_start, unsigned char y_pos, unsigned char x_end, unsigned char color)
{
	unsigned char temp;
	if( x_start > x_end )
	{
		temp = x_start;
		x_start = x_end;
		x_end = temp;
	}

	while( x_start <= x_end )
	{
		ks0xxx_SetDot(x_start, y_pos, color);
		x_start++;
	}
}

void Glcd_V_Line (unsigned char x_pos, unsigned char y_start, unsigned char y_end, unsigned char color)
{
	unsigned char temp;
	if( y_start > y_end )
	{
		temp = y_start;
		y_start = y_end;
		y_end = temp;
	}

	while( y_start <= y_end )
	{
		ks0xxx_SetDot(x_pos, y_start, color);
		y_start++;
	}
}

/*
 * Hardware-Functions ks0xxx_FillRect
 */
void ks0xxx_FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color)
{
    uint8_t mask, pageOffset, h, i, data;
    height++;

    pageOffset = y%8;
    y -= pageOffset;
    mask = 0xFF;
    if( height < 8-pageOffset )
    {
        mask >>= (8-height);
        h = height;
    }
    else
    {
        h = 8-pageOffset;
    }
    mask <<= pageOffset;

    uc1638_GotoXY(x, y);
    for( i=0; i<=width; i++ )
    {
        data = uc1638ReadScreenData();

        if( color == BLACK )
        {
            data |= mask;
        }
        else
        {
            data &= ~mask;
        }

        uc1638WriteScreenData(data);
    }

    while( h+8 <= height )
    {
        h += 8;
        y += 8;
        uc1638_GotoXY(x, y);

		uc1638WriteCommand(0x01);
        for( i=0; i<=width; i++ )
        {
            uc1638WriteData(color);
        }
    }

    if( h < height )
    {
        mask = ~(0xFF << (height-h));
		
        uc1638_GotoXY(x, y+8);

        for( i=0; i<=width; i++ )
        {
            data = uc1638ReadScreenData();

            if( color == BLACK )
            {
                data |= mask;
            }
            else
            {
                data &= ~mask;
            }

            uc1638WriteScreenData(data);
        }
    }
}

void ks0xxx_InvertRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t mask, pageOffset, h, i, data, tmpData;
    height++;

    pageOffset = y%8;
    y -= pageOffset;
    mask = 0xFF;
    if( height < 8-pageOffset )
    {
        mask >>= (8-height);
        h = height;
    }
    else
    {
        h = 8-pageOffset;
    }
    mask <<= pageOffset;

    uc1638_GotoXY(x, y);
    for( i=0; i<=width; i++ )
    {
        data = uc1638ReadScreenData();
        tmpData = ~data;
        data = (tmpData & mask) | (data & ~mask);
        uc1638WriteScreenData(data);
    }

    while( h+8 <= height )
    {
        h += 8;
        y += 8;
        uc1638_GotoXY(x, y);

        for( i=0; i<=width; i++ )
        {
            data = uc1638ReadScreenData();
            uc1638WriteScreenData(~data);
        }
    }

    if( h < height )
    {
        mask = ~(0xFF << (height-h));
        uc1638_GotoXY(x, y+8);

        for( i=0; i<=width; i++ )
        {
            data = uc1638ReadScreenData();
            tmpData = ~data;
            data = (tmpData & mask) | (data & ~mask);
            uc1638WriteScreenData(data);
        }
    }
}

void ks0xxx_SetInverted(uint8_t invert)
{
    if( ks0xxx_Inverted != invert )
    {
        ks0xxx_InvertRect(0,0,GLCD_PIXEL_X-1,GLCD_PIXEL_Y-1);
        ks0xxx_Inverted = invert;
    }
}

void ks0xxx_SetDot(uint8_t x, uint8_t y, uint8_t color)
{
    uint8_t data;

    uc1638_GotoXY(x, y-y%8);                 // read data from display memory
    data = uc1638ReadScreenData();

    if( color == BLACK )
    {
        data |= 0x01 << (y%8);              // set dot
    }
    else
    {
        data &= ~(0x01 << (y%8));           // clear dot
    }

    uc1638WriteScreenData(data);                  // write data back to display
}

//
// Font Functions
//

uint8_t ks0xxx_ReadFontData(const uint8_t* ptr)
{
    return pgm_read_byte(ptr);
}

void ks0xxx_SelectFont(const uint8_t* font, ks0xxx_FontCallback callback, uint8_t color)
{
    ks0xxx_Font = font;
    ks0xxx_FontRead = callback;
    ks0xxx_FontColor = color;
}

int ks0xxx_PutChar(char c)
{
	uint8_t width = 0;
	uint8_t height = ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT);
	uint8_t bytes = height/8;//(height+7)/8;
	uint8_t firstChar = ks0xxx_FontRead(ks0xxx_Font+FONT_FIRST_CHAR);
	uint8_t charCount = ks0xxx_FontRead(ks0xxx_Font+FONT_CHAR_COUNT);
	uint16_t index = 0;
	uint8_t x = ks0xxx_Coord.x, y = ks0xxx_Coord.y;
	uint8_t i=0, j=0;
	uint8_t page, data;
	
	if((ks0xxx_FontRead(ks0xxx_Font+FONT_FIXED_WIDTH)==0x0A) && (ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT)==0x12))
	{
		
	}
	else
	{
		if(height%8)
		{
			bytes++;
		}
	}

	if( c < firstChar || c >= (firstChar+charCount) )
	{
		return 1;
	}
	c-= firstChar;

	// read width data, to get the index
	for( i=0; i<c; i++ )
	{
		index += ks0xxx_FontRead(ks0xxx_Font+FONT_WIDTH_TABLE+i);
	}
	
	if((ks0xxx_FontRead(ks0xxx_Font+FONT_FIXED_WIDTH)==0x0A) && (ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT)==0x12))
	{
		index = index*(bytes+1)+charCount+FONT_WIDTH_TABLE;
	}
	else
	{
		index = index*(bytes)+charCount+FONT_WIDTH_TABLE;
	}
	
    //uint8_t width = 0;
    //uint8_t height = ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT);
    //uint8_t bytes = (height+7)/8;
//
    //uint8_t firstChar = ks0xxx_FontRead(ks0xxx_Font+FONT_FIRST_CHAR);
    //uint8_t charCount = ks0xxx_FontRead(ks0xxx_Font+FONT_CHAR_COUNT);
//
    //uint16_t index = 0;
    //uint8_t x = ks0xxx_Coord.x, y = ks0xxx_Coord.y;
	//uint8_t i=0, j=0;
	//uint8_t page, data;
//
    //if( c < firstChar || c >= (firstChar+charCount) )
    //{
        //return 1;
    //}
    //c-= firstChar;
//
    //// read width data, to get the index
    //for( i=0; i<c; i++ )
    //{
        //index += ks0xxx_FontRead(ks0xxx_Font+FONT_WIDTH_TABLE+i);
    //}
    //index = index*bytes+charCount+FONT_WIDTH_TABLE;
    width = ks0xxx_FontRead(ks0xxx_Font+FONT_WIDTH_TABLE+c);

    // last but not least, draw the character
    for( i=0; i<bytes; i++ )
    {
         page = i*width;
        for(  j=0; j<width; j++ )
        {
            data = ks0xxx_FontRead(ks0xxx_Font+index+page+j);

            if( height < (i+1)*8 )
            {
                data >>= (i+1)*8-height;
            }

            if( ks0xxx_FontColor == BLACK )
            {
                uc1638WriteScreenData(data);
            }
            else
            {
                uc1638WriteScreenData(~data);
            }
        }
        // 1px gap between chars
        if( ks0xxx_FontColor == BLACK )
        {
            uc1638WriteScreenData(0x00);
        }
        else
        {
            uc1638WriteScreenData(0xFF);
        }
        uc1638_GotoXY(x, ks0xxx_Coord.y+8);
    }
    uc1638_GotoXY(x+width+1, y);

    return 0;
}

void ks0xxx_Puts(char* str)
{
    int x = ks0xxx_Coord.x;
    while( *str != 0 )
    {
        if( *str == '\n' )
        {
            uc1638_GotoXY(x, ks0xxx_Coord.y+ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT));
        }
        else
        {
            ks0xxx_PutChar(*str);
        }
        str++;
    }
}

void ks0xxx_Puts_P(PGM_P str)
{
    int x = ks0xxx_Coord.x;
    while( pgm_read_byte(str) != 0 )
    {
        if( pgm_read_byte(str) == '\n' )
        {
            uc1638_GotoXY(x, ks0xxx_Coord.y+ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT));
        }
        else
        {
            ks0xxx_PutChar(pgm_read_byte(str));
        }
        str++;
    }
}

uint8_t ks0xxx_FontHeight()
{
    return ks0xxx_FontRead(ks0xxx_Font+FONT_HEIGHT);
}

uint8_t ks0xxx_CharWidth(char c)
{
    uint8_t width = 0;
    uint8_t firstChar = ks0xxx_FontRead(ks0xxx_Font+FONT_FIRST_CHAR);
    uint8_t charCount = ks0xxx_FontRead(ks0xxx_Font+FONT_CHAR_COUNT);

    // read width data
    if( c >= firstChar && c < (firstChar+charCount) )
    {
        c -= firstChar;
        width = ks0xxx_FontRead(ks0xxx_Font+FONT_WIDTH_TABLE+c)+1;
    }

    return width;
}

uint16_t ks0xxx_StringWidth(char* str)
{
    uint16_t width = 0;

    while( *str != 0 )
    {
        width += ks0xxx_CharWidth(*str++);
    }

    return width;
}

uint16_t ks0xxx_StringWidth_P(PGM_P str)
{
    uint16_t width = 0;

    while( pgm_read_byte(str) != 0 )
    {
        width += ks0xxx_CharWidth(pgm_read_byte(str++));
    }

    return width;
}

void ks0xxx_GotoXY(uint8_t x, uint8_t y)
{
	uc1638_Set_X(x);
	uc1638_Set_Y(y);
}

void LoadBitmap(unsigned char *bitmap)
{
	uint16_t i=0, k=0;
	
	uc1638_GotoXY(0,0);
	uc1638WriteCommand(0x01);
	for(i=0; i<240*20; i++)
	{
		uc1638WriteData(pgm_read_byte(&bitmap[k++]));
	}
	uc1638_GotoXY(0,0);
}

void ks0xxx_Circle(uint8_t xcenter, uint8_t ycenter, uint8_t radius, uint8_t color)
{
    int tswitch, y, x = 0;
    unsigned char d;

    d = ycenter - xcenter;
    y = radius;
    tswitch = 3 - 2 * radius;
    while (x <= y) 
	{
        ks0xxx_SetDot(xcenter + x, ycenter + y, color);     
		ks0xxx_SetDot(xcenter + x, ycenter - y, color);
        
		ks0xxx_SetDot(xcenter - x, ycenter + y, color);     
		ks0xxx_SetDot(xcenter - x, ycenter - y, color);
        
		ks0xxx_SetDot(ycenter + y - d, ycenter + x, color); 
		ks0xxx_SetDot(ycenter + y - d, ycenter - x, color);

        ks0xxx_SetDot(ycenter - y - d, ycenter + x, color); 
		ks0xxx_SetDot(ycenter - y - d, ycenter - x, color);

        if (tswitch < 0) 
		    tswitch += (4 * x + 6);
        else 
		{
            tswitch += (4 * (x - y) + 10);
            y--;
        }
        x++;
    }
}

void uc1638_Init(void)
{
	ks0xxx_Coord.x = 0;
	ks0xxx_Coord.y = 0;
	ks0xxx_Coord.page = 0;

	RST_DIR_OUTPUT;
	RS_DIR_OUTPUT;
	CS_DIR_OUTPUT;
	RDB_DIR_OUTPUT;
	WRD_DIR_OUTPUT;
	LCD_DATA_DIR_OUT;
	
	//system reset
	RST_LOW;
	_delay_ms(50);//100ms
	RST_HIGH;
	_delay_ms(200);//Delay more than 150ms.
	
	uc1638WriteCommand(0xe1);//system reset
	uc1638WriteData(0xe2);
	_delay_ms(10);
	
	uc1638WriteCommand(0x04);		//set colume Address
	uc1638WriteData(0x00);			//
	
	uc1638WriteCommand(0xEB);		//set bias=1/12
	
	uc1638WriteCommand(0x81);		//set vop
	//uc1638WriteData(130);			//  pm=106 Set VLCD=15V
    uc1638WriteData(138);			//  pm=106 Set VLCD=15V
	
	uc1638WriteCommand(0xB8);		//ÆÁ±ÎMTP
	uc1638WriteData(0x00);

	uc1638WriteCommand(0xC4);		//set lcd mapping control
	uc1638WriteCommand(0xA3);		//set line rate  20klps
	uc1638WriteCommand(0x95);		// PT0   1B P P
	
	uc1638WriteCommand(0xf1);		//set com end
	uc1638WriteData(159);			//set com end   240*128
	
	uc1638WriteCommand(0xC2);
	uc1638WriteCommand(0x31);		//APC
	uc1638WriteData(0X91);			// 1/0: sys_LRM_EN disable
	
	uc1638WriteCommand(0xC9);
	uc1638WriteData(0xAD);			//  display

	uc1638_FillScreen(BLANK1);                            // display clear
	uc1638_GotoXY(0,0);
}

void uc1638WriteCommand(uint8_t cmd)
{
	CS_LOW;
	RS_LOW;
	WRD_LOW;
	RDB_HIGH;
	
	LCD_DATA_OUT = cmd;
	
	CS_HIGH;
}

void uc1638WriteData(uint8_t cmd)
{
	CS_LOW;
	RS_HIGH;
	WRD_LOW;
	RDB_HIGH;
	
	LCD_DATA_OUT = cmd;
	
	CS_HIGH;
}

void uc1638WriteScreenData(uint8_t data)
{
	uc1638WriteCommand(0x01);
	uc1638WriteData(data);
}

uint8_t uc1638ReadData(void)
{
	uint8_t readbyte=0;
	
	CS_LOW;
	RS_LOW;
	WRD_HIGH;
	RDB_LOW;
	
	LCD_DATA_DIR_IN;
	_delay_us(100);//100ms
	
	readbyte = LCD_DATA_IN;
	
	LCD_DATA_DIR_OUT;
	
	CS_HIGH;
	
	return readbyte;
}

uint8_t uc1638ReadScreenData(void)
{
	uc1638WriteCommand(0x02);
	return uc1638ReadData();
}

/**
 * @brief Glcd Set X Position.
 * Sets x-axis position to x_pos dots from the left border of Glcd within the 
 * selected side.
 * @param x_pos Position on x-axis. Valid values: 0..63
 */
void uc1638_Set_X(uint8_t x_pos)
{
	// Boundary Checks
	if( x_pos >= GLCD_PIXEL_X ) x_pos = 0;                              // ensure that coordinates are legal
	ks0xxx_Coord.x = x_pos;                              // save new coordinates
	
	uc1638WriteCommand(0x04);   	//set column address
	uc1638WriteData(x_pos);
}

/**
 * @brief Glcd Set Page.
 * Selects the page on Glcd.
 * @param page Page Number. Valid values: 0..7
 */
void uc1638_Set_Y(uint8_t y_pos)
{
    // Boundary Checks
	if( y_pos >= GLCD_PIXEL_Y ) y_pos = 0;               // ensure that coordinates are legal
	ks0xxx_Coord.y = y_pos; 
	ks0xxx_Coord.page = y_pos/8;                             // save new coordinates
	
    uc1638WriteCommand(0x60 | (ks0xxx_Coord.page & 0xf));   	//set page address /lsb
    uc1638WriteCommand(0x70 | (ks0xxx_Coord.page >> 4));   	    //set page address /msb
}

void uc1638_GotoXY(uint8_t x, uint8_t y)
{
	uc1638_Set_X(x);
	uc1638_Set_Y(y);
}

void uc1638_FillScreen(uint8_t pattern)
{
	uint16_t i=0;
	
	uc1638_GotoXY(0,0);
	
	uc1638WriteCommand(0x01);
	for(i=0; i<240*20; i++)
	{
		if( !pattern )
			uc1638WriteData(0x00);       // erase a column
		else
			uc1638WriteData(0xFF);       // fill a column
	}

	uc1638_GotoXY(0,0);
}


