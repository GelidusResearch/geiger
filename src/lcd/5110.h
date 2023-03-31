/**
 *	File:       	5110.h
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	LCD driver for Nokia 3110 / 5110 LCD
 *	Project:		Geiger.Counter.1, Logging, USB Interface and low power field operations.
 *
 *	Copyright 2007 by Tony Myatt
 *	Copyright 2013 by Radu Motisan, radu.motisan@gmail.com
 *	Copyright 2016 by Magnasci SRL, www.magnasci.com
 *  Copyright 2017 by Gelidus Research Inc, mike.laspina@gelidus.ca
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 * 	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <util/delay.h>
#include "../gpio/pins.h"

// define  Lcd screen size
#define LCD_WIDTH 				84
#define LCD_HEIGHT 				48
#define CHAR_WIDTH 				6		// Character cell pixel width
#define CHAR_HEIGHT 			8		// Character cell pixel height
#define LCD_LINES 				6		// Array 6 lines of characters in height
#define LCD_LINE_WIDTH			14		// Array 14 characters of width

#define LCD_SETXADDR			0x80
#define LCD_SETYADDR			0x40
#define LCD_EXTENDED_COMMANDS	0x21	// LCD Extended Commands
#define LCD_VOP					0x00	// Set LCD Vop(Contrast) 3.0v
//#define LCD_VOP					0xC8	// Set LCD Vop(Contrast)
#define LCD_TEMP				0x06	// Set Temp coefficent
#define LCD_BIAS_MODE			0x13	// LCD bias mode 1:48
#define LCD_HOR_ADDRESSING		0x20	// Standard Commands, Horizontal addressing
#define LCD_NORMAL_MODE			0x0C	// LCD in normal mode
#define LCD_SLEEP_MODE			0x24	// LCD in sleep mode
#define LCD_WAKE_MODE			0x20	// LCD in sleep mode

class LCD_5110 {
	enum Type { LCD_CMD  = 0, LCD_DATA = 1 };

	uint16_t lcdCacheIdx;

	pin *m_rst, *m_ce, *m_dc, *m_data, *m_clk, *m_backlight;

public:
	LCD_5110(pin *rst, pin *ce, pin *dc, pin *data, pin *clk, pin *backlight);
	void init();
	void sleep(void);
	void wake(void);
	void contrast(uint8_t contrast);
	void clear(void);
	void clear_area(uint8_t line, uint8_t startX, uint8_t endX);
	void clear_line(uint8_t line);
	void goto_xy(uint8_t x, uint8_t y);
	void goto_xy_exact(uint8_t x, uint8_t y);
	void send(char chr);
	void send(char* str);
	void send(const char* str);
	void send(char *buffer, uint16_t len, const char *szFormat, ...);
	void send(uint8_t byte, Type cd);
	void base_addr(uint16_t addr) ;
	void col(char chr);
	void pixelBack(void);
	void printPictureOnLCD ( const uint8_t *data);
	void drawPixel(uint8_t  x, uint8_t  y, int color);
	void setBacklight(bool mode);
	bool getBacklight(void);
};



