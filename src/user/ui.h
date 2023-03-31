/**
 *	File:       	ui.h
 *	Version:  		1.0
 *	Date:       	2016
 *	License:		GPL v3
 *	Description:	User interface
 *	Project:		Geiger.Counter.1, Logging, USB Interface and low power field operations.
 *
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

#include <stdint.h>
#include "../lcd/5110.h"
#include "../geiger/detectors.h"
#include "../config.h"
#include "../user/data.h"

class UI {
	enum {
		UI_PAGE_HOME = 0,
		UI_PAGE_STATS = 1,
		UI_PAGE_CONFIG = 2
	};
	enum {
		BUTTON_NOPRESS = 0,
		BUTTON_PRESS_SHORT = 1,
		BUTTON_PRESS_LONG = 2,
		BUTTON_PRESS_HOLD = 3,
	};
	LCD_5110 *m_lcd;
	pin *m_ch_pd, *m_speaker, *m_button1, *m_button2;
	DATA *m_data;
	char *m_buffer;
	uint8_t m_page = 2;
	uint16_t m_size;

public:

	int runtickcount;
	// The GUI is an interface between user via the LCD, and Data
	UI(LCD_5110 *lcd, pin *ch_pd, pin *speaker, pin *button1, pin *button2, DATA *Data, char *buffer, uint16_t size);
	char *getSymbolBattery(uint16_t voltage);	// returns the battery symbol based on voltage level, given in millivolts
	float factorDose(float dose);				// to save display space, we use multipliers
	char factorDoseSymbol(float dose);			// get multiplier symbol
	uint16_t factorCPM(uint32_t cpm);			// apply multiplication factor to CPM
	char factorCPMSymbol(uint32_t cpm);			// get multiplication factor symbol for CPM
	char* labelDose(float dose);				// get string label for various dose thresholds
	char* labelInterval(uint8_t interval);		// get string label for interval name
	char* labelMode(uint8_t mode);				// get string for mode value
	void drawPage();
	void loop(bool *refresh);
	void moveCursor();
	void setScreenPos(uint8_t xPos,uint8_t yPos);
	void setTimeField();
	void setDateField();
	void setMacField();
	void editMacBits(uint8_t nibblePos);

};
