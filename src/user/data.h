/**
 *	File:       	data.h
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	Data handler and interfaces
 *	Project:		Geiger.Counter.1, Logging, USB Interface and low power field operations.
 *
 *	Copyright 2016 by Radu Motisan, radu.motisan@gmail.com
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

#include "../config.h"
#include "../misc/utils.h"
#include "../time/rtc.h"
#include "../lcd/5110.h"
#include "../misc/nvm.h"
#include <avr/pgmspace.h>

#define		FIELD_TYPE_STRING		0
#define		FIELD_TYPE_INT			1
#define		FIELD_TYPE_TIME			2
#define		FIELD_TYPE_DATE			3
#define		FIELD_TYPE_SYMBOL		4
#define		IFIELD_MAX				5

class DATA {

	public:
		struct LCDFIELD {
			uint8_t line_number;					//line number is equivalent to y position
			uint8_t field_start;					//field start is equivalent to x start character position
			uint8_t field_end;						//field end is equivalent to x end character position
			uint8_t field_type;						//field definitions - see define header
			uint8_t field_position;					//current cursor position in the field
			uint8_t field_data[LCD_LINE_WIDTH];		//buffer of characters up to the LCD line width
			};

		struct MENU {
			uint8_t num_fields;
			LCDFIELD fields[5];
			};

		struct STATE {
			uint8_t ButtonPressed1;
			uint8_t ButtonPressed2;
			uint8_t Alarm;
			uint8_t AlarmSilence;
			uint8_t Beep;
			uint8_t Read;
			uint8_t Sleep;
			uint8_t Edit;
			uint8_t xScreenPos = 0;
			uint8_t yScreenPos = 0;
			uint16_t batteryVoltage;				// voltage on battery in millivolts
			uint16_t inverterVoltage;				// high voltage produced by the inverter in volts
			uint16_t inverterDuty;					// inverter driver signal duty cycle in percents per millivolt
			uint32_t geigerCPM;						// radiation count as Counts Per Minute CPM
			double geigerDose;						// radiation dose approximated to equivalent dose based on Geiger tube factor
			//uint32_t alarmTimeStamp;
		};

		MENU menu[3];
		STATE State;
		void initFields();
		// read sensors Fast: called with a very high frequency
		void readSensorsFast();


};
