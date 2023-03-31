/**
 *	File:       	logger.h
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	Data handler and interfaces
 *	Project:		Geiger.Counter.1, Logging, USB Interface and low power field operations.
 *
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


#define SD_CS_ASSERT PORTB &= ~0x01
#define SD_CS_DEASSERT PORTB |= 0x01
#define CS (1<<PB2)
#define MOSI (1<<PB3)
#define MISO (1<<PB4)
#define SCK (1<<PB5)
#define CS_DDR DDRB
#define CS_ENABLE() (PORTB &= ~CS)
#define CS_DISABLE() (PORTB |= CS)

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message

class Logger {

public:
	enum {
		DISABLED = 0,
		ENABLED = 1
	};

// 	Logs(RTC *time);
// 	// return objects
// 	RTC* getTimeEvent();
	
	// state access calls
	bool getStateLogs();
	void setStateLogs(bool state);

};

