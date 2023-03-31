/*
 *	File:       	rtc.h
 *	Version:  		1.0
 *	Date:       	Nov 10,2017
 *	License:		GPL v3
 *	Description:	Real-Time Clock for atmega async capable chips e.g 328 
 *	Project:		GC1, Logging, USB Interface and low power field operations.
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

#ifndef RTC_H
#define RTC_H

//library version
#define RTC_VERSION 100
#include <avr/sfr_defs.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/common.h>

void timeEvent(void);
void setTimer(void);
void stopRTC(void);
void startRTC(void);
uint8_t not_leap(void);

typedef struct{
	uint8_t second; //enter the current time, date, month, and year
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	uint8_t counter;
	uint32_t ticks;
	uint32_t runticks;
	bool onSecInterval;
	bool onMinInterval;
	bool onHrInterval;
	bool onDayInterval;
	bool isRunning;
} time;

#endif


