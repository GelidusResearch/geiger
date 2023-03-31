/**
 *	File:       	timecounter.cpp
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	time handling subsystem for AVR microcontrollers - implement a time counter
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
#include "timecounter.h"

// init function used to set a callback to be called every new minute
void  TimeCounter::init(TimeCallback callSec, TimeCallback callMin) {
	ptrFuncSec = callSec;
	ptrFuncMin = callMin;

	TIMSK0 |= (1 << TOIE0); //atmega168
	sei(); // enable interrupts
	TCCR0B |= _BV(CS01) | _BV(CS00); // set prescaler to 64 and start the timer
}




