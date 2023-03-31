/**
 *	File:       	timecounter.h
 *	Version:  		1.0
 *	Date:       	2013
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

#pragma once

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

typedef void (*TimeCallback)();


class TimeCounter {

	TimeCallback	ptrFuncSec,	// called when a full second has elapsed
					ptrFuncMin;	// a full minute has elapsed
	public:

	TimeCounter() {
		ptrFuncSec = 0; ptrFuncMin = 0;
	}

	void init(TimeCallback callSec, TimeCallback callMin);
	void TimerEvent();

};


