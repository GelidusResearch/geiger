/**
 *	File:       	pins.cpp
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	I/O control general class for AVR micro controllers
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

#include "pins.h"

void pin::config(Type type) {
	if (type == INPUT) {
		*PORT2DDR(m_pport) &= mask0;
	} else {
		*PORT2DDR(m_pport) |= mask1;
	}
}

// Set the output, specified as 0 or 1 (int)
void pin::write(bool state) {
	state? *m_pport |= mask1 : *m_pport &= mask0;
	m_state = state;
}

// Return the output setting, represented as 0 or 1 (bool)
bool pin::read() {
	if (m_type == OUTPUT)
		return m_state;
	else
		return *PORT2PIN(m_pport) & mask1;
}

// negate state
bool pin::toggle() {
	bool state = !read();
	write(state);
	return state;
}

