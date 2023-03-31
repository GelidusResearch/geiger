/**
 *	File:       	nvm.h
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	Non volatile memory object class
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

#pragma once

#include <avr/io.h>
#include <string.h>
#include <avr/eeprom.h>


/************************************************************************/
/* Configuration                                                        */
/************************************************************************/


struct NVMConfig {
	uint8_t speakerState;
	uint8_t loggingState;
	uint8_t alarmState;
	uint8_t logInterval;
	uint8_t geigerTube;
	uint8_t calibration;
	uint8_t WifiState;
	uint8_t Mode;
	char SSID[16];
	uint32_t alarmTimeStamp;
	uint32_t unitID;
};
