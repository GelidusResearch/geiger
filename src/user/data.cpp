/**
 *	File:       	data.cpp
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
#include "../user/data.h"
#include "../lcd/5110.h"
#include "../geiger/detectors.h"
extern NVMConfig Running;
extern bool inv_adjustDutyCycle(uint16_t, uint16_t);

void DATA::initFields() {
	menu[0].num_fields = 2;
	menu[0].fields[0].line_number = 0;
	menu[0].fields[0].field_start = 0;
	menu[0].fields[0].field_end = 7;
	menu[0].fields[0].field_type = FIELD_TYPE_TIME;

	menu[0].fields[1].line_number = 0;
	menu[0].fields[1].field_start = 10;
	menu[0].fields[1].field_end = 10;
	menu[0].fields[1].field_type = FIELD_TYPE_SYMBOL;

	menu[0].fields[2].line_number = 1;
	menu[0].fields[2].field_start = 0;
	menu[0].fields[2].field_end = 9;
	menu[0].fields[2].field_type = FIELD_TYPE_DATE;

	menu[1].num_fields = 2;
	menu[1].fields[0].line_number = 1;
	menu[1].fields[0].field_start = 6;
	menu[1].fields[0].field_end = 13;
	menu[1].fields[0].field_type = FIELD_TYPE_INT;

	menu[1].fields[1].line_number = 3;
	menu[1].fields[1].field_start = 6;
	menu[1].fields[1].field_end = 13;
	menu[1].fields[1].field_type = FIELD_TYPE_STRING;

	//wifi state
	menu[1].fields[2].line_number = 5;
	menu[1].fields[2].field_start = 6;
	menu[1].fields[2].field_end = 13;
	menu[1].fields[2].field_type = FIELD_TYPE_STRING;

	//log state
	menu[2].num_fields = 2;
	menu[2].fields[0].line_number = 0;
	menu[2].fields[0].field_start = 11;
	menu[2].fields[0].field_end = 13;
	menu[2].fields[0].field_type = FIELD_TYPE_STRING;

	//log interval
	menu[2].fields[1].line_number = 1;
	menu[2].fields[1].field_start = 11;
	menu[2].fields[1].field_end = 13;
	menu[2].fields[1].field_type = FIELD_TYPE_STRING;

	//sdcard
	menu[2].fields[2].line_number = 2;
	menu[2].fields[2].field_start = 10;
	menu[2].fields[2].field_end = 13;
	menu[2].fields[2].field_type = FIELD_TYPE_STRING;


	for (uint8_t j = 0; j<=2 ; j++) {
		for (uint8_t i = 0; i<3 ; i++) {
		menu[j].fields[i].field_position = menu[j].fields[i].field_start;
		}
	}

}
// 	deviceID = id;
// 	localMac[0] = 0x2; // locally administered
// 	localMac[1] = 0x0;
// 	localMac[2] = (uint8_t)((deviceID >> 24) & 0xFF);
// 	localMac[3] = (uint8_t)((deviceID >> 16) & 0xFF);
// 	localMac[4] = (uint8_t)((deviceID >>  8) & 0xFF);
// 	localMac[5] = (uint8_t)((deviceID      ) & 0xFF);


void DATA::readSensorsFast() {
	// battery (AVCC) voltage in millivolts
	State.batteryVoltage = aux_readIntRefVcc();
	// read inverter voltage, via 10M/47K resistive divider, connected to pin ADC2, we use battery reference voltage
	State.inverterVoltage = (aux_readDivVoltage(State.batteryVoltage / 1000.0, P_TUBEVOLFB_R1, P_TUBEVOLFB_R2, P_TUBEVOLFB_PIN));
	// adjust duty cycle to regulate inverter output voltage, close to 	INVERTER_THRESHOLD
	inv_adjustDutyCycle(State.inverterVoltage, aux_detectorVoltage(Running.geigerTube));
}


