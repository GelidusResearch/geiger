/**
 *	File:       	detectors.cpp
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	Define Geiger tubes and conversion factors for CPM to equivalent dose
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

#include "detectors.h"

const char *aux_detectorName(uint8_t param) {
	switch (param) {
		case GEIGER_TUBE_SBM20:  return "SBM20";
		case GEIGER_TUBE_SBM20M: return "SBM20M";
		case GEIGER_TUBE_SI29BG: return "SI29BG";
		case GEIGER_TUBE_SBM19:  return "SBM19";
		case GEIGER_TUBE_LND712: return "LND712";
		case GEIGER_TUBE_STS5:   return "STS5";
		case GEIGER_TUBE_SI22G:  return "SI22G";
		case GEIGER_TUBE_SI3BG:  return "SI3BG";
		case GEIGER_TUBE_SBM21:  return "SBM21";
		case GEIGER_TUBE_SBT9:   return "SBT9";
		case GEIGER_TUBE_SI1G:   return "SI1G";
		case GEIGER_TUBE_J305:   return "J305";
		case GEIGER_TUBE_M4011:   return "M4011";
		default: return "Unknown";
	}
}


float aux_detectorFactor(uint8_t param) {
	float factor = 0;
	switch (param) {
		case GEIGER_TUBE_SBM20:
		 	factor =  0.005700;
			break;
		case GEIGER_TUBE_SI29BG:
		 	factor =  0.010000;
			break;
		case GEIGER_TUBE_SBM19:
		 	factor =  0.001500;
			break;
		case GEIGER_TUBE_STS5:
		 	factor =  0.006666;
			break;
		case GEIGER_TUBE_SI22G:
		 	factor =  0.081438;
			break;
		case GEIGER_TUBE_SI3BG:
		 	factor =  0.631578;
			break;
		case GEIGER_TUBE_SBM21:
		 	factor =  0.048000;
			break;
		case GEIGER_TUBE_LND712:
			factor =  0.010000;
			break;
		case GEIGER_TUBE_SI1G:
			factor =  0.006000;
			break;
		case GEIGER_TUBE_J305:
			factor =  0.008120;
			break;
		case GEIGER_TUBE_M4011:
			factor =  0.006550;
			break;
	}
	return factor;
}

uint16_t aux_detectorVoltage(uint8_t param) {

	uint16_t voltage = 0;
	switch (param) {
		case GEIGER_TUBE_SBM20:
			voltage = 400;
			break;
		case GEIGER_TUBE_LND712:
			voltage = 450;
			break;
		case GEIGER_TUBE_J305:
			voltage = 380;
			break;
		case GEIGER_TUBE_M4011:
			voltage = 380;
			break;
		default:
			voltage = 400;
	}
	return voltage;
}

// dose equivalent linear approximative conversion from CPM
float aux_CPM2uSVh(uint8_t tube, uint32_t cpm) {
	return aux_detectorFactor(tube) * cpm;
}
