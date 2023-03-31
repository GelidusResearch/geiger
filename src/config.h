/**
 *	File:       	config.h
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	Config header for Geiger.Counter.1 firmware
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
#ifndef F_CPU
#define				F_CPU					8000000UL	// 8MHz internal clock - only set if the env does not already do it
#endif

#define				DEV_CLASS				0x1
#define				VER_SW					2.01
#define				VER_HW					1.0
#define				UNITID					0x20000000
#define				MODEL_GRGC1				


// constants for this device
#define				P_TUBEVOLFB_PIN			PC2		// voltage divider input adc pin
#define				P_TUBEVOLFB_R1			10000	// voltage divider resistance used to measure high voltage 10M Ohm
#define				P_TUBEVOLFB_R2			47		// voltage divider to measure high voltage 47K Ohm
#define				GEIGER_TUBE				GEIGER_TUBE_SBM20 //Default Tube
#define				WARMUP					0		// initial warm up interval before sensors are ready, use only if needed
#define				READ_INTERVAL			5		// interval in seconds to read sensors

// user interaction
#define				LONG_PRESS				64	// RTC clock ticks on central button counts as a long press ~ .5 sec
#define				SHORT_PRESS				16
#define				HOLD_PRESS				64 * 16 // Hold is longer that 8 sec
#define 			BACKLIGHT_TIMEOUT		15		// turn back light in 15 seconds of inactivity to conserve power


// geiger tube radiation dose thresholds
#define 			DOSE_THRESHOLD_LOW 		0.11
#define 			DOSE_THRESHOLD_NORMAL 	0.21
#define 			DOSE_THRESHOLD_HIGH 	1.01

//nvram allocation
#define				EEPROM_ADDR_BEGIN		0x00
#define				EEPROM_ADDR_END			0x30  //Startup struct 48 bytes

// inverter control
#define INVERTER_DUTY_MIN		50			// inverter duty cycle will not go below 5%
#define INVERTER_DUTY_MAX		900 		// inverter duty cycle will not go above 90%
#define INVERTER_FREQUENCY		7000UL		// inverter frequency in Hertz
#define INVERTER_THRESHOLD		400			// inverter output voltage target
#define INVERTER_TOLERANCE		2			// tolerance for adjusting the duty cycle to control the voltage

#define SERIAL_TX_BUFFER_SIZE 168
#define SERIAL_RX_BUFFER_SIZE 96

// log settings
#define LOG_FILENAME "gc1.log"
enum {
	UI_LOG_INTERVAL_1S,
	UI_LOG_INTERVAL_1M,
	UI_LOG_INTERVAL_5M,
	UI_LOG_INTERVAL_15M,
	UI_LOG_INTERVAL_30M,
	UI_LOG_INTERVAL_1H,
	UI_LOG_INTERVAL_2H,
	UI_LOG_INTERVAL_4H,
	UI_LOG_INTERVAL_8H,
	UI_LOG_INTERVAL_12H,
	UI_LOG_INTERVAL_24H,
};


#define CLOCK_12MHZ
#define	JSMN_STRICT

//**********FUSES***************
//BODLEVEL = DISABLED
//RSTDISBL = [ ]
//DWEN = [ ]
//SPIEN = [X]
//WDTON = [ ]
//EESAVE = [ ]
//BOOTSZ = 2048W_3800
//BOOTRST = [ ]
//CKDIV8 = [X]
//CKOUT = [ ]
//SUT_CKSEL = INTRCOSC_8MHZ_6CK_14CK_65MS

//EXTENDED = 0xFF
//HIGH = 0xD9
//LOW = 0xE2


//AT+UART_DEF=38400,8,1,0,1
//avrdude -c stk500v2 -p m328p -V -P COMn -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xfd:m
//avrdude -c stk500v2 -p m328p -V -P COMn
