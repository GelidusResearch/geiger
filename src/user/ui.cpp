/**
 *	File:       	ui.cpp
 *	Version:  		1.0
 *	Date:       	2016
 *	License:		GPL v3
 *	Description:	User interface
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
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>
 */



/*
Menu 0

             F1                   F2
0  [0  1  2  3  4  5  6  7] 8  9 [10] 11  12  13
                F3
1  [0  1  2  3  4  5  6  7  8  9] 10  11  12  13

2   0  1  2  3  4  5  6  7  8  9  10  11  12  13

3   0  1  2  3  4  5  6  7  8  9  10  11  12  13

4   0  1  2  3  4  5  6  7  8  9  10  11  12  13

5   0  1  2  3  4  5  6  7  8  9  10  11  12  13

Menu 1


0   0  1  2  3  4  5  6  7  8  9  10  11  12  13

1   0  1  2  3  4  5  6  7  8  9  10  11  12  13

2   0  1  2  3  4  5  6  7  8  9  10  11  12  13

3   0  1  2  3  4  5  6  7  8  9  10  11  12  13

4   0  1  2  3  4  5  6  7  8  9  10  11  12  13

5   0  1  2  3  4  5  6  7  8  9  10  11  12  13

Menu 2


0   0  1  2  3  4  5  6  7  8  9  10  11  12  13

1   0  1  2  3  4  5  6  7  8  9  10  11  12  13

2   0  1  2  3  4  5  6  7  8  9  10  11  12  13

3   0  1  2  3  4  5  6  7  8  9  10  11  12  13

4   0  1  2  3  4  5  6  7  8  9  10  11  12  13

5   0  1  2  3  4  5  6  7  8  9  10  11  12  13

*/



#include "../user/ui.h"
#include <avr/pgmspace.h>

uint8_t ifield = 0;
uint8_t field_position = 0;
uint8_t field_start = 0;
uint8_t field_end = 0;
uint8_t positionT;
uint8_t xPos = 0;
uint8_t yPos = 0;
uint8_t interval;
uint16_t tdvar1;
uint8_t tdvar2;
uint8_t tdvar1_tens;
uint8_t tdvar1_ones;
uint8_t	tdvar1_huns;
uint8_t tdvar1_thos;
uint16_t buttonticks = 0;
bool enteringEdit = true;
extern uint8_t editMode;
extern NVMConfig Running;
extern bool onConfigChange;
extern bool softreset;
extern time rtc;
extern uint8_t ip[4];
extern bool WifiActive;



UI::UI(LCD_5110 *lcd, pin *ch_pd,pin *speaker, pin *button1, pin *button2, DATA *Data, char *buffer, uint16_t size) {
	m_page = UI_PAGE_HOME;
	m_lcd = lcd;
	m_ch_pd = ch_pd;
	m_speaker = speaker;
	m_button1 = button1;
	m_button2 = button2;
	m_data = Data;
	m_buffer = buffer;
	m_size = size;
}

// return 4 char LCD battery symbol
 char* UI::getSymbolBattery(uint16_t voltage) {
	 static char battery[4][4] = {{0x81,0x82,0x83,0},{0x84,0x85,0x86,0},{0x87,0x88,0x89,0},{0x8A,0x8B,0x8C,0} };
	 if (voltage > 3800) return battery[0]; //charging
	 else if (voltage > 3300) return battery[1]; //full
	 else if (voltage > 3200) return battery[2]; //half
	 else return battery[3]; //empty
}

 // to save display space, we use multipliers
 float UI::factorDose(float dose) {
	 if (dose < 10) return dose; // show in micros
	 else if (dose < 10000) return dose / 1000.0; // show in millis
	 else return dose / 1000000.0; // show in units
 }

 // get multiplier symbol
 char UI::factorDoseSymbol(float dose) {
	 if (dose < 10) return 0x8D; // show in micros
	 else if (dose < 10000) return 'm'; // show in millis
	 else return ' '; // show in units
 }

 // apply multiplication factor to CPM
 uint16_t UI::factorCPM(uint32_t cpm) {
	 if (cpm < 1000) return (uint16_t)cpm;
	 else if (cpm < 1000000) return cpm / 1000;
	 else return cpm / 1000000UL;
 }

 // get multiplication factor symbol for CPM
 char UI::factorCPMSymbol(uint32_t cpm) {
	 if (cpm < 1000) return ' ';
	 else if (cpm < 1000000) return 'K';
	 else return 'M';
 }

 // get string label for various dose thresholds
 char* UI::labelDose(float dose) {
	 static char label[10];
	 if (dose < DOSE_THRESHOLD_LOW) strcpy_P(label, PSTR("   LOW    "));
	 else if (dose < DOSE_THRESHOLD_NORMAL) strcpy_P(label, PSTR("  NORMAL  "));
	 else if (dose < DOSE_THRESHOLD_HIGH) strcpy_P(label, PSTR("   HIGH   "));
	 else strcpy_P(label, PSTR("  DANGER  "));
	 return label;
 }

// get string label for various intervals
char* UI::labelInterval(uint8_t interval) {
	static char label[3];

	switch (Running.logInterval) {
		case 0:
			strcpy_P(label, PSTR(" 1s"));
			break;
		case 1:
			strcpy_P(label, PSTR(" 1m"));
			break;
		case 2:
			strcpy_P(label, PSTR(" 5m"));
			break;
		case 3:
			strcpy_P(label, PSTR("15m"));
			break;
		case 4:
			strcpy_P(label, PSTR("30m"));
			break;
		case 5:
			strcpy_P(label, PSTR(" 1h"));
			break;
		case 6:
			strcpy_P(label, PSTR(" 2h"));
			break;
		case 7:
			strcpy_P(label, PSTR(" 4h"));
			break;
		case 8:
			strcpy_P(label, PSTR(" 8h"));
			break;
		case 9:
			strcpy_P(label, PSTR("12h"));
			break;
		case 10:
			strcpy_P(label, PSTR("24h"));
			break;
			default: break;
		}
	return label;
}

// get string label for operational mode
char* UI::labelMode(uint8_t mode) {
	static char label[3];

	switch (mode) {
		case 0:
			strcpy_P(label, PSTR("ESPHome"));
			break;
		case 1:
			strcpy_P(label, PSTR(" Online"));
			break;
		case 2:
			strcpy_P(label, PSTR("Offline"));
			break;
			default: break;
		}
	return label;
}

void UI::setTimeField() {

	if ((m_page == 0) && (ifield == 0)) {

		field_position = m_data->menu[0].fields[0].field_position;

		if (field_position == 0) {

			tdvar1 = rtc.hour;
			if (tdvar1 < 14  && tdvar1 >= 0  ) {
				tdvar1 = tdvar1+10;
				rtc.hour = tdvar1;
			} else {
				if (tdvar1 >= 20 ) {
					tdvar1 = tdvar1 - 20;
					rtc.hour = tdvar1;
				} else {
					tdvar1 = tdvar1 - 10;
					rtc.hour = tdvar1;
				}
			}
		}

		if (field_position == 1) {

			tdvar1 = rtc.hour;
			if ( tdvar1 == 23) {
				tdvar1 = 20;
				rtc.hour = tdvar1;
				} else if (tdvar1 == 19) {
				tdvar1 = 10;
				rtc.hour = tdvar1;
				} else if (tdvar1 == 9) {
				tdvar1 = 0;
				rtc.hour = tdvar1;
				} else {
				tdvar1++;
				rtc.hour = tdvar1;
			}
		}

		if (field_position == 3) {

			tdvar1 = rtc.minute;
			tdvar1_ones = tdvar1 % 10;
			if (tdvar1 < 50 ) {
				tdvar1 = tdvar1 + 10;
				rtc.minute = tdvar1;
			} else {
				rtc.minute = tdvar1_ones;
			}
		}
		if (field_position == 4) {

			tdvar1 = rtc.minute;
			tdvar1_tens = tdvar1/10 % 10;
			tdvar1_ones = tdvar1 % 10;
			if (tdvar1_ones < 9 ) {
				tdvar1++;
				rtc.minute = tdvar1;
			} else {
				rtc.minute = tdvar1_tens*10;
			}
		}

		if (field_position == 6) {

			tdvar1 = rtc.second;
			tdvar1_ones = tdvar1 % 10;
			if (tdvar1 < 50 ) {
				tdvar1 = tdvar1 + 10;
				rtc.second = tdvar1;
			} else {
				rtc.second = tdvar1_ones;
			}
		}

		if (field_position == 7) {

			tdvar1 = rtc.second;
			tdvar1_tens = tdvar1/10 % 10;
			tdvar1_ones = tdvar1 % 10;
			if (tdvar1_ones < 9 ) {
				tdvar1++;
				rtc.second = tdvar1;
			} else {
				rtc.second = tdvar1_tens * 10;
			}
		}
	}
}


void UI::setDateField() {
	if ((m_page == 0) && (ifield == 2))
	{
		field_position = m_data->menu[0].fields[2].field_position;
		if (field_position == 0) {
			tdvar1 = rtc.day;
			tdvar2 = rtc.month;
			tdvar1_ones = tdvar1 % 10;
			if ((tdvar2 == 4) || (tdvar2 == 6) || (tdvar2 == 9) || (tdvar2 == 11)) { //30 days
				if (tdvar1 > 0 && tdvar1 < 21 ) {
					tdvar1 = tdvar1 + 10;
					rtc.day = tdvar1;
				} else {
					tdvar1 = 0 + tdvar1_ones;
					if (tdvar1 == 0) tdvar1++;
					rtc.day = tdvar1;
				}
			} else if (tdvar2 == 2) {
				if (!not_leap()) {
					if (tdvar1 > 0 && tdvar1 < 19 ) {
						tdvar1 = tdvar1 + 10;
					} else {
						tdvar1 = 0 + tdvar1_ones;
					}
				} else {
					if (tdvar1 > 0 && tdvar1 < 20 ) {
						tdvar1 = tdvar1 + 10;
					} else {
						tdvar1 = 0 + tdvar1_ones;
					}
				}
				if (tdvar1 == 0) tdvar1++;
				rtc.day = tdvar1;
			} else {
				if (tdvar1 > 0 && tdvar1 < 22 ) {
					tdvar1 = tdvar1 + 10;
					rtc.day = tdvar1;
				} else {
					tdvar1 = 0 + tdvar1_ones;
					if (tdvar1 == 0) tdvar1++;
					rtc.day = tdvar1;
				}
			}
		}

		if (field_position == 1) {
			tdvar1 = rtc.day;
			tdvar2 = rtc.month;
			tdvar1_tens = (tdvar1/10) % 10;
			tdvar1_ones = tdvar1 % 10;
			if (tdvar2 == 2) {
				if (!not_leap()) {
					if (tdvar1_ones < 8 ) {
						tdvar1++;
						rtc.day = tdvar1;
					} else {
						if (tdvar1_tens == 0) {
							rtc.day = 1;
						} else {
							rtc.day = tdvar1_tens * 10;
						}
					}
				}
			} else {
				if (tdvar1_ones < 9 ) {
					tdvar1++;
					rtc.day = tdvar1;
				} else {
					if (tdvar1_tens == 0) {
						rtc.day = 1;
						} else {
						rtc.day = tdvar1_tens * 10;
					}
				}
			}
		}
		if ((field_position == 3) || (field_position == 4)) {

			tdvar1 = rtc.month;
			tdvar1_ones = tdvar1 % 10;
			tdvar1_tens = tdvar1/10 % 10;
			if (tdvar1 < 12) {
				tdvar1++;
				rtc.month = tdvar1;
			} else {
				tdvar1 = 1;
				rtc.month = tdvar1;
			}
		}

		if ((field_position > 5 ) && (field_position < 10)) {

			tdvar1 = rtc.year;
			tdvar1_thos = tdvar1/1000 % 10;
			tdvar1_huns = tdvar1/100 % 10;
			tdvar1_tens = tdvar1/10 % 10;
			tdvar1_ones = tdvar1 % 10;

			if (field_position == 6) {

				if (tdvar1_thos < 9 ) {
					tdvar1_thos++;
					} else {
					tdvar1_thos = 0;
				}
			}
			if (field_position == 7) {
				if (tdvar1_huns < 9 ) {
					tdvar1_huns++;
				} else {
					tdvar1_huns = 0;
				}
			}
			if (field_position == 8){
				if (tdvar1_tens < 9 ) {
					tdvar1_tens++;
				} else {
					tdvar1_tens = 0;
				}
			}
			if (field_position == 9) {
				if (tdvar1_ones < 9 ) {
					tdvar1_ones++;
				} else {
					tdvar1_ones = 0;
				}
			}

			tdvar1 = tdvar1_thos*1000;
			tdvar1 = tdvar1 + (tdvar1_huns*100);
			tdvar1 = tdvar1 + (tdvar1_tens*10);
			tdvar1 = tdvar1 + tdvar1_ones;
			rtc.year = tdvar1;
		}
	}
}

void UI::setMacField() {

	if ((m_page == 1) && (ifield == 0)) {

		field_position = m_data->menu[1].fields[0].field_position;
		switch (field_position) {

			case 6:
				editMacBits(8);
				break;
			case 7:
				editMacBits(7);
				break;
			case 8:
				editMacBits(6);
				break;
			case 9:
				editMacBits(5);
				break;
			case 10:
				editMacBits(4);
				break;
			case 11:
				editMacBits(3);
				break;
			case 12:
				editMacBits(2);
				break;
			case 13:
				editMacBits(1);
				break;
			default:
				break;
		}
	}
}

void UI::editMacBits(uint8_t nibblePos) {

	uint32_t tempID1;
	uint8_t shiftValue = nibblePos * 4;
	uint32_t bitMask= 0xF0000000;

	bitMask = (uint32_t)(bitMask >> (32 - shiftValue));
	tempID1 = Running.unitID & bitMask;

	tempID1 = (uint32_t)(tempID1 >> (shiftValue - 4));
	tempID1++;
	if (tempID1 > 0x0F) tempID1 = 0;
	tempID1 = (uint32_t)(tempID1 << (shiftValue - 4));
	bitMask = ~bitMask;
	Running.unitID = Running.unitID & bitMask;
	Running.unitID = Running.unitID | tempID1;
}

 void UI::drawPage() {

		m_lcd->goto_xy(0,0);
		switch (m_page) {
	 		case UI_PAGE_HOME:
	 		m_lcd->send(m_buffer, m_size,
			    #ifdef MODEL_GRGC2
				PSTR("%02d:%02d:%02d  %c%s"
					"%02u/%02u/%04u %3s"
					"%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
					"%c %4.2f %c%s %c"
					"%c%c%s%c%c"
					"%3dV %05d CPM"),
				// line 1: time, state symbols, speaker and battery
	 			rtc.hour, rtc.minute, rtc.second, (Running.speakerState == 1)?0x8E:0x8F,
				getSymbolBattery(m_data->State.batteryVoltage),
				// line 2 Date
				rtc.day, rtc.month, rtc.year,(m_data->State.Sleep ? "zzz" : ""),
				// line 3: Separator
				0x90,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x91,
				// line 4: dose
	 			0x92, factorDose(m_data->State.geigerDose), (m_data->State.geigerDose > 0?factorDoseSymbol(m_data->State.geigerDose):' '),(m_data->State.geigerDose > 0?"Sv/h":"Wait"),0x93,
				// line 5: dose label
	 			0x95,0x94, labelDose(m_data->State.geigerDose), 0x94,0x96,
				// line 6: voltage, duty, CPM
	 			m_data->State.inverterVoltage, factorCPM(m_data->State.geigerCPM));
				#else
				PSTR("%02d:%02d:%02d     %c"
					"%02u/%02u/%04u %3s"
					"%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
					"%c %4.2f %c%s %c"
					"%c%c%s%c%c"
					"%3dV %05d CPM"),
				// line 1: time, state symbols, speaker
	 			rtc.hour, rtc.minute, rtc.second, (Running.speakerState == 1)?0x8E:0x8F,
				// line 2 Date
				rtc.day, rtc.month, rtc.year,(m_data->State.Sleep ? "zzz" : ""),
				// line 3: Separator
				0x90,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x94,0x91,
				// line 4: dose
	 			0x92, factorDose(m_data->State.geigerDose), (m_data->State.geigerDose > 0?factorDoseSymbol(m_data->State.geigerDose):' '),(m_data->State.geigerDose > 0?"Sv/h":"Wait"),0x93,
				// line 5: dose label
	 			0x95,0x94, labelDose(m_data->State.geigerDose), 0x94,0x96,
				// line 6: voltage, duty, CPM
	 			m_data->State.inverterVoltage, factorCPM(m_data->State.geigerCPM));
				#endif
	 		break;
	 		case UI_PAGE_STATS:
				//if (Running.Mode > 0) {
					m_lcd->send(m_buffer, m_size, PSTR(
							"Model:   GRGC1"
							"ID:   %08lX"
							"Firmware: %4.2f"
							"Tube: %8s"
							"SSID: %8s"
							"Mode: %8s"),
						// line 1: Model
						// line 2: Device ID
						Running.unitID,
						// line 3: Firmware Version
						VER_SW,
						// line 4: Tube Type
						aux_detectorName(Running.geigerTube),
						// line 5:
						Running.SSID,
						// line 6:
						labelMode(Running.Mode));
				// } else {
				// 	m_lcd->send(m_buffer, m_size, PSTR(
				// 			"Model:  GR.GC1"
				// 			"ID:   %08lX"
				// 			"Firmware: %4.2f"
				// 			"Tube: %8s"
				// 			"Mode: %8s"),

				// 		Running.unitID,
				// 		VER_SW,
				// 		aux_detectorName(Running.geigerTube),
				// 		labelMode(Running.Mode));
				// }
 			break;
		case UI_PAGE_CONFIG:

			m_lcd->send(m_buffer, m_size, PSTR(
			"Logs: % 8s"
			"Interval:  % 3s"
			//"SDCard: %6s"
			"IP:           "
			"%d.%d.%d.%d   "
			"              "
			"              "),
			// line 1: Logging State
			(Running.loggingState ? "On" : "Off"),
			// line 2 - Log interval - 1m, 5m, 15m, 30m 1h, 2h, 4h, 8h, 12h, 24h,
			labelInterval(Running.logInterval),
			// line 3
			//IP
			// line 4:
			ip[0],ip[1],ip[2],ip[3]);
			// line 5
			// line 6
			break;
			}
 }



 void UI::moveCursor() {
	 m_lcd->goto_xy(m_data->State.xScreenPos,m_data->State.yScreenPos);
 }


 void UI::setScreenPos(uint8_t x, uint8_t y) {
	m_data->State.xScreenPos = x;
	m_data->State.yScreenPos = y;
 }


void UI::loop(bool *refresh) {

	// read sensors fast do directly, slow only on refresh (default once per second)
	m_data->readSensorsFast();

	// threshold to sound alarm reached, enable alarm and jump to page 0
	if (m_data->State.geigerDose > DOSE_THRESHOLD_HIGH) {
		if (!m_data->State.AlarmSilence) m_data->State.Alarm = true;
		} else {
			m_data->State.Alarm = false;
			if (rtc.runticks > (uint32_t(128) * uint32_t(300))) {
				m_data->State.AlarmSilence = false;
			}
	}

	if (!*m_button1 && !*m_button2) { // both buttons down
		rtc.runticks = 0;
		while (!*m_button1 && !*m_button2);
		if ((rtc.runticks > SHORT_PRESS) && (rtc.runticks < HOLD_PRESS)){
				m_data->State.Sleep = !m_data->State.Sleep;
				m_data->State.ButtonPressed1 = false;
				m_data->State.ButtonPressed2 = false;
				_delay_ms(300);
				return;
		} else {
				if (rtc.runticks > HOLD_PRESS) softreset = true;
		}

	} else {

		if (!m_data->State.Sleep) {

			// handle button presses
			uint8_t button1 = BUTTON_NOPRESS;
			if (!m_data->State.ButtonPressed1 && !*m_button1) { // button down
				if (m_data->State.Alarm) {
					m_data->State.AlarmSilence = true;;
				}
				rtc.runticks = 0;
				m_data->State.ButtonPressed1 = true;
				} else if (m_data->State.ButtonPressed1 && *m_button1) { // button up
				m_data->State.ButtonPressed1 = false;
				// check for how long the button was pressed
				if (rtc.runticks > LONG_PRESS)
				button1 = BUTTON_PRESS_LONG;
				else
				button1 = BUTTON_PRESS_SHORT;
			}

			switch (button1) {
				case BUTTON_PRESS_SHORT:
					if (m_data->State.Edit) {				//if we are in edit mode

						m_data->menu[m_page].fields[ifield].field_position++;

						if (m_data->menu[m_page].fields[ifield].field_position > m_data->menu[m_page].fields[ifield].field_end) {
							m_data->menu[m_page].fields[ifield].field_position = m_data->menu[m_page].fields[ifield].field_start;

							if (ifield < m_data->menu[m_page].num_fields ) {
								ifield++;
								} else {
								ifield = 0;
							}

						} else {

						field_position = m_data->menu[m_page].fields[ifield].field_end - m_data->menu[m_page].fields[ifield].field_position;

						}

						switch (m_data->menu[m_page].fields[ifield].field_type) {
							case FIELD_TYPE_TIME:
								if (field_position == 2) 	m_data->menu[m_page].fields[ifield].field_position++;
								if (field_position == 5) 	m_data->menu[m_page].fields[ifield].field_position++;
								break;

							case FIELD_TYPE_DATE:
								if (field_position == 4) 	m_data->menu[m_page].fields[ifield].field_position++;
								if (field_position == 7) 	m_data->menu[m_page].fields[ifield].field_position++;
								break;

							case FIELD_TYPE_STRING:
								if (m_data->menu[m_page].fields[ifield].field_position > m_data->menu[m_page].fields[ifield].field_start) {

									m_data->menu[m_page].fields[ifield].field_position = m_data->menu[m_page].fields[ifield].field_start;

									if (ifield < m_data->menu[m_page].num_fields ) {
										ifield++;
										} else {
										ifield = 0;
									}
								}
								break;
							default: break;
							}
					} else {

					//advance to next page etc.
					//user present so quiet the alarm
					if (m_data->State.Alarm) {
						m_data->State.Alarm = false;
						*m_speaker = false;
						//m_data->setGeigerCPM(0); // reset readings, alarm might sound again
						} else {

						if (m_lcd->getBacklight() == false) {
							m_lcd->setBacklight(true);
						} else {
							m_lcd->setBacklight(true);
							// next page
							m_page = (m_page + 1) %3;
							m_lcd->clear();
						}
					}
					break;
				case BUTTON_PRESS_LONG:
					if (m_data->State.Edit) {

						// Disable edits
						m_data->State.Edit = false;
						onConfigChange = true;
					} else {

						// Enable edits
						m_data->State.Edit = true;
						m_data->initFields();

					}
				break;
				default: break;
				}
			}

			uint8_t button2 = BUTTON_NOPRESS;
			if (!m_data->State.ButtonPressed2 && !*m_button2) { // button down
				rtc.runticks = 0;
				m_data->State.ButtonPressed2 = true;
				} else if (m_data->State.ButtonPressed2 && *m_button2) { // button up
				m_data->State.ButtonPressed2 = false;

				// check for how long the button was pressed
				if (rtc.runticks > LONG_PRESS)
				button2 = BUTTON_PRESS_LONG;
				else
				button2 = BUTTON_PRESS_SHORT;
			}

			switch (button2) {
				case BUTTON_PRESS_SHORT:
					if (m_data->State.Edit) {

						if (m_page == 0 && ifield == 1 && m_data->menu[0].fields[1].field_position == 10) {
							Running.speakerState = !Running.speakerState;
						}

						if (m_page == 1 && ifield == 1 ) {

							if (Running.geigerTube < TUBE_COUNT) Running.geigerTube++;
							else Running.geigerTube = 0;
						}

						if (m_page == 1 && ifield == 2 ) {
							if (Running.Mode < 2) {
								Running.Mode++;
								} else {
								Running.Mode=0;
							}
						}

						if (m_page == 2 && ifield == 0 ) {
							Running.loggingState = !Running.loggingState;
						}
						if (m_page == 2 && ifield == 1 && m_data->menu[2].fields[1].field_position >= 11) {
							if (Running.logInterval < UI_LOG_INTERVAL_24H) {
								Running.logInterval++;
								} else {
								Running.logInterval=0;
							}
						}

						setTimeField();
						setDateField();
						setMacField();

					} else {
						m_lcd->clear();
					}

				break;
				case BUTTON_PRESS_LONG:
				if (m_data->State.Edit) {

				}
				break;
				default: break;
				}
			}


		if (m_data->State.Edit) {
			if (rtc.counter == 63) {

				setScreenPos(m_data->menu[m_page].fields[ifield].field_position,m_data->menu[m_page].fields[ifield].line_number);
				moveCursor();
				if (m_data->menu[m_page].fields[ifield].field_type == FIELD_TYPE_STRING) {
					positionT = m_data->menu[m_page].fields[ifield].field_end - m_data->menu[m_page].fields[ifield].field_start + 1;;
					for (uint8_t i = 0; i < positionT; i++ ) m_lcd->send(0x20);
				} else {
				m_lcd->send(0x20);
				}
			}
		}

		// turn backlight off when timeout is reached, only if alarm is off, because in alarm mode screen is flickering
		if (!m_data->State.Alarm && rtc.runticks > (256 * BACKLIGHT_TIMEOUT)) {
			m_lcd->setBacklight(false);
		}

		if (m_data->State.Beep) {
			if (m_page == 0) {
				m_lcd->goto_xy(9,0); m_lcd->send(0x97); // show radiation sign on page 0
				// do a beep if alarm if off but speaker is not muted
				if (!m_data->State.Alarm && Running.speakerState) {
					*m_speaker = true;
					_delay_ms(10);
					*m_speaker = false;
				}
			}
			// done, disable state
			m_data->State.Beep = false;
		}

		if (Running.WifiState == 1) {
			*m_ch_pd = 1;
		}
		else
		{
			*m_ch_pd = 0;
		}

		// handle GUI display
		if (*refresh) {
			// draw GUI
			drawPage();
		}
	}
	// done until next refresh event (next second)
	*refresh = 0;
	}
