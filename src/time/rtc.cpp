/**
 *	File:       	rtc.c
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



//use the real-time counter 32768 oscillator
#include "rtc.h"

extern volatile time rtc;

uint8_t not_leap(void) //check for leap year
{
	if (!(rtc.year%100))
	return (uint8_t)(rtc.year%400);
	else
	return (uint8_t)(rtc.year%4);
}

void setTimer(void) {

	//Timer 2 - RTC
	GTCCR |= (1<<PSRASY);						//Reset the async timer
	TIMSK2 &= ~(1<<TOIE2);						//During setup, disable all the interrupts on timer2
	TIMSK2 &= ~((1<<OCIE2A) | (1<<OCIE2B));
	TCNT2 = 0;									//Reset timer
	ASSR |= (1<<AS2);							//Set timer2 as asynchronous
	TCCR2B |= (1 << CS20);						//Prescale the timer2 to be clock source
	TCCR2B &= ~((1 << CS22) | (1 << CS21));		//TC0 overflow precisely once every 1/128 second
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));		//CTC mode: timer increments until overflow
	TCCR2B &= ~(1<<WGM22);
	while (ASSR & ((1 << TCN2UB) | (1 << OCR2BUB) | (1 << TCR2BUB))) {} // Wait until TC0 is updated
	TIMSK2 |= (1 << TOIE2);						//Enable Timer/Counter2 Overflow Interrupts

	//Timer 0 - Used for millis()
	TCCR0B |= (1 << CS02 | 1<< CS00);			//Set up timer0 - prescale of 1024
	TCNT0 = 0;  								//initialize timer0 counter

	sei();										//Set the Global Interrupt Enable Bit
}


//start the software RTC
void startRTC() {

	setTimer();
	rtc.counter = 0;
	rtc.ticks = 0;
	rtc.isRunning = 1;
	TCNT2 = 0;
	TIMSK2 |= (1<<TOIE2);
	SREG |= (1<<SREG_I);
}


void stopRTC() {

	rtc.isRunning = 0;
	TIMSK2 &= ~(1<<TOIE2);
	SREG &= ~(1<<SREG_I);
}


void timeEvent(void)  {
	rtc.counter++;
	rtc.ticks++;							//ticks since the last CPU reset
	rtc.runticks++;						//elapsed tick count
	rtc.onSecInterval = 0;
	rtc.onMinInterval = 0;
	rtc.onHrInterval = 0;
	rtc.onDayInterval = 0;


	if (rtc.counter > 127) { // equal to 1 s ( 127 ticks per second. 32768KHz clock ~ 4 ms per tick 0.0039/s )
		rtc.counter = 0;
		rtc.onSecInterval = true;
		if (++rtc.second == 60) {
			rtc.second = 0;
			rtc.onMinInterval = true;
			if (++rtc.minute == 60) {
				rtc.minute = 0;
				rtc.onHrInterval = true;
				if (++rtc.hour == 24) {
					rtc.hour = 0;
					if (++rtc.day == 32) {
						rtc.month++;
						rtc.day = 1;
					} else if (rtc.day == 31) {
						if ((rtc.month == 4) || (rtc.month == 6) || (rtc.month == 9) || (rtc.month == 11)) {
							rtc.month++;
							rtc.day = 1;
						}
					} else if (rtc.day == 30) {
						if (rtc.month == 2) {
							rtc.month++;
							rtc.day = 1;
						}
					} else if (rtc.day == 29) {
						if ((rtc.month == 2) && not_leap()) {
							rtc.month++;
							rtc.day = 1;
						}
					}
						if (rtc.month == 13) {
						rtc.month = 1;
						rtc.year++;
					}
				}
			}
		}
	}
}

