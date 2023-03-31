/**
 *	File:       	logger.cpp
 *	Version:  		1.0
 *	Date:       	2017
 *	License:		GPL v3
 *	Description:	general purpose code for AVR micro controllers
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
#include "../logger/logger.h"
#include <avr/sfr_defs.h>
#include <avr/io.h>

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message.


void sd_port_init(void)
{
	CS_DDR |= CS; // SD card circuit select as output
	DDRB |= MOSI + SCK; // MOSI and SCK as outputs
	PORTB |= MISO; // pullup in MISO, might not be needed

}

void spi_init_log(void)
{
	SPCR = _BV(SPE)|_BV(MSTR)|_BV(SPR1)|_BV(SPR0);
	SPSR &= ~_BV(SPI2X);
}

void SPI_init() {
	
	// Enable SPI, master, set clock rate fck/128
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPR1);
}

unsigned char SPI_write(unsigned char ch) {
	SPDR = ch;
	while(!(SPSR & (1<<SPIF))) {}
	return SPDR;
}

unsigned char SPI_transmit(unsigned char data)
{
	// Start transmission
	SPDR = data;

	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));

	return SPDR;
}


// void SD_command(unsigned char cmd, unsigned long arg,
// unsigned char crc, unsigned char read) {
// 	unsigned char i, buffer[8];
// 
// 	usart_pstr("SD_command");
// 	SD_CS_ASSERT;
// 
// 	SPI_transmit(cmd);
// 	SPI_transmit(arg>>24);
// 	SPI_transmit(arg>>16);
// 	SPI_transmit(arg>>8);
// 	SPI_transmit(arg);
// 	SPI_transmit(crc);
// 
// 	for(i=0; i<read; i++)
// 	buffer[i] = SPI_transmit(0xFF);
// 
// 	SD_CS_DEASSERT;
// 
// 	for(i=0; i<read; i++) {
// 		usart_transmit(' ');
// 		usart_write_hex(buffer[i]);
// 	}
// 	usart_pstr("\r \n");
// 
// 	// print out read bytes
// }


// int sd (void)
// {
// 	unsigned char option;
// 	port_init();
// 	spi_init();
// 	usart_init(BAUD_PRESCALE);
// 
// 	usart_pstr("USART READY!");
// 	while(1)
// 	{
// 		option = usart_receive();
// 		switch(option)
// 		{
// 			case '1':
// 			usart_pstr("1 command");
// 			SD_command(0x40, 0x00000000, 0x95, 8);
// 			break;
// 			case '2':
// 			SD_command(0x41, 0x00000000, 0xFF, 8);
// 			usart_pstr("2 command");
// 			break;
// 			case '3':
// 			SD_command(0x50, 0x00000200, 0xFF, 8);
// 			usart_pstr("3 command");
// 			break;
// 		}
// 
// 	}
// }