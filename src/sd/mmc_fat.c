/**********************************************************/
/* mmc_fat.c                                              */
/* Copyright (c) 2010 by thomas seiler                    */ 
/* read a file from a FAT16 formatted MMC card            */
/* Code taken from HolgerBootloader (public domain)       */
/* from mikrokontroller.net and adapted for smaller size  */
/*                                                        */
/* -------------------------------------------------------*/
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/**********************************************************/

#ifdef SDCARD
#include <avr/io.h>
#include <inttypes.h>
// #if !defined(__AVR_ATmega168__) || !defined(__AVR_ATmega328P__)
// #include <avr/eeprom.h>  /* filename from eeprom */
// #endif
/*#include "board.h"*/
#include "mmc_fat.h"
//#include "prog_flash.h"

#define MMC_CMD0_RETRY	(unsigned char)16

static unsigned char cmd[6];

/* ---[ SPI Interface ]---------------------------------------------- */

static void spi_send_byte(unsigned char data)
{
	SPDR=data;
	loop_until_bit_is_set(SPSR, SPIF); // wait for byte transmitted...
}

static unsigned char send_cmd(void)
{
	unsigned char i;
	unsigned char *buf;

	spi_send_byte(0xFF);      //Dummy delay 8 clocks
	MMC_PORT &= ~(1<<MMC_CS); //MMC Chip Select -> Low (activate mmc)

	/* send the 6 cmd bytes */
	i=6;
	buf = cmd;
	while(i) {
		spi_send_byte(*buf++);
		i--;
	}

	unsigned char result;

	/* wait for response */
	for(i=0; i<255; i++) {

 		spi_send_byte(0xFF);
		result = SPDR;

		if ((result & 0x80) == 0)
			break;
	}

	return(result); // TimeOut !?
}

/* ---[ MMC Interface ]---------------------------------------------- */

//all MMC Commandos needed for reading a file from the card
#define MMC_GO_IDLE_STATE 0
#define MMC_SEND_OP_COND 1
#define MMC_READ_SINGLE_BLOCK 17

/* the sector buffer */
uint8_t buff[512];


/*
*		Call mmc_init one time after a card has been connected to the ?C's SPI bus!
*
*		return values:
*			MMC_OK:				MMC initialized successfully
*			MMC_INIT:			Error while trying to reset MMC
*			MMC_TIMEOUT:	Error/Timeout while trying to initialize MMC
*/
uint8_t mmc_init(void)
{
	// the default after reset is already input
	//SPI_DDR &= ~(1<<SPI_MISO);	//SPI Data Out -> Input
	SPI_PORT |= 1<<SPI_SS;   //PB2 output: High (deselect other SPI chips)

	SPI_DDR  |= 1<<SPI_CLK | 1<<SPI_MOSI | 1<<SPI_SS; // SPI Data -> Output
	MMC_DDR |= 1<<MMC_CS; 	//MMC Chip Select -> Output


	SPCR = 1<<SPE | 1<<MSTR | SPI_INIT_CLOCK; //SPI Enable, SPI Master Mode

	unsigned char i;

	i=10;
	while(i) { //Pulse 80+ clocks to reset MMC
		spi_send_byte(0xFF);
 		i--;
	}

	unsigned char res;

	cmd[0] = 0x40 + MMC_GO_IDLE_STATE;
	cmd[1] = 0x00; cmd[2] = 0x00; cmd[3] = 0x00; cmd[4] = 0x00;	cmd[5] = 0x95;

	for (i=0; i<MMC_CMD0_RETRY; i++)
	{
		res=send_cmd(); //store result of reset command, should be 0x01

		MMC_PORT |= 1<<MMC_CS; //MMC Chip Select -> High (deactivate mmc);
      	spi_send_byte(0xFF);
		if (res == 0x01)
			break;
	}

	if(i==MMC_CMD0_RETRY) return(MMC_TIMEOUT);

	if (res != 0x01) //Response R1 from MMC (0x01: IDLE, The card is in idle state and running the initializing process.)
		return(MMC_INIT);

	cmd[0]=0x40 + MMC_SEND_OP_COND;

//May be this becomes an endless loop ?
//Counting i from 0 to 255 and then timeout
//was too SHORT for some of my cards !
	while(send_cmd() != 0) {
		MMC_PORT |= 1<<MMC_CS; //MMC Chip Select -> High (deactivate mmc);
		spi_send_byte(0xFF);
	}

	return(MMC_OK);
}

static inline unsigned char wait_start_byte(void)
{
	unsigned char i;

	i=255;
	do {
		spi_send_byte(0xFF);
		if(SPDR == 0xFE) return MMC_OK;
	} while(--i);

	return MMC_NOSTARTBYTE;
}

/*
 *		mmc_start_read_sector initializes the reading of a sector
 *
 *		Parameters:
 *			adr: specifies address to be read from
 *
 *		Return values:
 *			MMC_OK:						Command successful
 *			MMC_CMDERROR:			Error while sending read command to mmc
 *			MMC_NOSTARTBYTE:	No start byte received
 */
static unsigned char mmc_start_read_block(unsigned long adr)
{
	adr <<= 1;

	cmd[0] = 0x40 + MMC_READ_SINGLE_BLOCK;
	cmd[1] = (adr & 0x00FF0000) >> 0x10;
	cmd[2] = (adr & 0x0000FF00) >> 0x08;
	cmd[3] = (adr & 0x000000FF);
	cmd[4] = 0;

	SPCR = 1<<SPE | 1<<MSTR | SPI_READ_CLOCK; //SPI Enable, SPI Master Mode

	if (send_cmd() != 0x00 || wait_start_byte()) {
		MMC_PORT |= 1<<MMC_CS; //MMC Chip Select -> High (deactivate mmc);
		return(MMC_CMDERROR); //wrong response!
	}

	//mmc_read_buffer
	unsigned char *buf;
	unsigned short len;

	buf = buff;
	len= 512;

	while (len) {
		spi_send_byte(0xFF);
		*buf++ = SPDR;
		len--;
	}

	//mmc_stop_read_block
	//read 2 bytes CRC (not used);
	spi_send_byte(0xFF);
	spi_send_byte(0xFF);
	MMC_PORT |= 1<<MMC_CS; //MMC Chip Select -> High (deactivate mmc);

	return(MMC_OK);
}

/* ---[ FAT16 ]------------------------------------------------------ */

static uint16_t  RootDirRegionStartSec;
static uint32_t  DataRegionStartSec;
static uint16_t  RootDirRegionSize;
static uint8_t   SectorsPerCluster;
static uint16_t  FATRegionStartSec;

static inline unsigned char fat16_init(void)
{
	mbr_t *mbr = (mbr_t*) buff;
	vbr_t *vbr = (vbr_t*) buff;

	if (mmc_init() != MMC_OK) return 1;

    mmc_start_read_block(0);

    // Try sector 0 as a bootsector
	if ((vbr->bsFileSysType[0] == 'F') && (vbr->bsFileSysType[4] == '6'))
	{
		FATRegionStartSec = 0;
	}
	else // Try sector 0 as a MBR
	{
		FATRegionStartSec = mbr->sector.partition[0].sectorOffset;

		mmc_start_read_block(mbr->sector.partition[0].sectorOffset);

        if ((vbr->bsFileSysType[0] != 'F') || (vbr->bsFileSysType[4] != '6'))
		   return 2; // No FAT16 found
     }

	SectorsPerCluster  			= vbr->bsSecPerClus; // 4

	// Calculation Algorithms
	FATRegionStartSec			+= vbr->bsRsvdSecCnt;						// 6
	RootDirRegionStartSec	 	= FATRegionStartSec + (vbr->bsNumFATs * vbr->bsNrSeProFAT16);		// 496	
	RootDirRegionSize		 	= (vbr->bsRootEntCnt / 16); 						// 32
	DataRegionStartSec 			= RootDirRegionStartSec + RootDirRegionSize;	// 528

	return 0;
}

static struct _file_s {
	uint16_t startcluster;
 	uint16_t sector_counter;
 	uint32_t size;
 	uint8_t* next;
} file;

#endif
