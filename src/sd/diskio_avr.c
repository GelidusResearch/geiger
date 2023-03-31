/*-------------------------------------------------------------------------*/
/* PFF - Low level disk control module for AVR							   */
/* Modified from (C)ChaN, 2014											   */
/*-------------------------------------------------------------------------*/

#include "pff.h"
#include "diskio.h"

/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions MODIFY FOR YOUR DEVICE           */
/*-------------------------------------------------------------------------*/

#include <avr/io.h> /* Device specific include files */
#include <util/delay.h>

//Port & Pin definitions.
#define SPI_PORT	PORTB
#define SPI_DDR		DDRB
#define SPI_MISO	PB4		//DataOut of Master in = uC Slave out = SD
#define SPI_MOSI	PB3		//DataIn of  Master out = uC Slave in = SD
#define SPI_CLK  	PB5		//Clock of Master
#define SPI_SS      PB2     //SS pin of SPI interface to CS of Slave

// These define the Pin, Port and DDR of the Chip Select to the MMC...
#define MMC_CS			SPI_SS
#define MMC_PORT        SPI_PORT
#define MMC_DDR         SPI_DDR

//Clock rate while initialization / reading / writing
#define SPI_INIT_CLOCK 1<<SPR1 | 1<<SPR0
#define SPI_READ_CLOCK 0<<SPR1 | 0<<SPR0
#define SPI_WRITE_CLOCK 1<<SPR1 | 0<<SPR0
#define SPI_DOUBLE_SPEED 0 //0: normal speed, 1: double speed

/* Port controls  (Platform dependent) */
#define SELECT() (MMC_PORT &= ~(1<<MMC_CS)) /* CS = L */
#define DESELECT() (MMC_PORT |= (1<<MMC_CS)) /* CS = H */
#define SELECTING (MMC_DDR & MMC_CS) && !(SPI_MOSI & MMC_CS)

static void init_spi(void)
{
		// the default after reset is already input
		//SPI_DDR &= ~(1<<SPI_MISO);	//SPI Data Out -> Input
		SPI_PORT |= 1<<SPI_SS;   //PB2 output: High (deselect other SPI chips)
		SPI_DDR  |= 1<<SPI_CLK | 1<<SPI_MOSI | 1<<SPI_SS; // SPI Data -> Output
		MMC_DDR |= 1<<MMC_CS; 	//MMC Chip Select -> Output
		SPCR = 1<<SPE | 1<<MSTR | SPI_INIT_CLOCK; //SPI Enable, SPI Master Mode
}

static uint8_t spi(uint8_t d)
{
	SPDR = d;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

static void xmit_spi(uint8_t d)
{
	spi(d);
}

static uint8_t rcv_spi(void)
{
	return spi(0xFF);
}

//--------------------------------------------------------------------------
//   Module Private Functions
//---------------------------------------------------------------------------



uint8_t CardType = 0;


/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/
/* 1st byte (Start + Index) */
/* Argument (32 bits) */
static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {

	uint8_t n, res;
	/* ACMD<n> is the command sequence of CMD55-CMD<n> */
	if (cmd & 0x80) {
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1)
			return res;
	}

	/* Select the card */
	DESELECT();
 	rcv_spi();
	SELECT();
 	rcv_spi();

	/* Send a command packet */
	xmit_spi(cmd);               /* Start + Command index */
	xmit_spi((uint8_t)(arg >> 24)); /* Argument[31..24] */
	xmit_spi((uint8_t)(arg >> 16)); /* Argument[23..16] */
	xmit_spi((uint8_t)(arg >> 8));  /* Argument[15..8] */
	xmit_spi((uint8_t)arg);         /* Argument[7..0] */
	n = 0x01;                    /* Dummy CRC + Stop */
	if (cmd == CMD0)
		n = 0x95; /* Valid CRC for CMD0(0) */
	if (cmd == CMD8)
		n = 0x87; /* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive a command response */
	n = 10; /* Wait for a valid response in timeout of 10 attempts */
	do {
		res = rcv_spi();
	} while ((res & 0x80) && --n);
	/* Return with the response value */
	return res;
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/** read CID or CSR register */
uint32_t readRegister(void) {
	uint32_t capacity = 0;
	uint8_t res = 0;
	res = send_cmd(CMD9, 0);
	if (res == RES_OK) {
		// transfer data
		for (uint8_t i = 0; i < 5; i++) rcv_spi(); //dump the first 5 bytes
		capacity = ((uint32_t)(rcv_spi() << 16)); //Need 6 LSB of this byte
		capacity &= 0x002FFFFF;
		capacity |= ((uint32_t)(rcv_spi() << 8));
		capacity |= ((uint32_t)(rcv_spi()));

		for (uint8_t i = 0; i < 8; i++) rcv_spi(); //dump the last 8 bytes
		//for (uint16_t i = 0; i < 16; i++) dst[i] = rcv_spi();
		rcv_spi();  // get first crc byte
		rcv_spi();  // get second crc byte
	}
	return capacity;
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(void)
{
	uint8_t n, cmd, ty, ocr[4];
	uint16_t tmr;

#if _USE_WRITE
	if (CardType && SELECTING)
		disk_writep(0, 0); /* Finalize write process if it is in progress */
#endif

	init_spi(); /* Initialize ports to control MMC */
	SELECT();
	DESELECT();
	for (n = 10; n; n--)
		rcv_spi(); /* 80 dummy clocks with CS=H */

	ty = 0;
	/* GO_IDLE_STATE */
	if (send_cmd(CMD0, 0) == 1) {
		/* SDv2 */
		if (send_cmd(CMD8, 0x1AA) == 1) { 
			for (n = 0; n < 4; n++)

			/* Get trailing return value of R7 resp */
			ocr[n] = rcv_spi();

			/* The card can work at vdd range of 2.7-3.6V */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {

				/* Wait for leaving idle state (ACMD41 with HCS bit) */
				for (tmr = 10000; tmr && send_cmd(ACMD41, 1UL << 30); tmr--) _delay_us(100);

				 /* Check CCS bit in the OCR */
				if (tmr && send_cmd(CMD58, 0) == 0) {
					for (n = 0; n < 4; n++)
						ocr[n] = rcv_spi();
					 /* SDv2 (HC or SC) */
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		} else { /* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) {
				ty  = CT_SD1;
				cmd = ACMD41; /* SDv1 */
			} else {
				ty  = CT_MMC;
				cmd = CMD1; /* MMCv3 */
			}
			for (tmr = 10000; tmr && send_cmd(cmd, 0); tmr--)
				_delay_us(100);                    /* Wait for leaving idle state */
			if (!tmr || send_cmd(CMD16, 512) != 0) /* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;
	DESELECT();
	rcv_spi();

	return ty ? 0 : STA_NOINIT;
}

//-----------------------------------------------------------------------
// Read partial sector
//-----------------------------------------------------------------------

// *buff  Pointer to the read buffer (NULL:Forward to the stream)
// sector Sector number (LBA)
// offset Byte offset to read from (0..511)
// count Number of bytes to read (ofs + cnt mus be <= 512)

DRESULT disk_readp(uint8_t *buff, uint32_t sector, uint16_t offset ,uint16_t count)
{
	DRESULT res;
	uint8_t    rc;
	uint16_t    bc;

	if (!(CardType & CT_BLOCK))
	sector *= 512; /* Convert to byte address if needed */

	res = RES_ERROR;
	/* READ_SINGLE_BLOCK */
	if (send_cmd(CMD17, sector) == 0) {

		do {
			rc = rcv_spi();
			} while (rc == 0xFF);

		/* A data packet arrived */
		if (rc == 0xFE) {
			bc = 512 + 2 - offset - count; /* Number of trailing bytes to skip */
			/* Skip leading bytes */
			while (offset--)
				rcv_spi();

			/* Receive a part of the sector */
			/* Store data to the memory */
			if (buff) {
				do {
					*buff++ = rcv_spi();
				} while (--count);
			/* Forward data to the outgoing stream */
			} else {
				// FORWARD(rcv_spi());
				do {
				} while (--count);
			}

			/* Skip trailing bytes and CRC */
			do
				rcv_spi();
				while (--bc);
				res = RES_OK;
		}
	}

	DESELECT();
	rcv_spi();

	return res;
}

//-----------------------------------------------------------------------
// Write partial sector
//-----------------------------------------------------------------------
// *buff Pointer to the bytes to be written (NULL:Initiate/Finalize sector write)
// sc Number of bytes to send, Sector number (LBA) or zero

#if _USE_WRITE
DRESULT disk_writep(const uint8_t *buff, uint32_t sc )
{
	DRESULT		res;
	uint16_t	bc;
	static uint16_t wc;	/* Sector write counter */

	res = RES_ERROR;

	if (buff) { /* Send data bytes */
		bc = sc;
		while (bc && wc) { /* Send data bytes to the card */
			xmit_spi(*buff++);
			wc--;
			bc--;
		}
		res = RES_OK;
	} else {
		if (sc) { /* Initiate sector write process */
			if (!(CardType & CT_BLOCK))
				sc *= 512;                  /* Convert to byte address if needed */
			if (send_cmd(CMD24, sc) == 0) { /* WRITE_SINGLE_BLOCK */
				xmit_spi(0xFF);
				xmit_spi(0xFE); /* Data block header */
				wc  = 512;      /* Set byte counter */
				res = RES_OK;
			}
		} else { /* Finalize sector write process */
			bc = wc + 2;
			while (bc--) {
				xmit_spi(0); /* Fill left bytes and CRC with zeros */
			}
			do {
				res = rcv_spi();
			} while (res == 0xFF);
			if ((res & 0x1F) == 0x05) { /* Receive data resp and wait for end of write process in timeout of 500ms */
				for (bc = 5000; rcv_spi() != 0xFF && bc; bc--) /* Wait for ready */
					_delay_us(100);
				if (bc)
					res = RES_OK;
			}
			DESELECT();
			rcv_spi();
		}
	}

	return res;
}
#endif
