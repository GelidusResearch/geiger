/*
  uart0.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include <avr/interrupt.h>
#include "uart.h"
#include "uart_private.h"

ISR(USART_RX_vect)
 {
   Serial._rx_complete_irq();
 }

ISR(USART_UDRE_vect)
{
  Serial._tx_udr_empty_irq();
}

uart Serial(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UCSR0C, &UDR0);

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.
bool Serial0_available() {
  return Serial.available();
}
