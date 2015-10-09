/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * bldc_interface_uart.h
 *
 *  Created on: 9 okt 2015
 *      Author: benjamin
 */

#ifndef BLDC_INTERFACE_UART_H_
#define BLDC_INTERFACE_UART_H_

// Includes
#include "packet.h" // For the MAX_PACKET_LEN define

// Functions
void bldc_interface_uart_init(void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_uart_process_byte(unsigned char b);
void bldc_interface_uart_run_timer(void);

#endif /* BLDC_INTERFACE_UART_H_ */
