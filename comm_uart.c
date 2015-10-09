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
 * comm_uart.c
 *
 *  Created on: 17 aug 2015
 *      Author: benjamin
 */

#include "comm_uart.h"
#include "ch.h"
#include "hal.h"
#include "bldc_interface_uart.h"

#include <string.h>

// Settings
#define UART_BAUDRATE			115200
#define UART_DEV				UARTD3
#define UART_GPIO_AF			7
#define UART_TX_PORT			GPIOB
#define UART_TX_PIN				10
#define UART_RX_PORT			GPIOB
#define UART_RX_PIN				11
#define SERIAL_RX_BUFFER_SIZE	1024

// Private functions
static void send_packet(unsigned char *data, unsigned int len);

// Threads
static THD_FUNCTION(timer_thread, arg);
static THD_WORKING_AREA(timer_thread_wa, 512);
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;

	/*
	 * Put the character in a buffer and notify a thread that there is data
	 * available. An alternative way is to use
	 *
	 * packet_process_byte(c);
	 *
	 * here directly and skip the thread. However, this could drop bytes if
	 * processing packets takes a long time.
	 */

	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chEvtSignalI(process_tp, (eventmask_t) 1);
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		UART_BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("comm_uart");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		/*
		 * Wait for data to become available and process it as long as there is data.
		 */

		while (serial_rx_read_pos != serial_rx_write_pos) {
			bldc_interface_uart_process_byte(serial_rx_buffer[serial_rx_read_pos++]);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Wait for the previous transmission to finish.
	while (UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	// Send the data over UART
	uartStartSend(&UART_DEV, len, buffer);
}

/**
 * This thread is only for calling the timer function once
 * per millisecond. Can also be implementer using interrupts
 * if no RTOS is available.
 */
static THD_FUNCTION(timer_thread, arg) {
	(void)arg;
	chRegSetThreadName("packet timer");

	for(;;) {
		bldc_interface_uart_run_timer();
		chThdSleepMilliseconds(1);
	}
}

void comm_uart_init(void) {
	// Initialize UART
	uartStart(&UART_DEV, &uart_cfg);
	palSetPadMode(UART_TX_PORT, UART_TX_PIN, PAL_MODE_ALTERNATE(UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(UART_RX_PORT, UART_RX_PIN, PAL_MODE_ALTERNATE(UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	// Initialize the bldc interface and provide a send function
	bldc_interface_uart_init(send_packet);

	// Start processing thread
	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
			NORMALPRIO, packet_process_thread, NULL);

	// Start timer thread
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa),
			NORMALPRIO, timer_thread, NULL);
}
