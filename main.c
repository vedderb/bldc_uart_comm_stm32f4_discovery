#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

#include "comm_usb_serial.h"
#include "comm_uart.h"
#include "bldc_interface.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(4096)
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(512)

void main_printf(const char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	chvprintf((BaseSequentialStream*)&SDU1, fmt, ap);
	va_end(ap);
}

static void cmd_val(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: val\r\n");
		return;
	}

	bldc_interface_get_values();
}

static const ShellCommand commands[] = {
		{"val", cmd_val},
		{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream *)&SDU1,
		commands
};

void bldc_val_received(mc_values *val) {
	main_printf("\r\n");
	main_printf("Input voltage: %.2f V\r\n", val->v_in);
	main_printf("Temp:          %.2f degC\r\n", val->temp_mos);
	main_printf("Current motor: %.2f A\r\n", val->current_motor);
	main_printf("Current in:    %.2f A\r\n", val->current_in);
	main_printf("RPM:           %.1f RPM\r\n", val->rpm);
	main_printf("Duty cycle:    %.1f %%\r\n", val->duty_now * 100.0);
	main_printf("Ah Drawn:      %.4f Ah\r\n", val->amp_hours);
	main_printf("Ah Regen:      %.4f Ah\r\n", val->amp_hours_charged);
	main_printf("Wh Drawn:      %.4f Wh\r\n", val->watt_hours);
	main_printf("Wh Regen:      %.4f Wh\r\n", val->watt_hours_charged);
	main_printf("Tacho:         %i counts\r\n", val->tachometer);
	main_printf("Tacho ABS:     %i counts\r\n", val->tachometer_abs);
	main_printf("Fault Code:    %s\r\n", bldc_interface_fault_to_string(val->fault_code));
}

int main(void) {
	thread_t *shelltp = NULL;

	// For ChibiOS
	halInit();
	chSysInit();

	// For the usb-serial shell
	comm_usb_serial_init();
	shellInit();

	// For the UART interface
	comm_uart_init();

	// Give bldc_interface a function to call when valus are received.
	bldc_interface_set_rx_value_func(bldc_val_received);

	// Main loop where only the shell is handled
	for(;;) {
		if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminatedX(shelltp)) {
			chThdRelease(shelltp);    // Recovers memory of the previous shell.
			shelltp = NULL;           // Triggers spawning of a new shell.
		}
		chThdSleepMilliseconds(1000);
	}
}
