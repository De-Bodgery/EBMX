#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "mcpwm_foc.h"
#include "stdio.h"
#include "stdlib.h"
#include "utils.h"
#include "commands.h"
#include "encoder.h"
#include "terminal.h"

#include "app.h"
extern volatile int inside0x53, crc_set5, crc_set6, crc_ten_53;

static THD_FUNCTION(display_thread, arg);
static THD_WORKING_AREA(display_thread_wa, 2048); // 2kb stack for this thread

void app_display_setting_init(void) {

// Start the example thread
	chThdCreateStatic(display_thread_wa, sizeof(display_thread_wa),
	NORMALPRIO, display_thread, NULL);

}

static THD_FUNCTION(display_thread, arg) {
	(void) arg;
	chRegSetThreadName("Display Setting");

	for (;;) {
		while (inside0x53) {
			set_display_config(crc_set5, crc_set6, crc_ten_53);
		//	chThdSleepMilliseconds(10);

		}
		//	commands_printf("display alive");
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / 60;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}

		chThdSleep(sleep_time);
		timeout_reset();
	}
}

