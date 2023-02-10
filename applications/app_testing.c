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

static THD_FUNCTION(testing_thread, arg);
static THD_WORKING_AREA(testing_thread_wa, 2048); // 2kb stack for this thread

static int terminal_state = 0;
static void life_test(int argc, const char **argv);
static void io_test(int argc, const char **argv);
static void stop(int argc, const char **argv);
static void encoder(int argc, const char **argv);

void app_testing_init(void) {

// Start the example thread
	chThdCreateStatic(testing_thread_wa, sizeof(testing_thread_wa),
	NORMALPRIO, testing_thread, NULL);
	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback("life_test", "Print the number d", "[d]",
			life_test);

	terminal_register_command_callback("io_test", "Print the number d", "[d]",
			io_test);

	terminal_register_command_callback("stop", "Print the number d", "[d]",
			stop);

	terminal_register_command_callback("encoder-test", "Print the number d",
			"[d]", encoder);

}

static void life_test(int argc, const char **argv) {
	if (argc == 1) {

		terminal_state = 1;
	}
}

static void io_test(int argc, const char **argv) {
	if (argc == 1) {
		terminal_state = 2;
	}
}
static void encoder(int argc, const char **argv) {
	if (argc == 1) {

		terminal_state = 3;
	}
}

static void stop(int argc, const char **argv) {
	if (argc == 1) {

		terminal_state = -1;
	}
}

static THD_FUNCTION(testing_thread, arg) {
	(void) arg;
	chRegSetThreadName("Testing");

	int overheat = 0;
	int ramping = 0;
	for (;;) {
		if (terminal_state == -1)
			mc_interface_release_motor();

		while (terminal_state == 1) //life test
		{

			while ((mc_interface_temp_fet_filtered() < 70
					&& mc_interface_temp_motor_filtered() < 80)
					&& (terminal_state == 1)) {
				if (ramping == 0) {
					for (int i = 0; i < 750; i++) {
						mc_interface_set_current(i);
						chThdSleepMilliseconds(7);
						ramping = 1;
					}
				}
				chThdSleepMilliseconds(100);

			}

			if (mc_interface_temp_fet_filtered() >= 70
					|| mc_interface_temp_motor_filtered() >= 80) { //one of them overheat
				while (mc_interface_temp_motor_filtered() >= 40) {
					mc_interface_release_motor();
					chThdSleepMilliseconds(100);
					ramping = 0;
				}
			}
			chThdSleepMilliseconds(100);
		}

		while (terminal_state == 2) //io test
		{
			commands_printf(
					"brake: %i stand: %i  tilt: %i  button: %i encoder z: %i hallA: %i hallB: %i hallC: %i encoder sin: %.2f cos: %.2f",
					palReadPad(BRAKE_IO, BRAKE_PIN),
					palReadPad(STAND_IO, STAND_PIN),
					palReadPad(TILT_IO, TILT_PIN),
					palReadPad(BUTTON_SWITCH_IO, BUTTON_SWITCH_PIN),
					palReadPad(ENCODER_ZERO_IO, ENCODER_ZERO_PIN),READ_HALL1(),READ_HALL2(),READ_HALL3(),ENCODER_SIN_VOLTS,ENCODER_COS_VOLTS);

			chThdSleepMilliseconds(100);
		}

		while (terminal_state == 3) //encoder
		{

			get_sincos();
			chThdSleepMilliseconds(10);
		}

		systime_t sleep_time = CH_CFG_ST_FREQUENCY / 100;
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}

		chThdSleep(sleep_time);
		timeout_reset();
	}
}

