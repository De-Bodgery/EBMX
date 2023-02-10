/*
 Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

 This file is part of the VESC firmware.

 The VESC firmware is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 The VESC firmware is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"
#include "jm9.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.c"

#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf

extern volatile int zeropoint;
extern volatile float walk_current;
// Threads
static THD_FUNCTION(adc_thread, arg);
static THD_WORKING_AREA(adc_thread_wa, 1024);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;

//local
volatile int button_state, last_button_state;
volatile float flag = 0xf;
volatile float reversed, pressed = 1;

//extern later
extern volatile float throttle_torque_limit, throttle_watt_limit, speed_limit,
		torque_output, reverse_strength;

extern volatile int brake_apply;

//end of extern

void app_adc_configure(adc_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
	//use_rx_tx_as_buttons = use_rx_tx;
	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO,
			adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}
int get_battery_model(void) {
	return (int) config.rev_button_inverted;
}

/*
 void pin_state_polling(void) {
 button_state = palReadPad(STAND_IO, STAND_PIN);
 if (button_state != last_button_state) { //pin changed

 if (button_state == 1) {
 zeropoint = 2;
 //toggle
 reversed = reversed * (float) -1;
 if (reversed < 0)
 reversed = -0.2;
 else
 reversed = 1;
 }
 }
 last_button_state = button_state;
 //	commands_printf("reversed %f.2", reversed);
 //chThdSleepMilliseconds(50);
 }
 */
void suron_talaria_switch(void) {
	if (stock_switch() == 1 && flag != -1) { //stock switch on and not in reverse gear
		if (bike_frame() == CAN_SURRON) { //surron
			if (palReadPad(BUTTON_SWITCH_IO, BUTTON_SWITCH_PIN))
				reversed = reverse_strength;
			else
				reversed = 1;

		} else if (bike_frame() == CAN_TALARIA) {
			if (palReadPad(BUTTON_SWITCH_IO, BUTTON_SWITCH_PIN)) {
				reversed = 1;
			} else
				reversed = reverse_strength;
		}
	} else if (stock_switch() == 1 && flag == -1) { //stock switch on and in reverse gear
		reversed = reverse_strength;
	}

	if (stock_switch() == 0 && flag == -1)
		reversed = reverse_strength;
	else if (stock_switch() == 0 && flag != -1)
		reversed = 1;

}
void Configure_PC13(void) {
	/* Set variables used */
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOB */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Tell system that you will use PC13 for EXTI_Line13 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	/* PB5 is connected to EXTI_Line13 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PC13 is connected to EXTI_Line13, which has EXTI15_10_IRQn vector */

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
	nvicEnableVector(EXTI15_10_IRQn, 0);

}

static THD_FUNCTION(adc_thread, arg) {
	(void) arg;

	chRegSetThreadName("APP_ADC");

	is_running = true;
	char buffer[20];
	int brake_pressed = 0;
	float pwr = 0;
	float brake = 0;



	for (;;) {

		suron_talaria_switch();
		pwr = (float) ADC_Value[ADC_IND_EXT];
		pwr /= 4095;
		pwr *= 5.55;

		read_voltage = pwr;

		brake = (float) ADC_Value[ADC_IND_EXT2];
		brake /= 4095;
		brake *= 5.55;

		read_voltage2 = brake;

// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / 300;	//300
// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

// For safe start when fault codes occur
		/*	if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		 ms_without_power = 0;
		 }*/

		if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_THROTTLE) {
			if (read_voltage > config.voltage_end + 0.5
					|| read_voltage <= (config.voltage_start - 0.5)) {
				mc_interface_fault_stop(FAULT_CODE_THROTTLE_VOLTAGE, false,
						false);
				pwr = 0;
			}

			if (config.voltage_end <= 2.0)	//avoid auto calibration set at 0.9
				mc_interface_fault_stop(FAULT_CODE_THROTTLE_VOLTAGE, false,
						false);

		}

		if (thumb_throttle_type() == REGEN_CTRL_TYPE_REGEN
				|| thumb_throttle_type() == REGEN_CTRL_TYPE_REGEN_AS_THROTTLE) {
			if (read_voltage2 > config.voltage2_end + 0.5
					|| read_voltage2 <= (config.voltage2_start - 0.5)) {
				mc_interface_fault_stop(FAULT_CODE_REGEN_VOLTAGE, false, false);
				brake = 0;
			}
		}

		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0,
				1.0);
// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		decoded_level = pwr;

		brake = utils_map(brake, config.voltage2_start, config.voltage2_end,
				0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);
		decoded_level2 = brake;

// Apply deadband
		utils_deadband(&brake, config.hyst + 0.05, 1.0);

		utils_deadband(&pwr, config.hyst, 1.0);
		//	commands_printf("power: %.2f, deadband: %.2f", pwr, config.hyst);
//	utils_deadband(&brake, config.hyst, 1.0);
// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp,
				config.throttle_exp_brake, config.throttle_exp_mode);

		brake = utils_throttle_curve(brake, config.throttle_exp,
				config.throttle_exp_brake, config.throttle_exp_mode);

// Apply ramping
		static systime_t last_time = 0;
		static float pwr_ramp = 0.0;
		float ramp_time =
				fabsf(pwr) > fabsf(pwr_ramp) ?
						config.ramp_time_pos : config.ramp_time_neg;
		//if (ramp_time > 0.01) {
		const float ramp_step = (float) ST2MS(chVTTimeElapsedSinceX(last_time))
				/ (ramp_time * 1000.0);
		utils_step_towards(&pwr_ramp, pwr, ramp_step);
		last_time = chVTGetSystemTimeX();
		pwr = pwr_ramp;
		//}
		//	commands_printf("ramp power: %.2f ", pwr);
		bool current_mode_brake = false;
//		bool current_mode_reverse = false;

		float current_rel = 0.0;
		if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_THROTTLE) {

			if (config.safe_start == false) {
				current_mode_brake = false;
				current_rel = pwr;
			} else		// off throttle regen
			{
				if (pwr <= 0) {
					current_mode_brake = true;
					current_rel = config.voltage_center * 0.005;	//50% max
					if (current_rel <= 0)
						current_mode_brake = false;
				} else {
					current_mode_brake = false;
					current_rel = pwr;
				}
			}

		}
		if (thumb_throttle_type() == REGEN_CTRL_TYPE_REGEN_AS_THROTTLE) {
			if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_THROTTLE) {//ignore off throttle regen
				current_rel = pwr + brake;
				current_mode_brake = false;

			} else if (config.ctrl_type == ADC_CTRL_TYPE_NONE) {
				current_rel = brake;
				current_mode_brake = false;
			}

		}
		if (thumb_throttle_type() == REGEN_CTRL_TYPE_REGEN) {

			if (brake > 0) {
				if (config.safe_start == false) {	//no off throttle regen
					current_mode_brake = true;
					current_rel = brake;
				} else // off throttle regen
				{
					current_mode_brake = true;
					current_rel = config.voltage_center * 0.005 + brake; //off throttle regen plus regen 50% max
				}

			} else {		//brake==0 or<0

				if (config.safe_start == false) {	//no off throttle regen
					current_mode_brake = false;
					current_rel = pwr;		//release for throttle
				} else {
					if (pwr <= 0) {		//twist throttle idle
						current_mode_brake = true;
					}
				}
			}

		}

		timeout_reset();
		float current_out = current_rel;

		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_NONE: 	//vesc tool control
			break;
		case ADC_CTRL_TYPE_CURRENT_THROTTLE:

			if (current_mode_brake) {

				if (flag != 0xf) { 	//stop the bike from braking when gear 0

					if (get_battery_series() <= 24
							&& ((GET_INPUT_VOLTAGE()
									/ (float) get_battery_series()) <= 4.1)) //regen protection
						mc_interface_set_brake_current_rel(current_out);

				} else
					mc_interface_set_current_rel(0);
			} else {
				if (flag != 0xf) {
					mc_interface_set_current_rel(
							current_out * brake_apply * reversed);
				} else
					mc_interface_set_current_rel(0);
			}
			break;

		}
		switch (thumb_throttle_type()) {

		case REGEN_CTRL_TYPE_REGEN_AS_THROTTLE: // will not have off throttle regen
			if (flag != 0xf) {
				mc_interface_set_current_rel(
						current_out * brake_apply * reversed);
			} else
				mc_interface_set_current_rel(0);
			break;

		}

	}
}
