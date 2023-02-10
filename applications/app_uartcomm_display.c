/*
 Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h" // Motor control functions
#include "eeprom.h"
#include "mcpwm_foc.h"
#include "flash_helper.h"
#include "stm32f4xx_flash.h"
#include <string.h>
#include "utils.h"
#include "timeout.h"
static volatile systime_t start_time;

const uint32_t set_address = 0x08060000;
const uint32_t phase_ahf = 0x08060010;
const uint32_t phase_alf = 0x08060020;
const uint32_t phase_bhf = 0x08060030;
const uint32_t phase_blf = 0x08060040;
const uint32_t phase_chf = 0x08060050;
const uint32_t phase_clf = 0x08060060;
const uint32_t serial_p = 0x08060070;

//extern imu_config imu_conf;
//extern nrf_config app_nrf_conf;
//extern volatile float nm;

//extern volatile float tick;
//extern volatile int get_motor_temp;
//extern volatile int get_fet_temp;
extern volatile int get_bbrs;
//extern volatile float get_rpm;
//extern volatile int brake_apply;
extern volatile float flag; //from adc thread
volatile int display_watt; //from app_control since bug in uart thread
//extern volatile int speed_time_interval_ms; //from speed sensor thread /irq
extern volatile int get_unit_battery;
//extern volatile int display_model;
//static char buffer[100];
volatile int crc_set5, crc_set6, crc_ten_53;
volatile float walk_current = 0;
volatile int done_52 = 0;
volatile int inside0x53 = 0;

volatile int oqc = 0;
volatile int mode_display = 0;
volatile int set5 = 0;
volatile int set6 = 0;
volatile int set11 = 0;
volatile int start = 0; //0x3A
volatile int two = 0;   //0X1A
volatile int three = 0; //0X52 STATUS
volatile int pwm = 0;
volatile int walk_handle = 0;
volatile int cal_pas_handle = 0;
volatile int trip_time = 0;
volatile int display_mode = 0;
volatile int display_present = 0;
volatile int brake_stand = 0;
uint16_t checksum = 0;
uint16_t checkhigh = 0;
uint16_t checklow = 0;

int motor_temp = 0;
int fet_temp = 0;
volatile uint16_t bbrs = 0; //bluetooth brakesensor race street control bit
uint16_t human_power = 0;
uint16_t human_power_high = 0;
uint16_t human_power_low = 0;
uint16_t i_test_speed = 0;
uint16_t speed_high = 0;
uint16_t speed_low = 0;

uint16_t fault_code = 0;
uint16_t heartrate = 0;
uint16_t motor_power = 0;
uint16_t motor_power_high = 0;
uint16_t motor_power_low = 0;

uint16_t motor_rpm = 0;

uint16_t motor_rpm_high = 0;
uint16_t motor_rpm_low = 0;

uint16_t cadence_rpm = 0;
uint16_t unit_battery = 0;
uint16_t wheel_diameter = 0;

// Settings
#define BAUDRATE					9600
#define PACKET_HANDLER				1
#define PACKET_HANDLER_P			2

// Threads
static THD_FUNCTION(display_process_thread, arg);
static THD_WORKING_AREA(display_process_thread_wa, 4096);

// Variables
static volatile bool thread_is_running = false;
static volatile bool uart_is_running = false;
static mutex_t send_mutex;
static bool send_mutex_init_done = false;

static volatile int four;
static volatile int six;
static volatile int odo_h;
static volatile int odo_m;
static volatile int odo_l;
static volatile uint32_t app_odo;
static volatile int tt_h;
static volatile int tt_l;
static volatile int watt_h;
static volatile int watt_m;
static volatile int watt_l;
static volatile int crc_h;
static volatile int crc_l;
static volatile int fixed_a;
static volatile int fixed_d;
static volatile int fo_53;
static volatile int se_53;
static volatile int ei_53;
static volatile int ni_53;
volatile int ten_53;
static volatile int crc_h_53;
static volatile int crc_l_53;

uint16_t crc_check52;
uint16_t crc_check53;
static SerialConfig uart_cfg = {
BAUDRATE, 0,
USART_CR2_LINEN, 0 };

void flash_calibration(uint32_t Address, uint8_t Data) {

	//FLASH_Unlock();
	//FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
	//FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_ProgramByte(Address, Data);
	//FLASH_Lock();
}

void erase_flash(void) {
	//For F405 only, Sector 7   	((uint32_t)0x08060000) // Base @ of Sector 7, 128 Kbytes
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
	FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	mc_interface_unlock();
	mc_interface_release_motor();
	utils_sys_lock_cnt();
	timeout_configure_IWDT_slowest();

	FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3);

	FLASH_Lock();
	timeout_configure_IWDT();
	utils_sys_unlock_cnt();
}

int app_get_assis_lv(void) {

	if (flag == -1)
		return -1;

	if (flag == 0xF)
		return 0;
	else if (flag == 0.2)
		return 1;
	else if (flag == 0.3)
		return 2;
	else if (flag == 0.4)
		return 3;
	else if (flag == 0.5)
		return 4;
	else if (flag == 0.6)
		return 5;
	else if (flag == 0.7)
		return 6;
	else if (flag == 0.8)
		return 7;
	else if (flag == 0.9)
		return 8;
	else if (flag == 1)
		return 9;

	return 0;

}

int app_get_total_tt(void) { //Total trip time

	return trip_time;
}

int app_get_odo(void) {

	return app_odo;

}
int app_get_rs_mode(void) { //1 is race
	return mode_display;
}

void ebmx_error_code(void) {

	static volatile int error = 0;

	error = mc_interface_get_fault();

	if (error != FAULT_CODE_NONE) {

		if (error == FAULT_CODE_OVER_VOLTAGE)
			fault_code = 0x1;
		else if (error == FAULT_CODE_UNDER_VOLTAGE) //
			fault_code = 0x2;
		else if (error == FAULT_CODE_ABS_OVER_CURRENT) //
			fault_code = 0x4;
		else if (error == FAULT_CODE_OVER_TEMP_FET) //
			fault_code = 0x5;
		else if (error == FAULT_CODE_OVER_TEMP_MOTOR) //
			fault_code = 0x6;
		else if (error == FAULT_CODE_MCU_UNDER_VOLTAGE) //
			fault_code = 0x9;
		else if (error == FAULT_CODE_HALL_SENSOR) //
			fault_code = 0x10;
		else if (error == FAULT_CODE_FLASH_CORRUPTION) //
			fault_code = 0x14;
		else if (error == FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1) //
			fault_code = 0x15;
		else if (error == FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2) //
			fault_code = 0x16;
		else if (error == FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3) //
			fault_code = 0x17;
		else if (error == FAULT_CODE_UNBALANCED_CURRENTS) //
			fault_code = 0x18;
		else if (error == FAULT_CODE_THROTTLE_VOLTAGE) //
			fault_code = 0x23;
		else if (error == FAULT_CODE_REGEN_VOLTAGE) //
			fault_code = 0x24;
		// dead error code, shall not run

	} else
		fault_code = 0;

}

void sw102_ebmx_reply(void) {

	/**********************************************Begin Send data from controller to display*********************/

	bbrs = get_bbrs;
	i_test_speed = app_get_speed_km_r();
	//i_test_speed = 50;
	if (chVTTimeElapsedSinceX(start_time) > 800000) //8 seconds
		ebmx_error_code();
	else
		fault_code = 0x0;

	motor_temp = (int) mc_interface_temp_motor_filtered() + 20; //To display, +20 means the actual temp;
	fet_temp = (int) mc_interface_temp_fet_filtered() + 20; //To display

	//motor_temp = 199;
	//fet_temp = 199;

	if (motor_temp < 0)
		motor_temp = 0;
	else if (motor_temp > 0xFF)
		motor_temp = 0xFF;
	if (fet_temp < 0)
		fet_temp = 0;
	else if (fet_temp > 0xFF)
		fet_temp = 0xFF; //Display marginal case
	static float kw;

	kw = (mc_interface_read_reset_avg_input_current() // Motor Power To display
	* GET_INPUT_VOLTAGE()) / (float) 1000;
	//kw = 5.4;
	motor_power_high = kw;
	motor_power_low = ((int) (kw * 10) % 10);

	motor_rpm = (abs((int) mc_interface_get_actual_rpm())) / 10;
	//motor_rpm = 123;
	motor_rpm_high = motor_rpm >> 8;
	motor_rpm_low = motor_rpm & 0xFF;

	unit_battery = get_unit_battery;
	static int battery_int, battery_f;

	battery_int = (int) GET_INPUT_VOLTAGE();
	battery_f = ((int) (GET_INPUT_VOLTAGE() * 10) % 10);

	//battery_int = 84;
	//battery_f = 5;

	checksum = 0x5A + three + 0xD + bbrs + i_test_speed + fault_code
			+ motor_temp + fet_temp + motor_power_high + motor_power_low
			+ motor_rpm_high + motor_rpm_low + unit_battery + brake_stand
			+ battery_int + battery_f;
	checkhigh = (checksum & 0xFF00) >> 8;

	checklow = (checksum & 0xFF);

	sdPut(&SD3, 0x3A);	            //1  Fixed
	sdPut(&SD3, 0x5A);		        //2  Fixed
	sdPut(&SD3, three);		        //3  52 or 53
	sdPut(&SD3, 0xD);		     	//4  data length, fixed 0x10
	sdPut(&SD3, bbrs);			    //5

	sdPut(&SD3, i_test_speed);		    //6   speed KM
	sdPut(&SD3, fault_code);	    //7  Error code
	sdPut(&SD3, motor_temp); //8 Motor temperature 0x00 is -20c 0xFF is 235c
	sdPut(&SD3, fet_temp); //9 Controller temperature  0x00 is -20c 0xFF is 235c

	sdPut(&SD3, motor_power_high);	//10 High motor power
	sdPut(&SD3, motor_power_low);   //11 Low motor power
	sdPut(&SD3, motor_rpm_high);	//12 High motor rpm
	sdPut(&SD3, motor_rpm_low);	    //13 Low motor rpm

	sdPut(&SD3, unit_battery);	 	//14 unit and battery cell
	sdPut(&SD3, brake_stand);	//15  brake/kickstand

	sdPut(&SD3, battery_int);	//16  battery integer part
	sdPut(&SD3, battery_f);	//17  battery float part

	sdPut(&SD3, checklow);	//18  Check sum low (sum of all excluding 0x3A)
	sdPut(&SD3, checkhigh);		    //19  Check sum high
	sdPut(&SD3, 0x0D);				//20  Fixed
	sdPut(&SD3, 0x0A);				//21  Fixed

}

void sw102_ebmx(void) {

	three = sdGet(&SD3);  //byte 3 0x52/53
	if (three == 0x52) {
		four = sdGet(&SD3); //byte4  fixed ignore
		pwm = sdGet(&SD3);  //byte5 assist level
		six = sdGet(&SD3); //byte 6  bit1=0, Display showing race mode bit1=1, Display showing street mode
		odo_h = sdGet(&SD3);		// byte 7 ODO High
		odo_m = sdGet(&SD3);	    // byte 8 ODO Mid
		odo_l = sdGet(&SD3);	    // byte 9 ODO Low
		tt_h = sdGet(&SD3);	    // byte 10 Tot trip time high
		tt_l = sdGet(&SD3);	    // byte 11 Tot trip time low *30
		watt_h = sdGet(&SD3);	    // byte 12 Tot watt hour high
		watt_m = sdGet(&SD3);	    // byte 13 Tot watt hour low
		watt_l = sdGet(&SD3);	    // byte 14 Tot watt hour digit
		crc_l = sdGet(&SD3);	    //15 crc check low
		crc_h = sdGet(&SD3);	    //16 crc check high
		fixed_d = sdGet(&SD3);	    //17 0x0D Fixed
		fixed_a = sdGet(&SD3);	    //18 0x0A Fixed

		crc_check52 = two + three + four + pwm + six + odo_h + odo_m + odo_l
				+ tt_h + tt_l + watt_h + watt_m + watt_l;

		if ((crc_h == ((crc_check52 & 0xFF00) >> 8))
				&& (crc_l == (crc_check52 & 0xFF))) {//52crcok safe to execute
			inside0x53 = 0;

			//	commands_printf("52ok");
			sw102_ebmx_reply();
			display_present = 1;
			//	non_stop_error_clear(FAULT_CODE_DISPLAY);

			if (((six & (1 << 4)) >> 4) == 0) {	    //no walk
				if (walk_handle == 0) {
					walk_current = 0;
					walk_handle = 1;
				}

			} else if (((six & (1 << 4)) >> 4) == 1) {	    //walk
				walk_handle = 0;

				walk_current = 0.30;

			}

			if (((six & (1 << 7)) >> 7) == 0) {	    //headlight
				//commands_printf("headlight off");
			} else {
				//commands_printf("headlight on");
			}

			if (((six & (1 << 1)) >> 1) == 0) {	    //race mode
				//commands_printf("Race");
				mode_display = 1;
			} else	    //street
			{
				//commands_printf("Street");
				mode_display = 0;
			}
			app_odo = (odo_h << 16) | (odo_m << 8) | odo_l;
			trip_time = (tt_h << 8) | tt_l;
			static int wh;
			wh = (watt_h << 16) | (watt_m << 8) | watt_l;
			/*
			 commands_printf(" ODO %X %X %X ", odo_h, odo_m, odo_l);
			 commands_printf(" TOT Trip time %X %X", tt_h, tt_l);
			 commands_printf(" wh %i %i %i, combined: %i ", watt_h, watt_m,
			 watt_l, wh);

			 */

			if (pwm == 0x20) {
				//commands_printf("reverse ");
				flag = -1;

			}

			if (pwm == 0x00) {
				//commands_printf("PWM 0");
				flag = 0xF;

			}

			else if (pwm == 0x40) {
				//commands_printf("PWM 1a");
				flag = 0.2;

			} else if (pwm == 0x57) {
				// commands_printf("PWM 2a");
				flag = 0.3;

			} else if (pwm == 0x99) {

				//commands_printf("PWM 1"); //1 =0.4
				flag = 0.4;

			} else if (pwm == 0x85) {
				// commands_printf("PWM 4a");
				flag = 0.5;

			} else if (pwm == 0x9c) {
				// commands_printf("PWM 5a");
				flag = 0.6;

			} else if (pwm == 0xCC) {
				//commands_printf("PWM 2"); //2 0.7
				flag = 0.7;

			} else if (pwm == 0xca) {
				//commands_printf("PWM 7a");
				flag = 0.8;

			} else if (pwm == 0xe1) {
				// commands_printf("PWM 8a");
				flag = 0.9;

			} else if (pwm == 0xff) {
				//commands_printf("PWM 3"); //3=1
				flag = 1;

			}

		}/* else	    //52crc error still reply
		 {
		 sw102_ebmx_reply();

		 }*/

	} else if (three == 0x53) {

		fo_53 = sdGet(&SD3);		//byte 4 fixed 0x07
		set5 = sdGet(&SD3);	 //byte 5 two units battery cell
		set6 = sdGet(&SD3);  //byte 6 rs menu lock
		se_53 = sdGet(&SD3);        //byte 7 ignore
		ei_53 = sdGet(&SD3);        //byte 8 ignore
		ni_53 = sdGet(&SD3);        //byte 9 ignore
		ten_53 = sdGet(&SD3);        //byte 10 kick/brake
		crc_l_53 = sdGet(&SD3);        //byte 11 CRC Low
		crc_h_53 = sdGet(&SD3);        //byte 12 CRC High
		sdGet(&SD3);        //byte 13 0x0D
		sdGet(&SD3);        //byte 14 0x0A

		crc_check53 = two + three + fo_53 + set5 + set6 + se_53 + ei_53 + ni_53
				+ ten_53;
		if ((crc_h_53 == ((crc_check53 & 0xFF00) >> 8))
				&& (crc_l_53 == (crc_check53 & 0xFF))) { //53 crc ok safe to execute
			//	commands_printf("ebmx 53crcok");
			crc_set5 = set5;
			crc_set6 = set6;
			crc_ten_53 = ten_53;
			inside0x53 = 1;

			sw102_ebmx_reply();

			//	if (chVTTimeElapsedSinceX(start_time) > 300000) //3 seconds
			//	{
			//	if (!done_52)
			//	}

		} /*else			//53crc error still reply
		 {
		 sw102_ebmx_reply();

		 }*/

	}

}
int check_display_present(void) {
	return display_present;
}
void switch_mode(void) {
	mode_display = !mode_display;
}
void change_mode(void) {
	if (!display_present) {		//only allow change mode if not using display

		if (flag == 0xf) {
			flag = 0.4;
			return;
		}
		if (flag == 0.4) {
			flag = 0.7;
			return;
		}
		if (flag == 0.7) {
			flag = 1;
			return;
		}
		if (flag == 1) {
			flag = -1;
			return;
		}
		if (flag == -1) {
			flag = 0xf;
			return;
		}

	}
}
void reboot(void) {
	__disable_irq();
	for (;;) {
	}
}
void app_uartcomm_display_start(void) {

	if (!thread_is_running) {
		chThdCreateStatic(display_process_thread_wa,
				sizeof(display_process_thread_wa),
				NORMALPRIO, display_process_thread, NULL);
		thread_is_running = true;
	}

	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN,
			PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST
					| PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN,
			PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST
					| PAL_STM32_PUDR_PULLUP);

	uart_is_running = true;
}

static THD_FUNCTION(display_process_thread, arg) {
	(void) arg;

	chRegSetThreadName("uartcomm proc display");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0),
	CHN_INPUT_AVAILABLE);

	int phase_ah = 0;
	int phase_al = 0;
	int phase_bh = 0;
	int phase_bl = 0;
	int phase_ch = 0;
	int phase_cl = 0;
	int phase_crc_h = 0;
	int phase_crc_l = 0;
	int phase_crc = 0;

	int s0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15,
			s16, s17, s18, s19, s20, s21;

	int clear_a;
	int clear_b;
	int clear_c;
	int testing = 0;
	int display_loop = 0;
	start_time = chVTGetSystemTimeX();

	//chThdSleepMilliseconds(300);
	for (;;) {
		// chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(10));
		while (display_loop) {
			start = sdGet(&SD3); //byte 1 0x3A fixed

			if (start == 0x3A) {
				two = sdGet(&SD3);  //byte 2 0x2A fixed*/
				if (two == 0x5A) {
					sw102_ebmx();
				}
			}
			timeout_reset();
		}
		start = sdGet(&SD3); //byte 1 0x3A fixed

		if (start == 0x3A) {
			two = sdGet(&SD3);  //byte 2 0x2A fixed*/
			if (two == 0x5A) {
				sw102_ebmx();
				display_loop = 1;
			}
		} else if (start == 0x2A) { //e.g. 2aFF0376039e03ff0345
			two = sdGet(&SD3);
			if (two == 0xFF) {
				phase_ah = sdGet(&SD3);
				phase_al = sdGet(&SD3);

				phase_bh = sdGet(&SD3);
				phase_bl = sdGet(&SD3);

				phase_ch = sdGet(&SD3);
				phase_cl = sdGet(&SD3);

				phase_crc_h = sdGet(&SD3);
				phase_crc_l = sdGet(&SD3);

				phase_crc = start + two + phase_ah + phase_al + phase_bh
						+ phase_bl + phase_ch + phase_cl;
				if ((phase_crc_h == ((phase_crc & 0xFF00) >> 8))
						&& (phase_crc_l == (phase_crc & 0xFF))) { //crc ok received calibration data
					if ((*(uint8_t*) set_address) == 0xFF) { //check if empty
						/*	FLASH_Unlock();
						 FLASH_ClearFlag(
						 FLASH_FLAG_EOP | FLASH_FLAG_OPERR
						 | FLASH_FLAG_WRPERR |
						 FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR
						 | FLASH_FLAG_PGSERR);
						 mc_interface_unlock();
						 mc_interface_release_motor();
						 utils_sys_lock_cnt();
						 timeout_configure_IWDT_slowest();

						 flash_calibration(set_address, 0x00);
						 flash_calibration(phase_ahf, phase_ah);
						 flash_calibration(phase_alf, phase_al);
						 flash_calibration(phase_bhf, phase_bh);
						 flash_calibration(phase_blf, phase_bl);
						 flash_calibration(phase_chf, phase_ch);
						 flash_calibration(phase_clf, phase_cl);

						 FLASH_Lock();
						 timeout_configure_IWDT();
						 utils_sys_unlock_cnt();
						 //update_phase_cal();*/
					}
					sdPut(&SD3, 0x2F);
					sdPut(&SD3, *(uint8_t* )phase_ahf);
					sdPut(&SD3, *(uint8_t* )phase_alf);
					sdPut(&SD3, *(uint8_t* )phase_bhf);
					sdPut(&SD3, *(uint8_t* )phase_blf);
					sdPut(&SD3, *(uint8_t* )phase_chf);
					sdPut(&SD3, *(uint8_t* )phase_clf);
					sdPut(&SD3, 0xff);
				}
			}

		} else if (start == 0x4A) { //4A0102030050 CLEAR CALIBRATION DATA
			clear_a = sdGet(&SD3);
			if (clear_a == 0x01) {
				clear_b = sdGet(&SD3);
				clear_c = sdGet(&SD3);
				phase_crc_h = sdGet(&SD3);
				phase_crc_l = sdGet(&SD3);
				phase_crc = start + clear_a + clear_b + clear_c;
				if ((phase_crc_l == phase_crc) && (phase_crc == 0x50)) { //4A+01+02+03  =0x50, CLEAR CALIBRATION DATA
					sdPut(&SD3, 0xFF);
					//erase_flash(); //Shall restart controller
					//reboot();
				}
			}
		} else if (start == 0x5A) { //5A1A5200FF // uart1 performance testing & turn on 5v
			two = sdGet(&SD3);
			if (two == 0x1A) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1C5) {
					//all change to x1pro settings to run the test, volatile
					//mc_motor_x1pro_volatile();

					palSetPad(GPIOC, 15);   //turn on 5V
					palWritePad(GPIOC, 15, 1);
					palSetPad(GPIOC, 15);   //turn on 5V
					change_ntc();   //set to kty84/130 for temp. checking
					sdPut(&SD3, 0x5A);
					sdPut(&SD3, 0x1A);
					sdPut(&SD3, 0xAB);
					sdPut(&SD3, 0xCD);

				}
			}

		} else if (start == 0x5B) { //5B1A5200FF // uart1 OQC testing & turn on 5v
			two = sdGet(&SD3);
			if (two == 0x1A) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1C6) {
					palSetPad(GPIOC, 15);   //turn on 5V
					palWritePad(GPIOC, 15, 1);
					palSetPad(GPIOC, 15);   //turn on 5V
					change_ntc();   //set to kty84/130 for temp. checking

					sdPut(&SD3, 0x5B);
					sdPut(&SD3, 0x1A);
					sdPut(&SD3, 0xAB);
					sdPut(&SD3, 0xCD);

				}
			}

		}

		else if (start == 0x6A) { //6A1B5200FF // TURN ON MOTOR ASSIST
			two = sdGet(&SD3);
			if (two == 0x1B) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1d6) {
					flag = 1; //gear3
					mode_display = 1; //race
					oqc = 0; //logic for focopenloop
					testing = 1;

					sdPut(&SD3, 0x6A);
					sdPut(&SD3, 0x1B);
					sdPut(&SD3, 0xAB);
					sdPut(&SD3, 0xCD);

				}
			}

		} else if (start == 0x7A) { //7A1C5200FF // TURN OFF MOTOR ASSIST
			two = sdGet(&SD3);
			if (two == 0x1C) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1e7) {
					flag = 0xF;
					oqc = 0; //logic for foc openloop
					testing = 1;

					sdPut(&SD3, 0x7A);
					sdPut(&SD3, 0x1C);
					sdPut(&SD3, 0xAB);
					sdPut(&SD3, 0xCD);

				}
			}

		}

		else if (start == 0x7B) { //7B1C5200FF // Change motor parameter to WA motor
			two = sdGet(&SD3);
			if (two == 0x1C) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1e8) {
					flag = 1; //gear3
					mode_display = 1; //race
					oqc_loading_test();
					changetowa();

					sdPut(&SD3, 0x7B);
					sdPut(&SD3, 0x1C);
					sdPut(&SD3, 0xAB);
					sdPut(&SD3, 0xCD);

				}
			}

		}

		else if (start == 0x8A) { //8A ASCII
			two = sdGet(&SD3);
			if (two == 0xFF) {
				s0 = sdGet(&SD3);
				s1 = sdGet(&SD3);
				s2 = sdGet(&SD3);
				s3 = sdGet(&SD3);
				s4 = sdGet(&SD3);
				s5 = sdGet(&SD3);
				s6 = sdGet(&SD3);
				s7 = sdGet(&SD3);
				s8 = sdGet(&SD3);
				s9 = sdGet(&SD3);
				s10 = sdGet(&SD3);
				s11 = sdGet(&SD3);
				s12 = sdGet(&SD3);
				s13 = sdGet(&SD3);
				s14 = sdGet(&SD3);
				s15 = sdGet(&SD3);
				s16 = sdGet(&SD3);
				s17 = sdGet(&SD3);
				s18 = sdGet(&SD3);
				s19 = sdGet(&SD3);
				s20 = sdGet(&SD3);
				//s21 = sdGet(&SD3);
				phase_crc_h = sdGet(&SD3);
				phase_crc_l = sdGet(&SD3);

				phase_crc = start + two + s0 + s1 + s2 + s3 + s4 + s5 + s6 + s7
						+ s8 + s9 + s10 + s11 + s12 + s13 + s14 + s15 + s16
						+ s17 + s18 + s19 + s20;
				if ((phase_crc_h == ((phase_crc & 0xFF00) >> 8))
						&& (phase_crc_l == (phase_crc & 0xFF))) { //crc ok received SERIAL NO.
					if (*(uint8_t*) serial_p == 0xff) { //check if first pointer is empty
						FLASH_Unlock();
						FLASH_ClearFlag(
								FLASH_FLAG_EOP | FLASH_FLAG_OPERR
										| FLASH_FLAG_WRPERR |
										FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR
										| FLASH_FLAG_PGSERR);
						mc_interface_unlock();
						mc_interface_release_motor();
						utils_sys_lock_cnt();
						timeout_configure_IWDT_slowest();

						flash_calibration(serial_p, s0);
						flash_calibration(serial_p + 1, s1);
						flash_calibration(serial_p + 2, s2);
						flash_calibration(serial_p + 3, s3);
						flash_calibration(serial_p + 4, s4);
						flash_calibration(serial_p + 5, s5);
						flash_calibration(serial_p + 6, s6);
						flash_calibration(serial_p + 7, s7);
						flash_calibration(serial_p + 8, s8);
						flash_calibration(serial_p + 9, s9);
						flash_calibration(serial_p + 10, s10);
						flash_calibration(serial_p + 11, s11);
						flash_calibration(serial_p + 12, s12);
						flash_calibration(serial_p + 13, s13);
						flash_calibration(serial_p + 14, s14);
						flash_calibration(serial_p + 15, s15);
						flash_calibration(serial_p + 16, s16);
						flash_calibration(serial_p + 17, s17);
						flash_calibration(serial_p + 18, s18);
						flash_calibration(serial_p + 19, s19);
						flash_calibration(serial_p + 20, s20);
						//flash_calibration(serial_p + 21, s21);

						FLASH_Lock();
						timeout_configure_IWDT();
						utils_sys_unlock_cnt();
						serial_no_generation();
						sdPut(&SD3, 0x8F);
					} else
						//first pointer is not empty, reject
						sdPut(&SD3, 0x4F);
				}
			}
		} else if (start == 0x8B) { //8b 8b1d5200FF // Query serial no
			two = sdGet(&SD3);
			if (two == 0x1d) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x1f9) {
					sdPut(&SD3, 0x8B);
					char *p = serial_p;
					for (int i = 0; i < 21; i++) {	//serial no.
						sdPut(&SD3, *(p + i));
					}
					sdPut(&SD3, 0xFF);
				}
			}
		} else if (start == 0x9A) { //9A1e5200FF // Turn on 90A openloop
			two = sdGet(&SD3);
			if (two == 0x1e) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x209) {
					flag = 1; //gear 3
					mode_display = 1; //race
					oqc = 1;
					sdPut(&SD3, 0x1e);
					oqc_current_test();
					changetome1803(); //need clear error first, voltage, current limit
					mcpwm_foc_set_openloop(700, 2000);
				}
			} else if (two == 0x1f) {  //9A1f5200FF // Turn off 90A openloop
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x20A) {
					flag = 0xf; //adc thread will off
					oqc = 0;
					sdPut(&SD3, 0x1f);
					mc_interface_release_motor();
				}

			}
		} else if (start == 0x9B) { //0x9b1f5200ff  // Report phase current
			two = sdGet(&SD3);
			if (two == 0x1f) {
				if ((start + two + sdGet(&SD3) + sdGet(&SD3) + sdGet(&SD3))
						== 0x20b) {
					static uint16_t crc;
					static int iah, ibh, ich;
					static int ial, ibl, icl, fti_ia, fti_ib, fti_ic; //float to int
					static float temp_ia, temp_ib, temp_ic;

					temp_ia = mcpwm_foc_get_ia2() * 10;
					fti_ia = temp_ia;
					iah = (fti_ia >> 8) & 0xff;
					ial = fti_ia & 0xff;

					/*	iah = temp_ia; //float to int truncate
					 fti_ia = temp_ia * (float) 10;
					 ial = ((int) fti_ia) % 10; //extract first decimal point
					 */
					temp_ib = mcpwm_foc_get_ib2() * 10;
					fti_ib = temp_ib;
					ibh = (fti_ib >> 8) & 0xff;
					ibl = fti_ib & 0xff;
					/*	ibh = temp_ib;
					 fti_ib = temp_ib * (float) 10;
					 ibl = ((int) fti_ib) % 10;
					 */
					temp_ic = mcpwm_foc_get_ic2() * 10;
					fti_ic = temp_ic;
					ich = (fti_ic >> 8) & 0xff;
					icl = fti_ic & 0xff;
					/*		ich = temp_ic;
					 fti_ic = temp_ic * (float) 10;
					 icl = ((int) fti_ic) % 10;*/
					crc = iah + ial + ibh + ibl + ich + icl;
					/*

					 commands_printf(
					 "A high %x A low %x B high %x B Low %x C Hi %x C low %x",
					 iah, ial, ibh, ibl, ich, icl);*/
					sdPut(&SD3, 0x9b);
					sdPut(&SD3, iah);
					sdPut(&SD3, ial);
					sdPut(&SD3, ibh);
					sdPut(&SD3, ibl);
					sdPut(&SD3, ich);
					sdPut(&SD3, icl);
					sdPut(&SD3, (crc & 0xFF00) >> 8);
					sdPut(&SD3, crc & 0xFF);

				}

			}

		}
	}
}
