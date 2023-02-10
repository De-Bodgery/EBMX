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

extern app_configuration appconf;
char *product_serial;
extern const uint32_t serial_p;
extern volatile float speed_km_h, avg_speed_km, ia_testing, reversed;
extern volatile int speed_time_interval_ms, frequency_speed, brake_stand;

extern volatile float sintest;
extern volatile uint32_t time_result;
extern volatile int zeropoint;
extern volatile int mode_display;
extern volatile float flag;
volatile int five_v_shutdown = 0;
volatile float throttle_watt_limit, throttle_torque_limit, speed_limit,
		battery_current, regen_current, reverse_strength, avg_voltage;
volatile int get_bbrs, get_unit_battery, once, brake_apply;
volatile int brake_lock, brake_unlock, stand_lock, stand_unlock, tilt_lock,
		tilt_unlock, display_lock, display_unlock;
volatile systime_t start_time;

static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread

volatile imu_config imu_conf;
volatile balance_config app_balance_conf;
volatile ppm_config app_ppm_conf;
volatile nrf_config app_nrf_conf;
volatile chuk_config app_chuk_conf;
extern volatile pas_config config;
int thumb_throttle_type(void) {
	return appconf.send_can_status;
}
void imu_init(imu_config *set) {
	imu_conf = *set;
}
void app_balance_configure(balance_config *conf) {
	app_balance_conf = *conf;
}
void app_ppm_configure(ppm_config *conf) {
	app_ppm_conf = *conf;
}
void rfhelp_update_conf(nrf_config *conf) {
	app_nrf_conf = *conf;
}
void app_nunchuk_configure(chuk_config *conf) {
	app_chuk_conf = *conf;

}

void app_pas_configure(pas_config *conf) {
	config = *conf;

}
void serial_no_generation(void) {

	char serial[25];
	char *p = serial_p;	//(uint32_t*) serial_p
	for (int i = 0; i < 21; i++) {	//serial no.
		serial[i] = *(p + i);
	}
	product_serial = (char*) malloc(1 + strlen(serial) + strlen(HW_NAME));
	strcpy(product_serial, HW_NAME);
	strcat(product_serial, serial);
}
int get_battery_series(void) {

	if (app_chuk_conf.ctrl_type == 0)
		return 16;
	else if (app_chuk_conf.ctrl_type == 1)
		return 20;
	else if (app_chuk_conf.ctrl_type == 2)
		return 22;
	else if (app_chuk_conf.ctrl_type == 3)
		return 24;

	return 99;	//bypassed

}
int slow_voltage_cutout(void) {
	return app_chuk_conf.multi_esc;
}
int stock_switch(void) {
	return appconf.permanent_uart_enabled; //0 is off 1 is on
}
int bike_frame(void) {
	return appconf.can_baud_rate; //0 is surron 1 is talaria
}
int sprocket_size(void) {
	static int sprocket_size;
	sprocket_size = appconf.shutdown_mode;
	if (sprocket_size == 0)
		return 42;
	else if (sprocket_size == 1)
		return 48;
	else if (sprocket_size == 2)
		return 52;
	else if (sprocket_size == 3)
		return 54;
	else if (sprocket_size == 4)
		return 55;
	else if (sprocket_size == 5)
		return 56;
	else if (sprocket_size == 6)
		return 58;
	else if (sprocket_size == 7)
		return 60;
	else if (sprocket_size == 8)
		return 64;
	else if (sprocket_size == 9)
		return 68;
	else if (sprocket_size == 10)
		return 70;
	else if (sprocket_size == 11) //custom
		return 999;

	return 0;

}
void wheel_setting(void) {
	if (sprocket_size() >= 999) //custom sprocket, update gear ratio manually
		update_gear_wheel(app_balance_conf.startup_roll_tolerance,
				app_balance_conf.startup_speed);
	else { //calculate sprocket size/bike frame to gear ratio
		if (bike_frame() == CAN_SURRON) {
			if (sprocket_size() == 42)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						6.651);
			else if (sprocket_size() == 48)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance, 7.6);
			else if (sprocket_size() == 52)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.233);
			else if (sprocket_size() == 54)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.551);
			else if (sprocket_size() == 55)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.71);
			else if (sprocket_size() == 56)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.868);
			else if (sprocket_size() == 58)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						9.184);
			else if (sprocket_size() == 64)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						10.135);
			else if (sprocket_size() == 68)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						10.768);
			else if (sprocket_size() == 70)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						11.085);

		} else if (bike_frame() == CAN_TALARIA) {

			if (sprocket_size() == 42)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						6.651);
			else if (sprocket_size() == 48)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance, 7.6);
			else if (sprocket_size() == 52)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.233);
			else if (sprocket_size() == 55)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						8.708);
			else if (sprocket_size() == 58)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						9.184);
			else if (sprocket_size() == 64)
				update_gear_wheel(app_balance_conf.startup_roll_tolerance,
						10.133);

		}
	}

}

int motor_type(void) {
	return imu_conf.mode;
}

float app_get_speed_km_r(void) {
	return mc_interface_get_speed() * (float) 3.6; //1m/s =3.6km/h
}
void misc_control(void) {
//commands_printf("temperature %i", config.ctrl_type);
//commands_printf("speed  %i", config.invert_pedal_direction);
	if (app_chuk_conf.throttle_exp_mode == 0)
		brake_stand &= ~(1UL << 0); //clear bit 0 stand sensor off
	else
		brake_stand |= 1UL << 0;  //set bit 0 stand sensor on

	if (app_nrf_conf.send_crc_ack == 0)
		brake_stand &= ~(1UL << 4); //clear bit 0 brake sensor off
	else
		brake_stand |= 1UL << 4;  //set bit 0 brake sensor on

	if (config.ctrl_type) { // Temperature Unit 1, C Temperature
		get_unit_battery |= 1UL << 0;  //set bit 0 C
	} else
		//0, F Temperature
		get_unit_battery &= ~(1UL << 0); //clear bit 0  F

	if (config.invert_pedal_direction) { //Speed Unit 1, miles
		get_unit_battery |= 1UL << 1;  //set bit 1 miles
	} else {
		//0, km
		get_unit_battery &= ~(1UL << 1); //clear bit 1  km
	}

	if (app_chuk_conf.ctrl_type == 0) //battery cells
			{ //0000xxxx
		get_unit_battery &= ~(1UL << 4); //clear bit
		get_unit_battery &= ~(1UL << 5); //clear bit
		get_unit_battery &= ~(1UL << 6); //clear bit
		get_unit_battery &= ~(1UL << 7); //clear bit

	} else if (app_chuk_conf.ctrl_type == 1) { //0001xxxx

		get_unit_battery |= 1UL << 4;  //set bit
		get_unit_battery &= ~(1UL << 5); //clear bit
		get_unit_battery &= ~(1UL << 6); //clear bit
		get_unit_battery &= ~(1UL << 7); //clear bit

	} else if (app_chuk_conf.ctrl_type == 2) {         //0010xxxx

		get_unit_battery &= ~(1UL << 4); //clear bit
		get_unit_battery |= 1UL << 5;  //set bit
		get_unit_battery &= ~(1UL << 6); //clear bit
		get_unit_battery &= ~(1UL << 7); //clear bit

	} else if (app_chuk_conf.ctrl_type == 3) {  //0011xxxx

		get_unit_battery |= 1UL << 4;  //set bit
		get_unit_battery |= 1UL << 5;  //set bit
		get_unit_battery &= ~(1UL << 6); //clear bit
		get_unit_battery &= ~(1UL << 7); //clear bit

	}
//display settings lock
	if (app_ppm_conf.median_filter == 0) { //no display settings lock
		get_bbrs |= 1UL << 4;
		get_bbrs &= ~(1UL << 5); //set bit 45 as 10
	} else if (app_ppm_conf.median_filter == 1) { //hv display settings lock
		get_bbrs |= 1UL << 4;
		get_bbrs |= 1UL << 5; //set bit 45 as 11
	}
	/*
	 if (rx == 1) {
	 get_bbrs |= 1UL << 6; //set bit 6 as 1 bluetooth
	 } else
	 get_bbrs &= ~(1UL << 6);*/
//mode after reboot
	if (app_nrf_conf.power == 0) { //mode before reboot
		get_bbrs |= 1UL << 2;
		get_bbrs |= 1UL << 3; //set bit 23 as 11
	} else if (app_nrf_conf.power == 1) { //set to street after reboot
		get_bbrs |= 1UL << 2;
		get_bbrs &= ~(1UL << 3); //set bit 23 as 10
		if (once == 0) {
			once = 1;
			if (chVTTimeElapsedSinceX(start_time) < 30000) //3 seconds
					{
				rs_handler(0); //set to street
			}
		}

	} else if (app_nrf_conf.power == 2) { //set to race after reboot
		get_bbrs &= ~(1UL << 2);
		get_bbrs &= ~(1UL << 3); //set bit 23 as 00
		if (once == 0) {
			once = 1;
			if (chVTTimeElapsedSinceX(start_time) < 30000) //3 seconds
					{
				rs_handler(1); //set to race
			}
		}
	}

	if (config.sensor_type) { //race street mode switch
		get_bbrs |= 1UL << 1; //set bit1=1 race
	} else {
		get_bbrs &= ~(1UL << 1); //clear bit1=0 street
	}
}

void brake_sensor_control(void) {

	if (app_nrf_conf.send_crc_ack) {	//brake sensor enable

		if (palReadPad(BRAKE_IO, BRAKE_PIN)) {

			brake_apply = 0;	//brake applied
			get_bbrs |= 1UL << 7;  //set bbrs bit 7 brake sensor on

		} else {

			brake_apply = 1;
			get_bbrs &= ~(1UL << 7); //clear bbrs bit 7 brake sensor off

		}
	} else { //brake sensor disable
		brake_apply = 1;
		get_bbrs &= ~(1UL << 7); //clear bbrs bit 7 brake sensor off
	}

}

int get_ntc(void) {

	return app_ppm_conf.safe_start; //1 is on 0 is off

}

void wattage_torque_speed_mode_control(void) {

	if (flag == 0xF) //display present and gear 0
			{
		mc_interface_lock();
	} else {
		mc_interface_unlock();
	}
	if (mode_display) {	//  race mode
		process_speed_limit(app_ppm_conf.pulse_end); //ok
		battery_current = app_balance_conf.torquetilt_on_speed;
		regen_current = app_balance_conf.fault_adc2 * 2; //100%*2 to 200A
		reverse_strength = config.ramp_time_neg * -0.005; //50% max

		if (flag == 0.4) {
			throttle_watt_limit = app_chuk_conf.ramp_time_pos;     //1  ok

			throttle_torque_limit = app_chuk_conf.ramp_time_neg;   //1  ok
		} else if (flag == 0.7) {
			throttle_watt_limit = app_chuk_conf.smart_rev_max_duty;  //2

			throttle_torque_limit = app_chuk_conf.smart_rev_ramp_time;

		} else if (flag == 1) {
			throttle_watt_limit = app_balance_conf.roll_steer_erpm_kp; //3

			throttle_torque_limit = app_balance_conf.yaw_current_clamp;

		} else if (flag == 2) { //race testing
			throttle_watt_limit = 5000; //3

			throttle_torque_limit = 100;

		} else if (flag == -1) {

			throttle_watt_limit = app_chuk_conf.ramp_time_pos; //1  ok for reverse current/watt
			throttle_torque_limit = app_chuk_conf.ramp_time_neg;   //1  ok
		}

		else { //0xf  //did not connect display for some reason
			   //brake_apply = 0;

		}
	} else   //Street
	{
		process_speed_limit(app_ppm_conf.pulse_start);  //ok
		battery_current = app_balance_conf.torquetilt_angle_limit;
		regen_current = app_balance_conf.fault_adc1 * 2; //100%*2 to 200A
		reverse_strength = config.ramp_time_pos * -0.005; //50% max

		if (flag == 0.4) {

			throttle_watt_limit = imu_conf.accel_confidence_decay;  //1

			throttle_torque_limit = imu_conf.mahony_kp;

		} else if (flag == 0.7) {

			throttle_watt_limit = imu_conf.rot_roll;   //2

			throttle_torque_limit = imu_conf.rot_pitch;

		} else if (flag == 1) {

			throttle_watt_limit = app_ppm_conf.tc_max_diff;   //3  ok

			throttle_torque_limit = app_ppm_conf.hyst;        //   ok

		}

		else if (flag == -1) {

			throttle_watt_limit = imu_conf.accel_confidence_decay; //1  ok for reverse current/watt
			throttle_torque_limit = imu_conf.mahony_kp;   //1  ok
		}

		else {  //0xf //did not connect display for some reason
			//brake_apply = 0;
			throttle_watt_limit = app_ppm_conf.tc_max_diff;   //3  ok
			throttle_torque_limit = app_ppm_conf.hyst;        //3  ok
		}
	}
	utils_truncate_number(&throttle_torque_limit, 0, get_motor_current_limit());
	update_wattage_current_regen(throttle_watt_limit, throttle_torque_limit,
			battery_current, regen_current);

}

void power_supply_12(void) {

	if (app_balance_conf.multi_esc == 0)			//12v pump 1 off
		palClearPad(P12V_OUT1_IO, P12V_OUT1_PIN);
	else if (app_balance_conf.multi_esc == 1) {
		if (mc_interface_temp_motor_filtered() >= config.update_rate_hz)//target motor temp
			palSetPad(P12V_OUT1_IO, P12V_OUT1_PIN);
		else
			palClearPad(P12V_OUT1_IO, P12V_OUT1_PIN);
	} else if (app_balance_conf.multi_esc == 2) {
		palSetPad(P12V_OUT1_IO, P12V_OUT1_PIN);
	}

	if (app_ppm_conf.ctrl_type == 0)			//2
		palClearPad(P12V_OUT2_IO, P12V_OUT2_PIN);
	else
		palSetPad(P12V_OUT2_IO, P12V_OUT2_PIN);

	if (app_ppm_conf.throttle_exp_mode == 0)			//3
		palClearPad(P12V_OUT3_IO, P12V_OUT3_PIN);
	else
		palSetPad(P12V_OUT3_IO, P12V_OUT3_PIN);

	if (app_nrf_conf.crc_type == 0)			//4
		palClearPad(P12V_OUT4_IO, P12V_OUT4_PIN);
	else
		palSetPad(P12V_OUT4_IO, P12V_OUT4_PIN);

}
void stand_tilt_sensor(void) {
//commands_printf("Tilt Sensor %i", app_nrf_conf.retry_delay);
//commands_printf("Stand Sensor %i", app_chuk_conf.throttle_exp_mode);

	if (app_nrf_conf.retry_delay == 0) {			//tilt sensor
		mc_interface_unlock();
	} else {			//on
		if (palReadPad(TILT_IO, TILT_PIN)) {
			mc_interface_lock();
		} else {
			mc_interface_unlock();
		}
	}

	if (app_chuk_conf.throttle_exp_mode == 0) {			//stand sensor
		mc_interface_unlock();
	} else {			//on
		if (!palReadPad(STAND_IO, STAND_PIN)) {			//0 is standing
			mc_interface_lock();
		} else {
			mc_interface_unlock();
		}

	}

}

void fw_sensor(void) {

	update_fw_sensor(app_balance_conf.booster_current, app_ppm_conf.multi_esc);

}

void five_v_protection(void) {   //5v short circuit protection
	if (palReadPad(GPIOC, 14) == 0) {
		if (five_v_shutdown == 0) {
			palClearPad(GPIOC, 15);
			palWritePad(GPIOC, 15, 0);
		}
		five_v_shutdown = 1;   //stop pulling down for the oqc test
	} else
		five_v_shutdown = 0;

}

void battery_series(void) {
	if (app_chuk_conf.ctrl_type == 0) {
		update_battery(app_balance_conf.torquetilt_filter,
				app_balance_conf.turntilt_strength);
	}
	if (app_chuk_conf.ctrl_type == 1) {
		update_battery(app_balance_conf.turntilt_angle_limit,
				app_balance_conf.turntilt_start_angle);
	}
	if (app_chuk_conf.ctrl_type == 2) {
		update_battery(app_balance_conf.torquetilt_off_speed,
				app_balance_conf.torquetilt_strength);
	}
	if (app_chuk_conf.ctrl_type == 3) {
		update_battery(app_balance_conf.booster_angle,
				app_balance_conf.booster_ramp);
	}
	if (app_chuk_conf.ctrl_type == 4) {			//bypass

		update_battery(MCCONF_L_MIN_VOLTAGE, MCCONF_L_MAX_VOLTAGE);

	}

}

void street_page_testing(void) {

	commands_printf("speed limit %.2f", app_ppm_conf.pulse_start);
	commands_printf("throttle watt limit 1 %.2f",
			imu_conf.accel_confidence_decay);
	commands_printf("throttle watt limit 2 %.2f", imu_conf.rot_roll);
	commands_printf("throttle watt limit 3 %.2f", app_ppm_conf.tc_max_diff);

	commands_printf("throttle current limit 1 %.2f", imu_conf.mahony_kp);
	commands_printf("throttle current limit 2 %.2f", imu_conf.rot_pitch);
	commands_printf("throttle current limit 3 %.2f", app_ppm_conf.hyst);

	commands_printf("Regen Strength %.2f", app_balance_conf.fault_adc1);
	commands_printf("Battery Current Limit %.2f",
			app_balance_conf.torquetilt_angle_limit);

	commands_printf("Reverse %.2f", config.ramp_time_pos);

}

void race_page_testing(void) {

	commands_printf("speed limit %.2f", app_ppm_conf.pulse_end);
	commands_printf("throttle watt limit 1 %.2f", app_chuk_conf.ramp_time_pos);
	commands_printf("throttle watt limit 2 %.2f",
			app_chuk_conf.smart_rev_max_duty);
	commands_printf("throttle watt limit 3 %.2f",
			app_balance_conf.roll_steer_erpm_kp);

	commands_printf("throttle current limit 1 %.2f",
			app_chuk_conf.ramp_time_neg);
	commands_printf("throttle current limit 2 %.2f",
			app_chuk_conf.smart_rev_ramp_time);
	commands_printf("throttle current limit 3 %.2f",
			app_balance_conf.yaw_current_clamp);

	commands_printf("Regen Strength %.2f", app_balance_conf.fault_adc2);
	commands_printf("Battery Current Limit %.2f",
			app_balance_conf.torquetilt_on_speed);
	commands_printf("Reverse %.2f", config.ramp_time_neg);

}

void peripherals_page_testing(void) {

	commands_printf("display model %i", app_nrf_conf.speed);
	commands_printf("temperature unit %i", config.ctrl_type);
	commands_printf("speed unit %i", config.invert_pedal_direction);
	commands_printf("Race/Street Mode %i", config.sensor_type);
	commands_printf("Race/Street Mode Lock%i", app_ppm_conf.median_filter);
	commands_printf("R/S Mode After Reboot%i", app_nrf_conf.power);
	commands_printf("Brake Sensor%i", app_nrf_conf.send_crc_ack);

	/*commands_printf("Invert Brake Sensor	%i",
	 app_adc_conf.rev_button_inverted);*/

	commands_printf("Wheel Diameter %.2f",
			app_balance_conf.startup_roll_tolerance);
	commands_printf("Gear Ratio %.2f", app_balance_conf.startup_speed);

	commands_printf("Tilt Sensor %i", app_nrf_conf.retry_delay);
	commands_printf("Stand Sensor %i", app_chuk_conf.throttle_exp_mode);
	commands_printf("Battery Series %i", app_chuk_conf.ctrl_type);
	commands_printf("16sMinimum Battery Voltage (EBMX/Dealers Only)%.2f",
			app_balance_conf.torquetilt_filter);
	commands_printf("16sMaximum Battery Voltage (EBMX/Dealers Only) %.2f",
			app_balance_conf.turntilt_strength);

	commands_printf("20sMinimum Battery Voltage (EBMX/Dealers Only)%.2f",
			app_balance_conf.turntilt_angle_limit);
	commands_printf("20sMaximum Battery Voltage (EBMX/Dealers Only) %.2f",
			app_balance_conf.turntilt_start_angle);

	commands_printf("22sMinimum Battery Voltage (EBMX/Dealers Only)%.2f",
			app_balance_conf.torquetilt_off_speed);
	commands_printf("22sMaximum Battery Voltage (EBMX/Dealers Only) %.2f",
			app_balance_conf.torquetilt_strength);

	commands_printf("24sMinimum Battery Voltage (EBMX/Dealers Only)%.2f",
			app_balance_conf.booster_angle);
	commands_printf("24sMaximum Battery Voltage (EBMX/Dealers Only) %.2f",
			app_balance_conf.booster_ramp);

	commands_printf("12v Output 0%i", imu_conf.type);
	commands_printf("Coolant Pump 12v Output 1 %i", app_balance_conf.multi_esc);
	commands_printf("Coolant Pump Start Temperature (Motor) %i",
			config.update_rate_hz);
	commands_printf("12v Output 2 %i", app_ppm_conf.ctrl_type);
	commands_printf("12v Output 3 %i", app_ppm_conf.throttle_exp_mode);
	commands_printf("12v Output 4 %i", app_nrf_conf.crc_type);

}

void advanced_motor_page_test(void) {

	commands_printf("Motor Model %i", imu_conf.mode);
	commands_printf("Motor Temperature Sensor%i", app_ppm_conf.safe_start);
	commands_printf("Motor Hall/Encoder Sensor %i", app_ppm_conf.multi_esc);
	commands_printf("FW %.2f", app_balance_conf.booster_current);

}

void app_control_init(void) {

// Start the example thread
	chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
	NORMALPRIO, example_thread, NULL);
}

static THD_FUNCTION(example_thread, arg) {
	(void) arg;
	start_time = chVTGetSystemTimeX();
	chRegSetThreadName("APP_Control");
	Configure_PC13();	//talaria encoder
	//float absolute = 0;
	hall_sensor_error_check_once();
	app_testing_init();

	serial_no_generation();   //generate serial no.
	//update_phase_cal();   // update phase current calibration

	palClearPad(P12V_OUT1_IO, P12V_OUT1_PIN);
	palClearPad(P12V_OUT2_IO, P12V_OUT2_PIN);
	palClearPad(P12V_OUT3_IO, P12V_OUT3_PIN);
	palClearPad(P12V_OUT4_IO, P12V_OUT4_PIN);

	for (;;) {
		misc_control();
		wattage_torque_speed_mode_control();
		power_supply_12();
		battery_series();
		wheel_setting();
		fw_sensor();
		stand_tilt_sensor();
		brake_sensor_control();
		high_current_sampling();
		five_v_protection();
		set_motor_temp_accel(app_balance_conf.kd_pt1_highpass_frequency);
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / 200;	//200
		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}/*
		 commands_printf("current 1: %.2f, current 2: %.2f, current 3: %.2f ",
		 mcpwm_foc_get_ia2(), mcpwm_foc_get_ib2(), mcpwm_foc_get_ic2());*/
		chThdSleep(sleep_time);
		timeout_reset();
	}
}

