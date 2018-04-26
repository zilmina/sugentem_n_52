/*
 * motor_control.c
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

/*　 =============================================================================
 *
 * インクルードファイル
 *
 * =============================================================================　*/
#include "motor_control.h"
#include "iodefine.h"
#include <math.h>
#include <stdlib.h>
#include "spi.h"
#include "adjust.h"
#include "timer_for_sugentem_n_52.h"
#include "periodic_interrupt_function.h"
#include "hardware_infomation_for_sugentem_n_52.h"
#include "ad_converter.h"
#include "loging.h"
/* ==================================================================== */
/*
 * グローバル変数
 * 																*/
/* ==================================================================== */
//機体情報
static volatile struct Machine mouse = { 0 }; //volatileは最適化防止の組み込みcのテクニック

/*==============================================================================
 *
 * グローバル関数
 *
 *============================================================================== */

void motor_initialize(void) {
	//変数設定
	mouse.r_motor.duty = 0;
	mouse.l_motor.duty = 0;

	//io_setup
	PORTA.PDR.BIT.B1 = 1;			//nSLEEP
	PORTA.PODR.BIT.B1 = 0;			//スリープ

	PORTE.PDR.BIT.B4 = 1;			//APHASE
	PORTE.PDR.BIT.B2 = 1;			//BPHASE
	R_MOTOR = R_FORWARD;
	L_MOTOR = L_FORWARD;

	PORTC.PDR.BIT.B4 = 1;			//MD_MODE
	PORTC.PODR.BIT.B4 = 1;			//1: PN/EN mode 0: IN/IN mode

	// MTU3
	SYSTEM.PRCR.WORD = 0xa50b;		// Release Protect
	MSTP(MTU3) = 0;					// Wake up MTU3
	SYSTEM.PRCR.WORD = 0xa500;		// Protect
	// Set MPC
	PORT1.PMR.BIT.B7 = 1;			// Set P17(AENBL): Peripheral
	PORT1.PMR.BIT.B6 = 1;			// Set P16(BENBL): Peripheral

	MPC.PWPR.BIT.B0WI = 0;			// protect
	MPC.PWPR.BIT.PFSWE = 1;			// Release Protect

	MPC.P16PFS.BIT.PSEL = 1;		// Set P16: MTIOC3C
	MPC.P17PFS.BIT.PSEL = 1;		// Set P17: MTIOC3A

	MPC.PWPR.BIT.PFSWE = 0;			// Protect
	MPC.PWPR.BIT.B0WI = 1;			// Release Protect

	// PWM Settings
	MTU.TOER.BIT.OE3B = 1;			// Enable MTIOC3B Output
	MTU.TOER.BIT.OE3D = 1;			// Enable MTIOC3D Output

	MTU3.TCR.BIT.TPSC = 0;			// PCLK/1 = 50MHz/1 = 50MHz
	MTU3.TCR.BIT.CKEG = 0;			// Count rising edge
	MTU3.TCR.BIT.CCLR = 1;			// TGRAのコンペアマッチでMTU3のカウントをリセット

	MTU2.TMDR.BIT.MD = 2;			// MTU3 PWMモード2
	MTU3.TMDR.BIT.MD = 2;			// MTU3 PWMモード1

	MTU3.TMDR.BIT.BFA = 0;			// TGRA, TGRC normal mode
	MTU3.TMDR.BIT.BFB = 0;			// TGRB, TGRD normal mode

	MTU3.TIORH.BIT.IOA = 6;			// Compare Output High
	MTU3.TIORH.BIT.IOB = 7;			// Compare Output Low
	MTU3.TIORL.BIT.IOC = 6;			// Compare Output High
	MTU3.TIORL.BIT.IOD = 7;			// Compare Output Low

	MTU3.TGRA = period - 1;				// 1kHz(65535)
	MTU3.TGRB = 1;						// Duty 0%
	MTU3.TGRC = period - 1;				// 1kHz(65535)
	MTU3.TGRD = 1;						// Duty 0%

	MTU3.TCNT = 0;						// Clear MTU3 count
	MTU.TSTR.BIT.CST3 = 1;				// Start MTU3 count

	change_r_motor_parameters(0, MOTOR_TARGET_VELOCITY);
	change_l_motor_parameters(0, MOTOR_TARGET_VELOCITY);

}
void motor_driver_enable(void) {
	PORTA.PODR.BIT.B1 = 1;			//スリープ解除
}
void motor_driver_disable(void) {
	PORTA.PODR.BIT.B1 = 0;			//スリープ
}

void motor_power_change(void) {	//intprog.c内で1kHz割り込み---------------------------------------------------------------------------------
	static int flag_r_motor_restart = 0;
	static int flag_l_motor_restart = 0;

	int r_duty = 0;
	int l_duty = 0;
	r_duty = period * fabs(mouse.r_motor.duty) - 1;
	l_duty = period * fabs(mouse.l_motor.duty) - 1;

	//r_motor
	if (mouse.r_motor.duty == 0) {
		MTU.TSTR.BIT.CST3 = 0;			// Stop MTU3 count
		MTU3.TIORL.BIT.IOC = 0;			// Compare Output Hi-Z
		MTU3.TIORL.BIT.IOD = 0;			// Compare Output Hi-Z
		MTU.TSTR.BIT.CST3 = 1;			// Start MTU3 count
		flag_r_motor_restart = 1;
	} else if (fabs(mouse.r_motor.duty) == 1) {
		r_duty -= 1;
	} else if (mouse.r_motor.duty > 0) {
		R_MOTOR = R_FORWARD;
	} else {
		R_MOTOR = R_BACKWARD;
	}
	if (mouse.r_motor.duty != 0) {
		if (flag_r_motor_restart == 1) {
			MTU.TSTR.BIT.CST3 = 0;			// Stop MTU3 count
			MTU3.TIORL.BIT.IOC = 6;			// Compare Output High
			MTU3.TIORL.BIT.IOD = 7;			// Compare Output Low
			MTU.TSTR.BIT.CST3 = 1;			// Start MTU3 count
			flag_r_motor_restart = 0;
		}
	}
	MTU3.TGRC = period - 1;				// 1kHz(65535)
	MTU3.TGRD = r_duty;				// r Duty

	//l_motor
	if (mouse.l_motor.duty == 0) {
		MTU.TSTR.BIT.CST3 = 0;			// Stop MTU3 count
		MTU3.TIORH.BIT.IOA = 0;			// Compare Output Hi-Z
		MTU3.TIORH.BIT.IOB = 0;			// Compare Output Hi-Z
		MTU.TSTR.BIT.CST3 = 1;			// Start MTU3 count
		flag_l_motor_restart = 1;
	} else if (fabs(mouse.l_motor.duty) == 1) {
		l_duty -= 1;
	} else if (mouse.l_motor.duty > 0) {
		L_MOTOR = L_FORWARD;
	} else {
		L_MOTOR = L_BACKWARD;
	}
	if (mouse.l_motor.duty != 0) {
		if (flag_l_motor_restart == 1) {
			MTU.TSTR.BIT.CST3 = 0;			// Stop MTU3 count
			MTU3.TIORH.BIT.IOA = 6;			// Compare Output High
			MTU3.TIORH.BIT.IOB = 7;			// Compare Output Low
			MTU.TSTR.BIT.CST3 = 1;			// Start MTU3 count
			flag_l_motor_restart = 0;
		}
	}
	MTU3.TGRA = period - 1;				// 1kHz(65535)
	MTU3.TGRB = l_duty;				// l Duty

}

void calculate_duty_with_pid(void) {

	if (mouse.flag_pid_calculate == 0) {
		return;
	}

	static float led_front_e_p = 0;
	static float led_front_e_i = 0;
	static float led_front_e_d = 0;
	static float past_led_front_e_p = 0;

	if (mouse.flag_reset_velocity_PID == 1) {
		mouse.PID.velocity_e_p = 0;
		mouse.PID.velocity_e_i = 0;
		mouse.PID.velocity_e_d = 0;
		mouse.PID.past_velocity_e_p = 0;
		mouse.flag_reset_velocity_PID = 0;
	}
	if (mouse.flag_reset_angle_PID == 1) {
		mouse.PID.angle_e_p = 0;
		mouse.PID.angle_e_i = 0;
		mouse.PID.angle_e_d = 0;
//		mouse.PID.past_angle_e_p = 0;

		mouse.flag_reset_angle_PID = 0;
	}
	if (mouse.flag_reset_led_PID == 1) {
		mouse.PID.led_side_e_p = 0;
		mouse.PID.led_side_e_i = 0;
		mouse.PID.led_side_e_d = 0;
		mouse.PID.past_led_side_e_p = 0;

		mouse.flag_reset_led_PID = 0;
	}

	//本体の角速度制御

	mouse.PID.angle_e_p = mouse.request_angular_velocity
			- mouse.now_angular_velocity;
	mouse.PID.angle_e_i += mouse.PID.angle_e_p;
	mouse.PID.angle_e_d = mouse.PID.angle_e_p - mouse.PID.past_angle_e_p;

	if (mouse.flag_side_WALL_CONTROL == 1) {
		mouse.angle_gain = ANGLE_GAIN_P * mouse.PID.angle_e_p
				+ ANGLE_GAIN_I_SIDE_WALL_ON * mouse.PID.angle_e_i
				+ ANGLE_GAIN_D * mouse.PID.angle_e_d;
	} else {
		mouse.angle_gain = ANGLE_GAIN_P * mouse.PID.angle_e_p
				+ ANGLE_GAIN_I_SIDE_WALL_OFF * mouse.PID.angle_e_i
				+ ANGLE_GAIN_D * mouse.PID.angle_e_d;
	}
//桁落ちが気になるのでここでu(利得)を/1000します
	mouse.angle_gain /= 1000;

//本体の横壁センサー制御
	if (mouse.flag_side_WALL_CONTROL == 1) {
		mouse.PID.led_side_e_p = AD_Line_Error_Get();
		mouse.PID.led_side_e_i += mouse.PID.led_side_e_p;
		mouse.PID.led_side_e_d = mouse.PID.led_side_e_p
				- mouse.PID.past_led_side_e_p;

		if (mouse.mode_side_led_P_gain == 0) {
			mouse.led_side_gain = LED_SIDE_GAIN_P_LOW_SPEED
					* mouse.PID.led_side_e_p
					+ LED_SIDE_GAIN_I * mouse.PID.led_side_e_i
					+ LED_SIDE_GAIN_D * mouse.PID.led_side_e_d;
		} else if (mouse.mode_side_led_P_gain == 1) {
			mouse.led_side_gain = LED_SIDE_GAIN_P_HIGH_SPEED
					* mouse.PID.led_side_e_p
					+ LED_SIDE_GAIN_I * mouse.PID.led_side_e_i
					+ LED_SIDE_GAIN_D * mouse.PID.led_side_e_d;
		}
		//桁落ちが気になるのでここでu(利得)を/1000します
		mouse.led_side_gain /= 1000;
	} else {
		mouse.led_side_gain = 0;
		mouse.PID.led_side_e_i = 0;
	}

//本体の前壁センサー制御
	if (mouse.flag_front_WALL_CONTROL == 1) {
		led_front_e_p = AD_front_Error_Get();
		led_front_e_i += mouse.PID.led_side_e_p;
		led_front_e_d = led_front_e_p - past_led_front_e_p;

		mouse.led_front_gain = LED_FRONT_GAIN_P * led_front_e_p
				+ LED_FRONT_GAIN_I * led_front_e_i
				+ LED_FRONT_GAIN_D * led_front_e_d;

		//桁落ちが気になるのでここでu(利得)を/1000します
		mouse.led_front_gain /= 1000;
	} else {
		mouse.led_front_gain = 0;
	}

//モーターの速度制御

//error(偏差はおおよそ整数)

	mouse.PID.velocity_e_p = mouse.request_velocity - mouse.now_velocity;
	mouse.PID.velocity_e_i += mouse.PID.velocity_e_p;
	mouse.PID.velocity_e_d = mouse.PID.velocity_e_p
			- mouse.PID.past_velocity_e_p;

	float velocity_u_motor = MOTOR_VELOCITY_GAIN_P * mouse.PID.velocity_e_p
			+ MOTOR_VELOCITY_GAIN_I * mouse.PID.velocity_e_i
			+ MOTOR_VELOCITY_GAIN_D * mouse.PID.velocity_e_d;

//桁落ちが気になるのでここでu(利得)を/1000します
	velocity_u_motor /= 1000;

	//フェイルセーフ変数の操作
	if (ABS(mouse.PID.angle_e_i) > GYRO_I_THRESHOLD) {
		mouse.flag_fail_safe = 1;
//		LED_numeric_lighting(8);
	} else if (ABS(mouse.PID.velocity_e_i) > ENCODER_I_THRESHOLD) {
		mouse.flag_fail_safe = 1;
//		LED_numeric_lighting(4);
	} else {
		mouse.flag_fail_safe = 0;

	}

//	logging(velocity_u_motor * 100);
//	logging(mouse.angle_gain * 100);
//	logging(mouse.led_side_gain * 100);

	float duty_r = velocity_u_motor / 2 + mouse.angle_gain - mouse.led_side_gain
			+ mouse.led_front_gain;

	float duty_l = velocity_u_motor / 2 - mouse.angle_gain + mouse.led_side_gain
			- mouse.led_front_gain;

//電圧保証
	duty_r = duty_r / battery_voltage() * 4200;
	duty_l = duty_l / battery_voltage() * 4200;

//duty変更
	change_duty(duty_r, r_motor);
	change_duty(duty_l, l_motor);
}
void update_velocity_parameters(void) {
//BODY直線方向速度
	if (mouse.target_velocity > mouse.request_velocity) {
		mouse.request_velocity += mouse.target_accel / 1000;
		if (mouse.target_velocity < mouse.request_velocity) {
			mouse.request_velocity = mouse.target_velocity;
		}
	} else if (mouse.target_velocity < mouse.request_velocity) {
		mouse.request_velocity -= (mouse.target_accel / 1000);
		if (mouse.target_velocity > mouse.request_velocity) {
			mouse.request_velocity = mouse.target_velocity;
		}
	}

}

void update_angular_velocity_parameters(void) {
//BODY回転方向速度
	if (mouse.target_angular_velocity > mouse.request_angular_velocity) {
		if ((mouse.request_angular_velocity + mouse.target_angular_accel / 1000)
				< mouse.target_angular_velocity) {
			mouse.request_angular_velocity += mouse.target_angular_accel / 1000;
		} else {
			mouse.request_angular_velocity = mouse.target_angular_velocity;
		}
	} else if (mouse.target_angular_velocity < mouse.request_angular_velocity) {
		if ((mouse.request_angular_velocity
				- (mouse.target_angular_accel / 1000))
				> mouse.target_angular_velocity) {
			mouse.request_angular_velocity -=
					(mouse.target_angular_accel / 1000);
		} else {
			mouse.request_angular_velocity = mouse.target_angular_velocity;
		}
	}

}

void duty_override(float temp_duty, enum Motor temp) {
	//操作後enable_pid_calculate();を実行して復帰。
	//pidゲインリセットが必要
	disable_pid_calculate();
	change_duty(temp_duty, temp);
}

void change_duty(float temp_duty, enum Motor temp) {//外部からモーターのduty比を変更します-------------------------------------

	if (temp_duty > 1) {
		temp_duty = 1;
	} else if (temp_duty < -1) {
		temp_duty = -1;
	}

	if (temp == r_motor) {
		mouse.r_motor.duty = temp_duty;
	} else if (temp == l_motor) {
		mouse.l_motor.duty = temp_duty;
	}
}

void update_gyro(void) {
	static double past_dps_raw = 0;
	short dps_raw = 0;

	dps_raw = SPI_Gyro_Zaxis_get() - mouse.gyro.offset_raw;
//角速度に対する返り値の傾斜補正
	if (dps_raw >= 0) {
		dps_raw *= Z_GYRO_GAIN_FORWARD;
	} else {
		dps_raw *= Z_GYRO_GAIN_BACKWARD;
	}

	mouse.now_angular_velocity = dps_raw / 16.4F;
	mouse.deg += (0.5F) * (dps_raw + past_dps_raw) * 0.001F / 16.4F;
	past_dps_raw = dps_raw;

}

void update_encoder(void) {
///右エンコーダの更新
	int temp_encr_raw = 0;
	const short r_count = 1;
	volatile static long encr_past_raw = 0;
	for (int i = 0; i < r_count; i++) {
		temp_encr_raw += SPI_Encoder_R_Angle_get();
		TIMER_Wait_for(200);
	}
	temp_encr_raw /= r_count;	//エンコーダのraw値を取得
	mouse.encr.raw = temp_encr_raw;

	if ((temp_encr_raw - encr_past_raw) > 8000) {
		encr_past_raw += 16384;
	} else if ((temp_encr_raw - encr_past_raw) < -8000) {
		encr_past_raw -= 16384;
	}
	mouse.encr.raw_angular_velocity = temp_encr_raw - encr_past_raw;
	mouse.encr.raw_integral += mouse.encr.raw_angular_velocity;
	encr_past_raw = mouse.encr.raw;

///左エンコーダの更新
	int temp_encl_raw = 0;
	const short l_count = 1;
	volatile static long encl_past_raw = 0;
	for (int i = 0; i < l_count; i++) {
		temp_encl_raw += SPI_Encoder_L_Angle_get();
		TIMER_Wait_for(200);
	}
	temp_encl_raw /= l_count;	//エンコーダのraw値を取得
	mouse.encl.raw = temp_encl_raw;

	if ((temp_encl_raw - encl_past_raw) > 8000) {
		encl_past_raw += 16384;
	} else if ((temp_encl_raw - encl_past_raw) < -8000) {
		encl_past_raw -= 16384;
	}
	mouse.encl.raw_angular_velocity = temp_encl_raw - encl_past_raw;
	mouse.encl.raw_integral += mouse.encl.raw_angular_velocity;
	encl_past_raw = mouse.encl.raw;
	update_motor_parameters();
}

void update_motor_parameters(void) {

	mouse.r_motor.deg = (float) mouse.encr.raw_integral / 45.511111111F;
	mouse.l_motor.deg = (float) -mouse.encl.raw_integral / 45.511111111F;

	mouse.r_motor.angular_velocity = (float) mouse.encr.raw_angular_velocity
			/ 45.5111111111F * 1000;
	mouse.l_motor.angular_velocity = (float) -mouse.encl.raw_angular_velocity
			/ 45.5111111111F * 1000;

	mouse.r_motor.velocity = (float) mouse.r_motor.angular_velocity
			/ 360* 3.1459265F * TIRE_RADIUS;
	mouse.l_motor.velocity = (float) mouse.l_motor.angular_velocity
			/ 360* 3.1459265F * TIRE_RADIUS;

	mouse.r_motor.distance = (float) mouse.r_motor.deg * 3.14159265F
			* TIRE_RADIUS / 360;
	mouse.l_motor.distance = (float) mouse.l_motor.deg * 3.14159265F
			* TIRE_RADIUS / 360;

	mouse.now_velocity = (mouse.r_motor.velocity + mouse.l_motor.velocity) / 2;
	mouse.distance_mm += (mouse.now_velocity / 1000);

}

/********************************************:
 * MPU6500からZ軸ジャイロの値を取得
 * @return
 */
int SPI_Gyro_Zaxis_Offset_get(void) {
	short count = 100;
	int temp_offset = 0;
	disable_interrupt();
	delay(4);
	for (int i = 0; i < count; i++) {
		temp_offset += SPI_Gyro_Zaxis_get();
		delay(1);
	}
	temp_offset /= count;

	mouse.gyro.offset_raw = temp_offset;
	enable_interrupt();
	return mouse.gyro.offset_raw;
}

float check_body_velocity_parameters(enum Body_Velocity_St temp) {
	switch (temp) {
	case DISTANCE:
		return mouse.distance_mm;
		break;
	case NOW_VELOCITY:
		return mouse.now_velocity;
		break;
	case REQUEST_VELOCITY:
		return mouse.request_velocity;
		break;
	case TARGET_VELOCITY:
		return mouse.target_velocity;
		break;

	case TARGET_ACCEL:				//現在software上ではこれが加速度
		return mouse.target_accel;
		break;
	default:
		return 1;
		break;
	}
}
float check_body_angular_velocity_parameters(enum Body_Angular_Velocity_St temp) {
	switch (temp) {
	case ANGLE_GAIN:
		return mouse.angle_gain;
		break;
	case DEG:
		return mouse.deg;
		break;
	case TARGET_DEG:
		return mouse.target_deg;
		break;
	case NOW_ANGULAR_VELOCITY:
		return mouse.now_angular_velocity;
		break;
	case REQUEST_ANGULAR_VELOCITY:
		return mouse.request_angular_velocity;
		break;
	case TARGET_ANGULAR_VELOCITY:
		return mouse.target_angular_velocity;
		break;
	case NOW_ANGULAR_ACCEL:
		return mouse.now_angular_accel;
		break;
	case TARGET_ANGULAR_ACCEL:
		return mouse.target_angular_accel;
		break;
	default:
		return 1;
		break;
	}
}

float check_r_motor_parameters(enum Motor_St temp) {
	switch (temp) {
	case DUTY:
		return mouse.r_motor.duty;
		break;
	case MOTOR_DEG:
		return mouse.r_motor.deg;
		break;
	case MOTOR_ANGULAR_VELOCITY:
		return mouse.r_motor.angular_velocity;
		break;
	case MOTOR_DISTANCE:
		return mouse.r_motor.distance;
		break;
	case MOTOR_VELOCITY:
		return mouse.r_motor.velocity;
		break;
	case MOTOR_REQUEST_VELOCITY:
		return mouse.r_motor.request_velocity;
		break;
	case MOTOR_TARGET_VELOCITY:
		return mouse.r_motor.target_velocity;
		break;
	default:
		return 1;
		break;
	}
}
float check_r_encoder_parameters(enum Encoder_St temp) {
	switch (temp) {
	case VALUE:
		return mouse.encr.value;
		break;
	case RAW:
		return mouse.encr.raw;
		break;
	default:
		return 1;
		break;
	}
}
float check_l_motor_parameters(enum Motor_St temp) {
	switch (temp) {
	case DUTY:
		return mouse.l_motor.duty;
		break;
	case MOTOR_DEG:
		return mouse.l_motor.deg;
		break;
	case MOTOR_ANGULAR_VELOCITY:
		return mouse.l_motor.angular_velocity;
		break;
	case MOTOR_DISTANCE:
		return mouse.l_motor.distance;
		break;
	case MOTOR_VELOCITY:
		return mouse.l_motor.velocity;
		break;
	case MOTOR_REQUEST_VELOCITY:
		return mouse.l_motor.request_velocity;
		break;

	case MOTOR_TARGET_VELOCITY:
		return mouse.l_motor.target_velocity;
		break;
	default:
		return 1;
		break;
	}
}
float check_l_encoder_parameters(enum Encoder_St temp) {
	switch (temp) {
	case VALUE:
		return mouse.encl.value;
		break;
	case RAW:
		return mouse.encl.raw;
		break;
	default:
		return 1;
		break;
	}
}
void change_body_velocity_parameters(float temp_num,
		enum Body_Velocity_St temp_mode,
		enum Numetric_Operator_St temp_operator) {
	switch (temp_mode) {
	case DISTANCE:
		switch (temp_operator) {
		case SUBSTITUTION:
			mouse.distance_mm = temp_num;
			break;
		case ADDITION:
			mouse.distance_mm += temp_num;
			break;
		case SUBSTRACTION:
			mouse.distance_mm -= temp_num;
			break;
		case MULTIPLICATION:
			mouse.distance_mm *= temp_num;
			break;
		case DIVITION:
			mouse.distance_mm /= temp_num;
			break;
		}
		break;
	case NOW_VELOCITY:
		mouse.now_velocity = temp_num;
		break;
	case REQUEST_VELOCITY:
		mouse.request_velocity = temp_num;
		break;
	case TARGET_VELOCITY:
		mouse.target_velocity = temp_num;
		break;

	case TARGET_ACCEL:				//現在software上ではこれが加速度
		mouse.target_accel = temp_num;
		break;
	default:
		break;
	}
}

void change_body_angular_velocity_parameters(float temp_num,
		enum Body_Angular_Velocity_St temp_mode,
		enum Numetric_Operator_St temp_operator) {
	switch (temp_mode) {
	case DEG:
		switch (temp_operator) {
		case SUBSTITUTION:
			mouse.deg = temp_num;
			break;
		case ADDITION:
			mouse.deg += temp_num;
			break;
		case SUBSTRACTION:
			mouse.deg -= temp_num;
			break;
		case MULTIPLICATION:
			mouse.deg *= temp_num;
			break;
		case DIVITION:
			mouse.deg /= temp_num;
			break;
		}
		break;
	case TARGET_DEG:
		mouse.target_deg = temp_num;
		break;
	case NOW_ANGULAR_VELOCITY:
		mouse.now_angular_velocity = temp_num;
		break;
	case REQUEST_ANGULAR_VELOCITY:
		mouse.request_angular_velocity = temp_num;
		break;
	case TARGET_ANGULAR_VELOCITY:
		switch (temp_operator) {
		case SUBSTITUTION:
			mouse.target_angular_velocity = temp_num;
			break;
		case ADDITION:
			mouse.target_angular_velocity += temp_num;
			break;
		case SUBSTRACTION:
			mouse.target_angular_velocity -= temp_num;
			break;
		case MULTIPLICATION:
			mouse.target_angular_velocity *= temp_num;
			break;
		case DIVITION:
			mouse.target_angular_velocity /= temp_num;
			break;
		}
		break;
	case NOW_ANGULAR_ACCEL:
		mouse.now_angular_accel = temp_num;
		break;
	case TARGET_ANGULAR_ACCEL:
		mouse.target_angular_accel = temp_num;
		break;
	default:
		break;
	}
}

void change_r_motor_parameters(float temp_num, enum Motor_St temp_mode) {
	switch (temp_mode) {
	case DUTY:
		mouse.r_motor.duty = temp_num;
		break;
	case MOTOR_DEG:
		mouse.r_motor.deg = temp_num;
		break;
	case MOTOR_ANGULAR_VELOCITY:
		mouse.r_motor.angular_velocity = temp_num;
		break;
	case MOTOR_DISTANCE:
		mouse.r_motor.distance = temp_num;
		break;
	case MOTOR_VELOCITY:
		mouse.r_motor.velocity = temp_num;
		break;
	case MOTOR_TARGET_VELOCITY:
		mouse.r_motor.target_velocity = temp_num;
		break;
	default:
		break;
	}
}
void change_l_motor_parameters(float temp_num, enum Motor_St temp_mode) {
	switch (temp_mode) {
	case DUTY:
		mouse.l_motor.duty = temp_num;
		break;
	case MOTOR_DEG:
		mouse.l_motor.deg = temp_num;
		break;
	case MOTOR_ANGULAR_VELOCITY:
		mouse.l_motor.angular_velocity = temp_num;
		break;
	case MOTOR_DISTANCE:
		mouse.l_motor.distance = temp_num;
		break;
	case MOTOR_VELOCITY:
		mouse.l_motor.velocity = temp_num;
		break;
	case MOTOR_TARGET_VELOCITY:
		mouse.l_motor.target_velocity = temp_num;
		break;
	default:
		break;
	}
}

void initialize_distance(void) {
	mouse.distance_mm = 0;
	mouse.r_motor.distance = 0;
	mouse.l_motor.distance = 0;
	mouse.encr.raw_integral = 0;
	mouse.encl.raw_integral = 0;
//	mouse.encr.raw_angular_velocity = 0;
//	mouse.encl.raw_angular_velocity = 0;
}

void initialize_all_pid(void) {
	mouse.flag_reset_led_PID = 1;
	mouse.flag_reset_angle_PID = 1;
	mouse.flag_reset_velocity_PID = 1;
}

void initialize_led_pid(void) {
	mouse.flag_reset_led_PID = 1;
}
void initialize_angle_pid(void) {
	mouse.flag_reset_angle_PID = 1;
}
void initialize_velocity_pid(void) {
	mouse.flag_reset_velocity_PID = 1;
}

/*********************************************************************
 * 直進モジュール (区間、最終速度)	台形加減速
 *
 * @param Block_num		:前進する区画数
 * @param Last_speed	:現在の速度
 * @param Max_speed		:最高速度
 * @param Control_flag	0:横壁制御なし　1:横壁制御あり
 */
char request_move_mm(long temp_mm, float start_speed, float last_speed,
		float max_speed, float accel, enum Run_Control control) {

	long distance_down = 0;

// 横壁制御の決定
	SET_side_WALL_CONTROL(control);
// 距離カウンタをリセットする
//	initialize_distance();
//PID制御の初期化
	initialize_velocity_pid();

	change_body_angular_velocity_parameters(0.0F, TARGET_ANGULAR_VELOCITY,
			SUBSTITUTION);

//加速度
	change_body_velocity_parameters(accel, TARGET_ACCEL, SUBSTITUTION);

//現在の速度設定
	change_body_velocity_parameters(start_speed, REQUEST_VELOCITY,
			SUBSTITUTION);
// 目標速度の設定
	if (temp_mm > check_body_velocity_parameters(DISTANCE)) {		//正の方向に進む
		change_body_velocity_parameters(max_speed, TARGET_VELOCITY,
				SUBSTITUTION);
	} else {				//負の方向に進む
		change_body_velocity_parameters(-max_speed, TARGET_VELOCITY,
				SUBSTITUTION);
	}

//	 加速＋定速
	while (1) {
		// 減速区間の計算
		distance_down =
				(long) ((mouse.request_velocity * mouse.request_velocity)
						- (last_speed * last_speed)) / (2 * mouse.target_accel);

		// 設定距離を超えたかどうかの判定
		if (CONTROL_Distance_Watch(temp_mm - distance_down, control) == FINISH
				|| mouse.flag_fail_safe == 1) {
			break;
		}
	}

//　最低目標速度の設定
	change_body_velocity_parameters(last_speed, TARGET_VELOCITY, SUBSTITUTION);
//	減速
	while (1) {
		if (CONTROL_Distance_Watch(temp_mm, control) == FINISH
				|| mouse.flag_fail_safe == 1) {
			break;
//			LED_numeric_lighting(8);
		}
	}

//SCI_Value_Println(Mouse.speed_now, 5);
// 距離カウンタをリセットする
//横壁制御OFF
	SET_side_WALL_CONTROL(CONTROL_OFF);
//異常終了か否か
	if (mouse.flag_fail_safe == 1) {
		return 1;
	} else {
		return 0;
	}
}

/*********************************************************************
 * スラロームモジュール (区間、最終速度)	台形加減速
 *
 * @param Block_num		:前進する区画数
 * @param Last_speed	:現在の速度
 * @param Max_speed		:最高速度
 * @param Control_flag	0:横壁制御なし　1:横壁制御あり
 */
char request_slalom(long temp_deg, float start_angular_speed,
		float last_angular_speed, float max_angular_velocity,
		float angular_accel) {

	long degree_down = 0;

// 横壁制御の無効化
	SET_side_WALL_CONTROL(CONTROL_OFF);

// 角度カウンタをリセットする
//	change_body_angular_velocity_parameters(0.0F, DEG);
//PID制御の初期化
	initialize_angle_pid();

//角加速度
	change_body_angular_velocity_parameters(angular_accel, TARGET_ANGULAR_ACCEL,
			SUBSTITUTION);

// 目標速度の設定
	change_body_angular_velocity_parameters(max_angular_velocity,
			TARGET_ANGULAR_VELOCITY, SUBSTITUTION);

//	 加速＋定速
	while (1) {
//		LED_numeric_lighting(2);
		// 減速区間の計算
		degree_down = (long) ((mouse.request_angular_velocity
				* mouse.request_angular_velocity)
				- (last_angular_speed * last_angular_speed))
				/ (2 * mouse.target_angular_accel);

		// 設定距離を超えたかどうかの判定
		if (CONTROL_Degree_Watch(ABS(temp_deg) - degree_down, CONTROL_OFF)
				== FINISH || mouse.flag_fail_safe == 1) {
			break;
		}
	}

//　  最低目標速度の設定
	change_body_angular_velocity_parameters(last_angular_speed,
			TARGET_ANGULAR_VELOCITY, SUBSTITUTION);
//	LED_numeric_lighting(4);
//	減速
	while (1) {
		if (mouse.request_angular_velocity == mouse.target_angular_velocity
				|| mouse.flag_fail_safe == 1) {
			break;
//			LED_numeric_lighting(8);
		}
	}
//　  要求角速度の設定
	change_body_angular_velocity_parameters(last_angular_speed,
			REQUEST_ANGULAR_VELOCITY, SUBSTITUTION);

//SCI_Value_Println(Mouse.speed_now, 5);

//横壁制御OFF
	SET_side_WALL_CONTROL(CONTROL_OFF);
//異常終了か否か
	if (mouse.flag_fail_safe == 1) {
		return 1;
	} else {
		return 0;
	}

}

enum Run_St CONTROL_Distance_Watch(volatile long distance,
		enum Run_Control control) {

	enum Run_St ans = RUNNING;

// 横壁制御のON or OFF
	SET_side_WALL_CONTROL(control);

// 距離を監視　設定した距離を超えるとFINISHを代入する
	if (mouse.distance_mm > distance)
		ans = FINISH;

	return ans;
}
enum Run_St CONTROL_Degree_Watch(volatile long temp_deg,
		enum Run_Control control) {

	enum Run_St ans = RUNNING;

// 横壁制御のON or OFF
	SET_side_WALL_CONTROL(CONTROL_OFF);

// 距離を監視　設定した距離を超えるとFINISHを代入する
	if (fabs(mouse.deg) > fabs(temp_deg))
		ans = FINISH;

	return ans;
}

void SET_side_WALL_CONTROL(enum Run_Control temp) {
	switch (temp) {
	case CONTROL_OFF:
		mouse.flag_side_WALL_CONTROL = 0;
		break;
	case CONTROL_ON:
		mouse.flag_side_WALL_CONTROL = 1;
		break;

	}
}
void SET_front_WALL_CONTROL(enum Run_Control temp) {
	switch (temp) {
	case CONTROL_OFF:
		mouse.flag_front_WALL_CONTROL = 0;
		break;
	case CONTROL_ON:
		mouse.flag_front_WALL_CONTROL = 1;
		break;

	}
}

float check_angular_velocity_PID(enum Gain temp) {
	switch (temp) {
	case P_GAIN:
		return mouse.PID.angle_e_p;
	case I_GAIN:
		return mouse.PID.angle_e_i;
	case D_GAIN:
		return mouse.PID.angle_e_d;
	}
}
float check_velocity_PID(enum Gain temp) {
	switch (temp) {
	case P_GAIN:
		return mouse.PID.velocity_e_p;
	case I_GAIN:
		return mouse.PID.velocity_e_i;
	case D_GAIN:
		return mouse.PID.velocity_e_d;
	}
}
float check_side_led_PID(enum Gain temp) {
	switch (temp) {
	case P_GAIN:
		return mouse.PID.led_side_e_p;
	case I_GAIN:
		return mouse.PID.led_side_e_i;
	case D_GAIN:
		return mouse.PID.led_side_e_d;
	}
}

void enable_suction(void) {
	SUCTION = 1;
}
void disable_suction(void) {
	SUCTION = 0;
}
void enable_pid_calculate(void) {
	mouse.flag_pid_calculate = 1;
}
void disable_pid_calculate(void) {
	mouse.flag_pid_calculate = 0;
}

void change_P_gain_mode(char temp){
	mouse.mode_side_led_P_gain = temp;
}
