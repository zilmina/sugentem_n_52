/*
 * mode.c
 *
 *  Created on: 2017/09/29
 *      Author: 廣明
 */

/* ==================================================================== */
/* #includeファイル														*/
/* ==================================================================== */
#include "iodefine.h"
#include "hardware_infomation_for_sugentem_n_52.h"
#include "mode.h"
#include "motor_control.h"
#include "ad_converter.h"
#include "sci.h"
#include "timer_for_sugentem_n_52.h"
#include "spi.h"
#include "adjust.h"
#include "Algorithm.h"
#include "loging.h"
#include "motor_control.h"
/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
static void MODE_mode0(short num);
static void MODE_mode1(short num);
static void MODE_mode2(short num);
static void MODE_mode3(short num);
static void MODE_mode4(short num);
static void MODE_mode5(short num);
static void MODE_mode6(short num);
static void MODE_mode7(short num);
static void MODE_mode8(short num);
static void MODE_mode9(short num);
static void MODE_mode10(short num);
static void MODE_mode11(short num);
static void MODE_mode12(short num);
static void MODE_mode13(short num);
static void MODE_mode14(short num);
static void MODE_mode15(short num);

static void countdown(void);
void set_position(void);
/* ==================================================================== */
/* mode関数																*/
/* ==================================================================== */
/* 	モード0処理：センサ調整													*/
/* ==================================================================== */
static void MODE_mode0(short num) {
	LED_numeric_lighting(0);
	SCI_Str_println("Mode0");
	change_body_angular_velocity_parameters(0, DEG, SUBSTITUTION);
	while (SW_MODE == 0) {

		SCI_Str_println(" RAW ");
		AD_Line_Difference_Print();
		SCI_Str_println(" DIST ");
		AD_Line_Distance_Print();

		delay(100);
	}
	delay(1000);
} //end MODE_mode0
/* ==================================================================== */
/*		モード1処理：　探索　足立法(150mm/s)(round trip)						*/
/* ==================================================================== */
static void MODE_mode1(short num) {
	LED_numeric_lighting(1);
	SCI_Str_println("Mode1");

	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始
		delay(1000);

		set_position();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();

		LED_numeric_lighting(4);
		request_move_mm(61.5F, 0.0F, SEARCH_150_SPEED, SEARCH_150_SPEED,
		SEARCH_150_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);
		enable_flag_logging();
		ENABLE_SLALOM_FLAG();
		// ゴールまで探索しながら走行する
		mouse_search_150mm_Adachi_Normal(GOAL_X, GOAL_Y, S_MODE);

		disable_flag_logging();

		LED_numeric_lighting(15);
		delay(1000);

		delay(500);
		SPI_Gyro_Zaxis_Offset_get();

		initialize_distance();
		ENABLE_SLALOM_FLAG();

		//　前進
		if (request_move_mm(45, 0, SEARCH_150_SPEED,
		SEARCH_150_SPEED,
		SEARCH_ACCEL, CONTROL_ON)) {
			return;
		}
		//距離を引く
		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

		// ゴールまで探索しながら走行する
		mouse_search_150mm_Adachi_Normal(0, 0, S_MODE);

		//MD_OFF
		motor_driver_disable();

		while (SW_MODE == 0)
			;
		delay(500);
	} //end EXEC
} //end MODE_mode1
/* ==================================================================== */
/*		モード2処理：　探索　足立法(150mm/s)(Only outward way)									*/
/* ==================================================================== */
static void MODE_mode2(short num) {
	LED_numeric_lighting(2);
	SCI_Str_println("Mode2");

	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		delay(1000);

		set_position();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();

		LED_numeric_lighting(4);
		request_move_mm(61.5F, 0.0F, SEARCH_150_SPEED, SEARCH_150_SPEED,
		SEARCH_150_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);
		enable_flag_logging();
		ENABLE_SLALOM_FLAG();
		// ゴールまで探索しながら走行する
		mouse_search_150mm_Adachi_Normal(GOAL_X, GOAL_Y, S_MODE);

		disable_flag_logging();

		LED_numeric_lighting(15);
		delay(1000);

		delay(500);
		SPI_Gyro_Zaxis_Offset_get();

		initialize_distance();
//		ENABLE_SLALOM_FLAG();
//
//		//　前進
//		if (request_move_mm(45, 0, SEARCH_150_SPEED,
//		SEARCH_150_SPEED,
//		SEARCH_ACCEL, CONTROL_ON)) {
//			return;
//		}
//		//距離を引く
//		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
//
//		// ゴールまで探索しながら走行する
//		mouse_search_150mm_Adachi_Normal(0, 0, S_MODE);

		//MD_OFF
		motor_driver_disable();

//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end Mode_mode2
/* ==================================================================== */
/*		モード3処理：TRY(150mm/s)											*/
/* ==================================================================== */
static void MODE_mode3(short num) {
	LED_numeric_lighting(3);
	SCI_Str_println("Mode3");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		delay(1000);

		set_position();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();

		LED_numeric_lighting(4);
		request_move_mm(18.0F, 0.0F, SEARCH_150_SPEED, SEARCH_150_SPEED,
		SEARCH_150_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);

		enable_flag_logging();

		// ゴールまで探索しながら走行する
		mouse_search_150mm_Adachi_Normal(GOAL_X, GOAL_Y, T_MODE);

		disable_flag_logging();

		//MD_OFF
		motor_driver_disable();

//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end MODE_mode3
/* ==================================================================== */
/*		モード4処理：try: turn(150mm/s), stright(500mm/s)					*/
/* ==================================================================== */
static void MODE_mode4(short num) {
	LED_numeric_lighting(4);
	SCI_Str_println("Mode4");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始
		delay(1000);

		set_position();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		enable_flag_logging();

		// 距離カウンタをリセットする
		initialize_distance();

		LED_numeric_lighting(4);
		request_move_mm(62.0F, 0.0F, TRY_TURN_150mm_SPEED, TRY_TURN_150mm_SPEED,
		TRY_TURN_150mm_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);

		ENABLE_SLALOM_FLAG();

		// ゴールまで探索しながら走行する
		mouse_try_STRAIGHT_1ST_TURN_150mm_Adachi(GOAL_X, GOAL_Y);

		//MD_OFF
		motor_driver_disable();

		disable_flag_logging();

//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end MODE_mode4
/* ==================================================================== */
/*		モード5処理：try: 300mm/s											*/
/* ==================================================================== */
static void MODE_mode5(short num) {
	LED_numeric_lighting(5);
	SCI_Str_println("Mode5");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始
		delay(1000);

		set_position();

		enable_suction();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();
		initialize_angle_pid();
		initialize_velocity_pid();
		LED_numeric_lighting(4);
		request_move_mm(61.0F, 0.0F, TRY_TURN_300mm_SPEED, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);

		ENABLE_SLALOM_FLAG();

		enable_flag_logging();
		// ゴールまで探索しながら走行する
		mouse_search_300mm_Adachi_Normal(GOAL_X, GOAL_Y, T_MODE);
		disable_flag_logging();

		disable_suction();
		//MD_OFF
		motor_driver_disable();

//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end MODE_mode5
/* ==================================================================== */
/*		モード6処理：try: turn(300mm/s), stright(600mm/s)					*/
/* ==================================================================== */
static void MODE_mode6(short num) {
	LED_numeric_lighting(6);
	SCI_Str_println("Mode6");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		set_position();

		enable_suction();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();
		initialize_angle_pid();
		initialize_velocity_pid();
		LED_numeric_lighting(4);
		request_move_mm(61.0F, 0.0F, TRY_TURN_300mm_SPEED, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);

		ENABLE_SLALOM_FLAG();

		enable_flag_logging();
		// ゴールまで探索しながら走行する
		mouse_try_STRAIGHT_2ND_TURN_300mm_Adachi(GOAL_X, GOAL_Y);
		disable_flag_logging();

		disable_suction();
		//MD_OFF
		motor_driver_disable();

		//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end MODE_mode6
/* ==================================================================== */
/*		モード7処理：try: turn(300mm/s), stright(750mm/s)					*/
/* ==================================================================== */
static void MODE_mode7(short num) {
	LED_numeric_lighting(7);
	SCI_Str_println("Mode7");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		set_position();

		enable_suction();

		countdown();

		// 探索走行前設定
		SEARCH_Para_Set(0, 0, 0);

		// 距離カウンタをリセットする
		initialize_distance();
		initialize_angle_pid();
		initialize_velocity_pid();
		LED_numeric_lighting(4);
		request_move_mm(61.0F, 0.0F, TRY_TURN_300mm_SPEED, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);

		ENABLE_SLALOM_FLAG();

		enable_flag_logging();
		// ゴールまで探索しながら走行する
		mouse_try_STRAIGHT_3RD_TURN_300mm_Adachi(GOAL_X, GOAL_Y);
		disable_flag_logging();

		disable_suction();
		//MD_OFF
		motor_driver_disable();

		//		LED_numeric_lighting(5);
		while (SW_MODE == 0)
			;
		delay(500);
	} //end if EXEC
} //end MODE_mode7
/* ==================================================================== */
/*		モード8処理：直進調整													*/
/* ==================================================================== */
static void MODE_mode8(short num) {
	LED_numeric_lighting(8);
	SCI_Str_println("Mode8");
	delay(500);
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		LED_numeric_lighting(8);
		delay(1000);
		LED_numeric_lighting(4);
		delay(1000);
		LED_numeric_lighting(2);
		delay(1000);
		LED_numeric_lighting(1);
		delay(1000);
		LED_numeric_lighting(0);

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		enable_flag_logging();
		request_move_mm(90 * 15, 0.0F, 0.0F, 150, 3000, CONTROL_ON);
		disable_flag_logging();
		LED_numeric_lighting(15); //完全に停止するための時間確保
		delay(1000);
		LED_numeric_lighting(1);

		motor_driver_disable();

	}
}

/* ==================================================================== */
/*		モード9処理：counter rotation turn L								*/
/* ==================================================================== */
static void MODE_mode9(short num) {
	LED_numeric_lighting(9);
	SCI_Str_println("Mode9");

	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SPI_Gyro_Zaxis_Offset_get();
		change_body_angular_velocity_parameters(0, DEG, SUBSTITUTION);
		change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);
		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		LED_numeric_lighting(8);
		delay(1000);
		LED_numeric_lighting(4);
		delay(1000);
		LED_numeric_lighting(2);
		delay(1000);
		LED_numeric_lighting(1);
		delay(1000);
		LED_numeric_lighting(0);

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		request_move_mm(61.5F - 45.0F, 0.0F, 0.0F, 150, 5000, CONTROL_OFF);
		initialize_distance();
		for (int i = 0; i < 20; i++) {

			// 左90度旋回
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(
			COUNTER_ROTATION_TURN, DEG, SUBSTRACTION);

		}
	}

}
/* ==================================================================== */
/*		モード10処理：150mm/s turn R										*/
/* ==================================================================== */
static void MODE_mode10(short num) {
	LED_numeric_lighting(10);
	SCI_Str_println("Mode10");
	delay(500);
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SPI_Gyro_Zaxis_Offset_get();
		change_body_angular_velocity_parameters(0.0F, DEG, SUBSTITUTION);
		change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);
		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		LED_numeric_lighting(8);
		delay(1000);
		LED_numeric_lighting(4);
		delay(1000);
		LED_numeric_lighting(2);
		delay(1000);
		LED_numeric_lighting(1);
		delay(1000);
		LED_numeric_lighting(0);

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		request_move_mm(62.5, 0.0F, 150.0F, 150, 3000, CONTROL_ON);
		initialize_distance();
		for (int i = 0; i < 33; i++) {
			LED_numeric_lighting(1);
			//R-turn

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_150_R_IN_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_150_R_90_DEG, 0.0F, 0.0F,
					-SLALOM_150_ANGULAR_VELOCITY,
					SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_150_R_90_DEG, DEG,
					ADDITION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_150_R_OUT_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(SLALOM_150_R_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			LED_numeric_lighting(0);
		}
		request_move_mm(45, 150.0F, 0.0F, 150, 3000, CONTROL_ON);

		LED_numeric_lighting(15); //完全に停止するための時間確保
		delay(1000);
		LED_numeric_lighting(1);

		motor_driver_disable();

	}
}
/* ==================================================================== */
/*		モード11処理：150mm/s turn L											*/
/* ==================================================================== */
static void MODE_mode11(short num) {
	LED_numeric_lighting(11);
	SCI_Str_println("Mode11");
	delay(500);
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SPI_Gyro_Zaxis_Offset_get();
		change_body_angular_velocity_parameters(0.0F, DEG, SUBSTITUTION);
		change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);
		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		LED_numeric_lighting(8);
		delay(1000);
		LED_numeric_lighting(4);
		delay(1000);
		LED_numeric_lighting(2);
		delay(1000);
		LED_numeric_lighting(1);
		delay(1000);
		LED_numeric_lighting(0);

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		request_move_mm(63.0F, 0.0F, 150.0F, 150, SEARCH_150_ACCEL, CONTROL_ON);
		initialize_distance();
		for (int i = 0; i < 33; i++) {
			LED_numeric_lighting(1);
			//L-turn

			// 距離カウンタリセット
			initialize_distance();
			// 壁補正PIDリセット
			initialize_led_pid();
			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_150_L_IN_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_150_L_90_DEG, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}

			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_150_L_90_DEG, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_150_L_OUT_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(SLALOM_150_L_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			// 壁補正PIDリセット
			initialize_led_pid();
			LED_numeric_lighting(0);
		}
		request_move_mm(45, 150.0F, 0.0F, 150, 3000, CONTROL_ON);

		LED_numeric_lighting(15); //完全に停止するための時間確保
		delay(1000);
		LED_numeric_lighting(1);

		motor_driver_disable();

	}
}
/* ==================================================================== */
/*		モード12処理：300mm/s turn R											*/
/* ==================================================================== */
static void MODE_mode12(short num) {
	LED_numeric_lighting(12);
	SCI_Str_println("Mode12");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SPI_Gyro_Zaxis_Offset_get();
		change_body_angular_velocity_parameters(0.0F, DEG, SUBSTITUTION);
		change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);
		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		enable_suction();
		LED_numeric_lighting(8);
		delay(1000);
		LED_numeric_lighting(4);
		delay(1000);
		LED_numeric_lighting(2);
		delay(1000);
		LED_numeric_lighting(1);
		delay(1000);
		LED_numeric_lighting(0);

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		request_move_mm(63.0F, 0.0F, SEARCH_300_SPEED, SEARCH_300_SPEED,
		SEARCH_300_ACCEL, CONTROL_ON);
		initialize_distance();
		for (int i = 0; i < 17; i++) {
			LED_numeric_lighting(1);
			//R-turn

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_300_R_IN_OFFSET, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_300_SPEED, SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			LED_numeric_lighting(8);

			if (request_slalom(SLALOM_300_R_90_DEG, 0.0F, 0.0F,
					-SLALOM_300_ANGULAR_VELOCITY,
					SLALOM_300_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_300_R_90_DEG, DEG,
					ADDITION);

			// 距離カウンタリセット
			initialize_distance();

			LED_numeric_lighting(2);

			if (request_move_mm(SLALOM_300_R_OUT_OFFSET, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_300_SPEED, SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(SLALOM_300_R_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			LED_numeric_lighting(0);
		}
		request_move_mm(45, SEARCH_300_SPEED, 0.0F, SEARCH_300_SPEED,
		SEARCH_300_ACCEL, CONTROL_ON);

		LED_numeric_lighting(15); //完全に停止するための時間確保
		delay(1000);
		LED_numeric_lighting(1);

		motor_driver_disable();
		disable_suction();

	}
}
/* ==================================================================== */
/*		モード13処理：300mm/s turn L											*/
/* ==================================================================== */
static void MODE_mode13(short num) {
	LED_numeric_lighting(13);
	SCI_Str_println("Mode13");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		SPI_Gyro_Zaxis_Offset_get();
		change_body_angular_velocity_parameters(0.0F, DEG, SUBSTITUTION);
		change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);
		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		LED_numeric_lighting(15);
		delay(1000);
		initialize_velocity_pid();
		initialize_angle_pid();
		motor_driver_enable();
		enable_suction();

		countdown();

		SCI_Value_print(SPI_Gyro_Zaxis_Offset_get(), 5); //GYRO_OFFSET再計算
		initialize_distance();
		request_move_mm(63.0F, 0.0F, SEARCH_300_SPEED, SEARCH_300_SPEED,
		SEARCH_300_ACCEL, CONTROL_ON);
		initialize_distance();
		for (int i = 0; i < 17; i++) {
			LED_numeric_lighting(1);
			//L-turn

			// 距離カウンタリセット
			initialize_distance();
			// 壁補正PIDリセット
			initialize_led_pid();
			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_300_L_IN_OFFSET, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_250_SPEED, SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

			LED_numeric_lighting(8);

			if (request_slalom(SLALOM_300_L_90_DEG, 0.0F, 0.0F,
			SLALOM_300_ANGULAR_VELOCITY,
			SLALOM_300_ANGULAR_ACCEL)) {
				return;
			}

			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_300_L_90_DEG, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			LED_numeric_lighting(2);
			if (request_move_mm(SLALOM_300_L_OUT_OFFSET, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_300_SPEED, SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(SLALOM_300_L_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			// 壁補正PIDリセット
			initialize_led_pid();
			LED_numeric_lighting(0);
		}
		request_move_mm(45, SEARCH_300_SPEED, 0.0F, SEARCH_300_SPEED,
		SEARCH_300_ACCEL, CONTROL_ON);

		LED_numeric_lighting(15); //完全に停止するための時間確保
		delay(1000);
		LED_numeric_lighting(1);

		motor_driver_disable();
		disable_suction();

	}
}
/* ==================================================================== */
/*		モード14処理：												*/
/* ==================================================================== */
static void MODE_mode14(short num) {
	LED_numeric_lighting(14);
	SCI_Str_println("Mode14");
	if (num == EXEC) {
		enable_pid_calculate(); //PID計算開始

		set_position();

		enable_suction();

		countdown();
		// 距離カウンタをリセットする
		initialize_distance();
		initialize_angle_pid();
		LED_numeric_lighting(4);
		request_move_mm(54.5F, 0.0F, SEARCH_150_SPEED, SEARCH_150_SPEED,
		SEARCH_150_ACCEL, CONTROL_ON);
		//区間中央であると定義
		change_body_velocity_parameters(0, DISTANCE, SUBSTITUTION);
		LED_numeric_lighting(0);
		for (int i = 0; i < 2; i++) {
			request_move_mm(15 * 180, SEARCH_150_SPEED, 0, 1200,
			SEARCH_150_ACCEL, CONTROL_ON);
			//R-turn

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_150_R_IN_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_150_R_90_DEG, 0.0F, 0.0F,
					-SLALOM_150_ANGULAR_VELOCITY,
					SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_150_R_90_DEG, DEG,
					ADDITION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_150_R_OUT_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(SLALOM_150_R_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			request_move_mm(7 * 180, SEARCH_150_SPEED, SEARCH_150_SPEED, 2000,
			SEARCH_150_ACCEL, CONTROL_ON);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_REFER - AD_Sensor_Distance_Get(FRONT_RIGHT)))
						/ 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(SLALOM_150_R_IN_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_150_R_90_DEG, 0.0F, 0.0F,
					-SLALOM_150_ANGULAR_VELOCITY,
					SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_150_R_90_DEG, DEG,
					ADDITION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_150_R_OUT_OFFSET, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_150_SPEED, SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(SLALOM_150_R_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();
		}

	} //end if EXEC
}
/* ==================================================================== */
/*		モード15処理：Log output												*/
/* ==================================================================== */
static void MODE_mode15(short num) {
	LED_numeric_lighting(15);
	if (num == EXEC) {
		print_log_data();
	} else {
		SCI_Str_println("Mode15");
	}

}
/* ==================================================================== */
/* change_mode関数														*/
/* ==================================================================== */
signed char MODE_Mode_Change(signed char add, char status) {

	static signed char Mode_number = 0;

	//MD_OFF
	motor_driver_disable();

//　モード数を更新
	Mode_number += add;

// モード数が最高値になった場合の処理
	if (Mode_number > MODE_MAX)
		Mode_number = 0;
// モード数が負になった場合の処理
	if (Mode_number < 0)
		Mode_number = MODE_MAX;

// モード数に対応した関数
	switch (Mode_number) {

	case 0:
		MODE_mode0(status);
		break;
	case 1:
		MODE_mode1(status);
		break;
	case 2:
		MODE_mode2(status);
		break;
	case 3:
		MODE_mode3(status);
		break;
	case 4:
		MODE_mode4(status);
		break;
	case 5:
		MODE_mode5(status);
		break;
	case 6:
		MODE_mode6(status);
		break;
	case 7:
		MODE_mode7(status);
		break;
	case 8:
		MODE_mode8(status);
		break;
	case 9:
		MODE_mode9(status);
		break;
	case 10:
		MODE_mode10(status);
		break;
	case 11:
		MODE_mode11(status);
		break;
	case 12:
		MODE_mode12(status);
		break;
	case 13:
		MODE_mode13(status);
		break;
	case 14:
		MODE_mode14(status);
		break;
	case 15:
		MODE_mode15(status);
		break;

	default:
		break;
	}

	return Mode_number;
}

void countdown(void) {
	LED_numeric_lighting(8);
	delay(500);
	LED_numeric_lighting(4);
	delay(500);
	LED_numeric_lighting(2);
	delay(500);
	LED_numeric_lighting(1);
	delay(500);
	LED_numeric_lighting(0);
}
void set_position(void) {

	LED_numeric_lighting(9);

	change_body_velocity_parameters(0.0F, REQUEST_VELOCITY, SUBSTITUTION);
	change_body_velocity_parameters(0.0F, TARGET_VELOCITY, SUBSTITUTION);

	change_body_angular_velocity_parameters(0, DEG, SUBSTITUTION);
	change_body_angular_velocity_parameters(0.0F, REQUEST_ANGULAR_VELOCITY,
			SUBSTITUTION);
	change_body_angular_velocity_parameters(0.0F, TARGET_ANGULAR_VELOCITY,
			SUBSTITUTION);

	initialize_all_pid();

	LED_numeric_lighting(6);

	duty_override(0.0F, r_motor); //motor出力を0で固定
	duty_override(0.0F, l_motor);

	motor_driver_enable();
	delay(500);
	duty_override(-0.21F, r_motor);
	duty_override(-0.25F, l_motor);
	delay(600);
	duty_override(0.0F, r_motor);
	duty_override(0.0F, l_motor);
	delay(2000);

	initialize_all_pid();
	initialize_distance();
	SPI_Gyro_Zaxis_Offset_get();
	change_body_angular_velocity_parameters(0.0F, DEG, SUBSTITUTION);

	enable_pid_calculate();
}
