/*
 * Algorithm.c
 *
 *  Created on: 2017/10/19
 *      Author: 廣明
 */

/* ==================================================================== */
/* #includeファイル															*/
/* ==================================================================== */
#include "iodefine.h"
#include "hardware_infomation_for_sugentem_n_52.h"
#include "ad_converter.h"
#include "motor_control.h"
#include "adjust.h"
#include "spi.h"
#include "timer_for_sugentem_n_52.h"
#include "algorithm.h"
#include "sci.h"

/*　 =============================================================================
 * プロトタイプ宣言
 * =============================================================================　*/

static void make_map_data(void);
static unsigned char get_wall_data(void);
/* ==================================================================== */
/* グローバル変数															*/
/* ==================================================================== */
// 迷路
static char Maze_Map[NUM_BLOCK][NUM_BLOCK] = { 0 };

// ステップ
//static struct Step Step;

// マウス座標
static struct Maze Maze;

/* 等高線 データ */
short smap[NUM_BLOCK][NUM_BLOCK];

//スラローム走行後であることを示すフラグ
static char slalom_flag = 0; //0:通常,1:スラローム走行後

/* ====================================================================	*/
/* 関数																	*/
/* ====================================================================	*/
/**********************************************************************
 * 探索走行開始前セット関数
 * @param Mx	:初期迷路ｘ座標
 * @param My	:初期迷路ｙ座標
 * @param Head	:初期進行方向
 */
void SEARCH_Para_Set(char mx, char my, char head) {

//	Step.straight = STEP_GO;
//	Step.turn = STEP_TURN;
	Maze.mx = mx;
	Maze.my = my;
	Maze.head = head;

}

///######################################################################

//		探索（足立法）150mm

///######################################################################
void mouse_search_150mm_Adachi_Normal(int tx, int ty, int mode) {

	short temp0;

	short s0, s1;
	//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	char flag_back_wall = 0;

	change_body_velocity_parameters(SEARCH_150_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

	// 壁補正PIDリセット
	initialize_led_pid();

	while (1) {
		change_body_velocity_parameters(SEARCH_150_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);
		// ---- 直進開始 ----------------------------------------------

		if (slalom_flag == 0) {

			LED_numeric_lighting(0);
			//　区画中央へ前進
			if (request_move_mm(45, SEARCH_150_SPEED, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
		}

		// 座標更新
		if (Maze.head == 0)				//北
			Maze.my += 1;
		else if (Maze.head == 1)		//東
			Maze.mx += 1;
		else if (Maze.head == 2)		//南
			Maze.my -= 1;
		else if (Maze.head == 3)		//西
			Maze.mx -= 1;

		make_smap(tx, ty, mode); /* 等高線マップを作る			*/

		// マップ情報の更新
		make_map_data();

		/* 壁データから迷路データを作る	*/

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
		/* (既探索区間,旋回）の順で選択する								*/
		temp0 = Maze_Map[Maze.my][Maze.mx];
		s0 = 1024;
		if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
			s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 0) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_FORWARD;
			}
		}
		if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 1) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_RIGHT;
			}
		}
		if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
			s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 2) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_BACK;
			}
		}
		if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 3) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_LEFT;
			}
		}
//		decide_direction(&mouse_head0, &s0, &s1);		//置き換え

		mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {
			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);

			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {		//後ろ壁当て判定
				flag_back_wall = 1;
			} else {
				flag_back_wall = 0;
			}		//end後ろ壁判定

			// 壁補正PIDリセット
			initialize_led_pid();
			//　半区画前進
			if (request_move_mm(45, SEARCH_150_SPEED, 0.0F, SEARCH_150_SPEED,
			SEARCH_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (flag_back_wall == 1) {		//後ろ壁あり
				duty_override(-0.21F, r_motor);
				duty_override(-0.25F, l_motor);
				delay(500);
				duty_override(0.0F, r_motor);
				duty_override(0.0F, l_motor);
				delay(100);

				initialize_all_pid();
				initialize_distance();
				change_body_angular_velocity_parameters(0.0F, DEG,
						SUBSTITUTION);

				enable_pid_calculate();

				//　区画中心に前進
				if (request_move_mm(62.5F - 45.0F, 0.0F, 0.0F,
				SEARCH_150_SPEED,
				SEARCH_ACCEL, CONTROL_ON)) {
					return;
				}
				//距離を引く
				change_body_velocity_parameters(62.5F - 45.0F, DISTANCE,
						SUBSTRACTION);

			}		//end後ろ壁当て動作
			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

			LED_numeric_lighting(6);

			//　半区画前進
			if (request_move_mm(45, SEARCH_150_SPEED, SEARCH_150_SPEED,
			SEARCH_150_SPEED,
			SEARCH_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			slalom_flag = 0;
			LED_numeric_lighting(0);

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
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

			}

			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {		//後ろ壁当て判定
				flag_back_wall = 1;
			} else {
				flag_back_wall = 0;
			}		//end後ろ壁判定

			//１区画まで前進
			if (request_move_mm(45, SEARCH_150_SPEED, 0.0F, SEARCH_150_SPEED,
			SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
			}

			if (request_move_mm(0, 0.0F, 0.0F, SEARCH_150_SPEED,
			SEARCH_150_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

//			SPI_Gyro_Zaxis_Offset_get();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (flag_back_wall == 1) {		//後ろ壁あり
				duty_override(-0.21F, r_motor);
				duty_override(-0.25F, l_motor);
				delay(500);
				duty_override(0.0F, r_motor);
				duty_override(0.0F, l_motor);
				delay(100);

				initialize_all_pid();
				initialize_distance();
				change_body_angular_velocity_parameters(0.0F, DEG,
						SUBSTITUTION);

				enable_pid_calculate();

				//　区画中心に前進
				if (request_move_mm(62.5F, 0.0F, 0.0F,
				SEARCH_150_SPEED,
				SEARCH_ACCEL, CONTROL_ON)) {
					return;
				}
				//距離を引く
				change_body_velocity_parameters(62.5F, DISTANCE, SUBSTRACTION);

			} /*end後ろ壁当て動作*/else {

				if (request_move_mm(45, SEARCH_150_SPEED, 0.0F,
				SEARCH_150_SPEED,
				SEARCH_150_ACCEL, CONTROL_ON)) {
					return;
				}
				//距離を引く
				change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
			}
			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}
///######################################################################

//		探索（足立法）250mm

///######################################################################
void mouse_search_250mm_Adachi_Normal(int tx, int ty, int mode) {

	short temp0;

	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(SEARCH_250_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

// 壁補正PIDリセット
	initialize_led_pid();

	while (1) {
		change_body_velocity_parameters(SEARCH_250_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);
		// ---- 直進開始 ----------------------------------------------

		if (slalom_flag == 0) {

			LED_numeric_lighting(0);
			//　区画中央へ前進
			if (request_move_mm(45, SEARCH_250_SPEED, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
		}

		// 座標更新
		if (Maze.head == 0)				//北
			Maze.my += 1;
		else if (Maze.head == 1)		//東
			Maze.mx += 1;
		else if (Maze.head == 2)		//南
			Maze.my -= 1;
		else if (Maze.head == 3)		//西
			Maze.mx -= 1;

		make_smap(tx, ty, mode); /* 等高線マップを作る			*/

		// マップ情報の更新
		make_map_data();

		/* 壁データから迷路データを作る	*/

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
		/* (既探索区間,旋回）の順で選択する								*/
		temp0 = Maze_Map[Maze.my][Maze.mx];
		s0 = 1024;
		if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
			s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 0) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_FORWARD;
			}
		}
		if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 1) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_RIGHT;
			}
		}
		if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
			s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 2) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_BACK;
			}
		}
		if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 3) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_LEFT;
			}
		}

		mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {
			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);
			// 壁補正PIDリセット
			initialize_led_pid();
			//　半区画前進
			if (request_move_mm(45, SEARCH_250_SPEED, 0.0F, SEARCH_250_SPEED,
			SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

			LED_numeric_lighting(6);

			//　半区画前進
			if (request_move_mm(45, SEARCH_250_SPEED, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			slalom_flag = 0;
			LED_numeric_lighting(0);

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

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

			if (request_move_mm(SLALOM_250_R_IN_OFFSET, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_250_SPEED, SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_250_R_90_DEG, 0.0F, 0.0F,
					-SLALOM_250_ANGULAR_VELOCITY,
					SLALOM_250_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_250_R_90_DEG, DEG,
					ADDITION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_250_R_OUT_OFFSET, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_250_SPEED, SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(SLALOM_250_R_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			LED_numeric_lighting(0);
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

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

			if (request_move_mm(SLALOM_250_L_IN_OFFSET, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_250_SPEED, SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_slalom(SLALOM_250_L_90_DEG, 0.0F, 0.0F,
			SLALOM_250_ANGULAR_VELOCITY,
			SLALOM_250_ANGULAR_ACCEL)) {
				return;
			}

			//角度を初期化
			change_body_angular_velocity_parameters(SLALOM_250_L_90_DEG, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_250_L_OUT_OFFSET, SEARCH_250_SPEED,
			SEARCH_250_SPEED,
			SEARCH_250_SPEED, SEARCH_250_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(SLALOM_250_L_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			// 壁補正PIDリセット
			initialize_led_pid();
			LED_numeric_lighting(0);
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
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

			//１区画まで前進
			if (request_move_mm(45, SEARCH_250_SPEED, 0.0F, SEARCH_250_SPEED,
			SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(0, 0.0F, 0.0F, SEARCH_150_SPEED,
			SEARCH_150_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

//			SPI_Gyro_Zaxis_Offset_get();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_move_mm(45, SEARCH_150_SPEED, 0.0F, SEARCH_150_SPEED,
			SEARCH_150_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}
///######################################################################

//		探索（足立法）300mm

///######################################################################
void mouse_search_300mm_Adachi_Normal(int tx, int ty, int mode) {

	short temp0;

	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(SEARCH_300_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

// 壁補正PIDリセット
	initialize_led_pid();

	while (1) {
		change_body_velocity_parameters(SEARCH_300_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);
		// ---- 直進開始 ----------------------------------------------

		if (slalom_flag == 0) {

			initialize_angle_pid();

			LED_numeric_lighting(0);
			//　区画中央へ前進
			if (request_move_mm(45, SEARCH_300_SPEED, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
		}

		// 座標更新
		if (Maze.head == 0)				//北
			Maze.my += 1;
		else if (Maze.head == 1)		//東
			Maze.mx += 1;
		else if (Maze.head == 2)		//南
			Maze.my -= 1;
		else if (Maze.head == 3)		//西
			Maze.mx -= 1;

		make_smap(tx, ty, mode); /* 等高線マップを作る			*/

		// マップ情報の更新
		make_map_data();

		/* 壁データから迷路データを作る	*/

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
		/* (既探索区間,旋回）の順で選択する								*/
		temp0 = Maze_Map[Maze.my][Maze.mx];
		s0 = 1024;
		if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
			s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 0) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_FORWARD;
			}
		}
		if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 1) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_RIGHT;
			}
		}
		if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
			s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 2) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_BACK;
			}
		}
		if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 3) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_LEFT;
			}
		}

		mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {
			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);
			// 壁補正PIDリセット
			initialize_led_pid();
			//　半区画前進
			if (request_move_mm(45, SEARCH_300_SPEED, 0.0F, SEARCH_300_SPEED,
			SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

			LED_numeric_lighting(6);

			//　半区画前進
			if (request_move_mm(45, SEARCH_300_SPEED, SEARCH_300_SPEED,
			SEARCH_300_SPEED,
			SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			slalom_flag = 0;
			LED_numeric_lighting(0);

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

			initialize_angle_pid();

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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

			LED_numeric_lighting(1);

			initialize_angle_pid();

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
			SEARCH_300_SPEED, SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

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

			//１区画まで前進
			if (request_move_mm(45, SEARCH_300_SPEED, 0.0F, SEARCH_300_SPEED,
			SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(0, 0.0F, 0.0F, SEARCH_150_SPEED,
			SEARCH_150_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

			//			SPI_Gyro_Zaxis_Offset_get();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_move_mm(45, SEARCH_300_SPEED, 0.0F, SEARCH_300_SPEED,
			SEARCH_300_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}

///######################################################################

//		Try（足立法）STRAIGHT:500mm, TURN:150mm

///######################################################################
void mouse_try_STRAIGHT_1ST_TURN_150mm_Adachi(int tx, int ty) {

	short temp0;

	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(TRY_TURN_150mm_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

	make_smap(tx, ty, T_MODE); /* 等高線マップを作る			*/

	while (1) {
		change_body_velocity_parameters(TRY_TURN_150mm_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);

		//前もって次の区間の移動方向を確定

		// 座標更新
		if (Maze.head == 0)				//北
			Maze.my += 1;
		else if (Maze.head == 1)		//東
			Maze.mx += 1;
		else if (Maze.head == 2)		//南
			Maze.my -= 1;
		else if (Maze.head == 3)		//西
			Maze.mx -= 1;

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
		/* (既探索区間,旋回）の順で選択する								*/
		temp0 = Maze_Map[Maze.my][Maze.mx];
		s0 = 1024;
		if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
			s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 0) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_FORWARD;
			}
		}
		if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 1) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_RIGHT;
			}
		}
		if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
			s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 2) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_BACK;
			}
		}
		if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 3) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_LEFT;
			}
		}

		mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

		// ---- 区画中央へ前進 ----------------------------------------------

		if (slalom_flag == 0) {
			if (mouse_head0 == GO_FORWARD) {
				//　区画中央へ前進
				if (request_move_mm(45, TRY_STRAIGHT_1ST, TRY_STRAIGHT_1ST,
				TRY_STRAIGHT_1ST,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}

			} else {
				//　区画中央へ前進
				if (request_move_mm(45, TRY_STRAIGHT_1ST, TRY_TURN_150mm_SPEED,
				TRY_STRAIGHT_1ST,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}

			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

		}

		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {
			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);
			initialize_distance();
			initialize_angle_pid();
			//　半区画前進
			if (slalom_flag == 1) {
				if (request_move_mm(45, TRY_TURN_150mm_SPEED, 0.0F,
				TRY_TURN_150mm_SPEED,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}
			} else {
				if (request_move_mm(45, TRY_STRAIGHT_1ST, 0.0F,
				TRY_STRAIGHT_1ST,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

			LED_numeric_lighting(6);
			if (slalom_flag == 0) {
				//　半区画前進
				if (request_move_mm(45, TRY_STRAIGHT_1ST, TRY_STRAIGHT_1ST,
				TRY_STRAIGHT_1ST,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}
			} else {
				if (request_move_mm(45, TRY_TURN_150mm_SPEED, TRY_STRAIGHT_1ST,
				TRY_STRAIGHT_1ST,
				TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			slalom_flag = 0;
			LED_numeric_lighting(0);

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

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

			if (request_move_mm(SLALOM_150_R_IN_OFFSET, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED, TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
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
			if (request_move_mm(SLALOM_150_R_OUT_OFFSET, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED, TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

			LED_numeric_lighting(1);

			//L-turn

			// 距離カウンタリセット
			initialize_distance();

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

			if (request_move_mm(SLALOM_150_L_IN_OFFSET, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED, TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			//角速度PID初期化
			initialize_angle_pid();

			if (request_slalom(SLALOM_150_L_90_DEG, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を0に戻す
			change_body_angular_velocity_parameters(SLALOM_150_L_90_DEG, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();
			if (request_move_mm(SLALOM_150_L_OUT_OFFSET, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_SPEED, TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(SLALOM_150_L_OUT_OFFSET, DISTANCE,
					SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
			initialize_distance();

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

			//１区画まで前進
			if (request_move_mm(45, TRY_TURN_150mm_SPEED, 0.0F,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(0, 0.0F, 0.0F, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_move_mm(45, TRY_TURN_150mm_SPEED, 0.0F,
			TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}
///######################################################################

//		Try（足立法）STRAIGHT:600mm, TURN:300mm

///######################################################################
void mouse_try_STRAIGHT_2ND_TURN_300mm_Adachi(int tx, int ty) {

	short temp0;

	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

	make_smap(tx, ty, T_MODE); /* 等高線マップを作る			*/

	while (1) {
		change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);

		//前もって次の区間の移動方向を確定

		// 座標更新
		if (Maze.head == 0)				//北
			Maze.my += 1;
		else if (Maze.head == 1)		//東
			Maze.mx += 1;
		else if (Maze.head == 2)		//南
			Maze.my -= 1;
		else if (Maze.head == 3)		//西
			Maze.mx -= 1;

		/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
		/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
		/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
		/* (既探索区間,旋回）の順で選択する								*/
		temp0 = Maze_Map[Maze.my][Maze.mx];
		s0 = 1024;
		if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
			s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 0) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_FORWARD;
			}
		}
		if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 1) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_RIGHT;
			}
		}
		if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
			s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
			if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 2) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_BACK;
			}
		}
		if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
			s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
			if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
				s1 = s1 - 2;
			}
			if (Maze.head == 3) {
				s1 = s1 - 1;
			}
			if (s1 < s0) {
				s0 = s1;
				mouse_head0 = GO_LEFT;
			}
		}

		mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

		// ---- 区画中央へ前進 ----------------------------------------------

		if (slalom_flag == 0) {

			if (mouse_head0 == GO_FORWARD) {
				//　区画中央へ前進
				if (request_move_mm(45, TRY_STRAIGHT_2ND, TRY_STRAIGHT_2ND,
				TRY_STRAIGHT_2ND,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}

			} else {
				//　区画中央へ前進
				if (request_move_mm(45, TRY_STRAIGHT_2ND, TRY_TURN_300mm_SPEED,
				TRY_STRAIGHT_2ND,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}

			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

		}

		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {

			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);
			initialize_distance();
			//　半区画前進
			if (slalom_flag == 1) {
				if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
				TRY_TURN_300mm_SPEED,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			} else {
				if (request_move_mm(45, TRY_STRAIGHT_2ND, 0.0F,
				TRY_STRAIGHT_2ND,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

			LED_numeric_lighting(6);
			if (slalom_flag == 0) {
				//　半区画前進
				if (request_move_mm(45, TRY_STRAIGHT_2ND, TRY_STRAIGHT_2ND,
				TRY_STRAIGHT_2ND,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			} else {
				if (request_move_mm(45, TRY_TURN_300mm_SPEED, TRY_STRAIGHT_2ND,
				TRY_STRAIGHT_2ND,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			slalom_flag = 0;
			LED_numeric_lighting(0);

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

			initialize_angle_pid();

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

			if (request_move_mm(SLALOM_300_R_IN_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

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
			if (request_move_mm(SLALOM_300_R_OUT_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

			LED_numeric_lighting(1);

			initialize_angle_pid();

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

			if (request_move_mm(SLALOM_300_L_IN_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

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
			if (request_move_mm(SLALOM_300_L_OUT_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
			initialize_distance();

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

			//１区画まで前進
			if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(0, 0.0F, 0.0F, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}
///######################################################################

//		Try（足立法）STRAIGHT:750mm, TURN:300mm

///######################################################################
void mouse_try_STRAIGHT_3RD_TURN_300mm_Adachi(int tx, int ty) {

	short temp0;

	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

	make_smap(tx, ty, T_MODE); /* 等高線マップを作る			*/

	change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);

	//前もって次の区間の移動方向を確定

	// 座標更新
	if (Maze.head == 0)				//北
		Maze.my += 1;
	else if (Maze.head == 1)		//東
		Maze.mx += 1;
	else if (Maze.head == 2)		//南
		Maze.my -= 1;
	else if (Maze.head == 3)		//西
		Maze.mx -= 1;

	/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
	/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
	/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
	/* (既探索区間,旋回）の順で選択する								*/
	temp0 = Maze_Map[Maze.my][Maze.mx];
	s0 = 1024;
	if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
		s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
		if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
			s1 = s1 - 2;
		}
		if (Maze.head == 0) {
			s1 = s1 - 1;
		}
		if (s1 < s0) {
			s0 = s1;
			mouse_head0 = GO_FORWARD;
		}
	}
	if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
		s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
		if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
			s1 = s1 - 2;
		}
		if (Maze.head == 1) {
			s1 = s1 - 1;
		}
		if (s1 < s0) {
			s0 = s1;
			mouse_head0 = GO_RIGHT;
		}
	}
	if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
		s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
		if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
			s1 = s1 - 2;
		}
		if (Maze.head == 2) {
			s1 = s1 - 1;
		}
		if (s1 < s0) {
			s0 = s1;
			mouse_head0 = GO_BACK;
		}
	}
	if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
		s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
		if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
			s1 = s1 - 2;
		}
		if (Maze.head == 3) {
			s1 = s1 - 1;
		}
		if (s1 < s0) {
			s0 = s1;
			mouse_head0 = GO_LEFT;
		}
	}

	mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

	// ---- 区画中央へ前進 ----------------------------------------------

	if (slalom_flag == 0) {

		if (mouse_head0 == GO_FORWARD) {
			//　区画中央へ前進
			if (request_move_mm(45, TRY_STRAIGHT_3RD, TRY_STRAIGHT_3RD,
			TRY_STRAIGHT_3RD,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}

		} else {
			//　区画中央へ前進
			if (request_move_mm(45, TRY_STRAIGHT_3RD, TRY_TURN_300mm_SPEED,
			TRY_STRAIGHT_3RD,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
		}

		//距離を引く
		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

	}

	/* ---- 決定した方向に移動する --------------------------------	*/
	if ((Maze.mx == tx) && (Maze.my == ty)) {

		// ---- 目的地の場合 ----
		LED_numeric_lighting(0);
		initialize_distance();
		//　半区画前進
		if (slalom_flag == 1) {
			if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
		} else {
			if (request_move_mm(45, TRY_STRAIGHT_3RD, 0.0F,
			TRY_STRAIGHT_3RD,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
		}
		//距離を引く
		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
		// 左90度旋回x2
		if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
		SLALOM_150_ANGULAR_VELOCITY,
		SLALOM_150_ANGULAR_ACCEL)) {
			return;
		}
		//角度を初期化
		change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
				SUBSTRACTION);
		if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
		SLALOM_150_ANGULAR_VELOCITY,
		SLALOM_150_ANGULAR_ACCEL)) {
			return;
		}
		//角度を初期化
		change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
				SUBSTRACTION);

		slalom_flag = 0;
		// 壁補正PIDリセット
		initialize_led_pid();

		initialize_angle_pid();

		// 進行方向更新
		mouse_head0 = GO_BACK;
		Maze.head = (Maze.head + mouse_head0) & 3;
		return;

	} else if (mouse_head0 == GO_FORWARD) { // ---- そのまま前進 ----

		LED_numeric_lighting(6);
		if (slalom_flag == 0) {
			//　半区画前進
			if (request_move_mm(45, TRY_STRAIGHT_3RD, TRY_STRAIGHT_3RD,
			TRY_STRAIGHT_3RD,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
		} else {
			if (request_move_mm(45, TRY_TURN_300mm_SPEED, TRY_STRAIGHT_3RD,
			TRY_STRAIGHT_3RD,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
		}
		//距離を引く
		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

		slalom_flag = 0;
		LED_numeric_lighting(0);

	} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
		LED_numeric_lighting(8);

		initialize_angle_pid();

		//R-turn

		// 距離カウンタリセット
		initialize_distance();

		// 壁補正PIDリセット
		initialize_led_pid();

		//距離を計測
		if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
			float distance_wall = 0;
			distance_wall =
					((FRONT_L_REFER - AD_Sensor_Distance_Get(FRONT_LEFT))
							+ (FRONT_R_REFER
									- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
			change_body_velocity_parameters(distance_wall, DISTANCE,
					SUBSTITUTION);
			SET_front_WALL_CONTROL(CONTROL_ON);
		}

		if (request_move_mm(SLALOM_300_R_IN_OFFSET, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
			return;
		}
		SET_front_WALL_CONTROL(CONTROL_OFF);

		// 距離カウンタリセット
		initialize_distance();

		// 壁補正PIDリセット
		initialize_led_pid();

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
		if (request_move_mm(SLALOM_300_R_OUT_OFFSET, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
		slalom_flag = 1;

	} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

		LED_numeric_lighting(1);

		initialize_angle_pid();

		//L-turn

		// 距離カウンタリセット
		initialize_distance();
		// 壁補正PIDリセット
		initialize_led_pid();
		//距離を計測
		if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
			float distance_wall = 0;
			distance_wall =
					((FRONT_L_REFER - AD_Sensor_Distance_Get(FRONT_LEFT))
							+ (FRONT_R_REFER
									- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
			change_body_velocity_parameters(distance_wall, DISTANCE,
					SUBSTITUTION);
			SET_front_WALL_CONTROL(CONTROL_ON);
		}

		if (request_move_mm(SLALOM_300_L_IN_OFFSET, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
			return;
		}

		SET_front_WALL_CONTROL(CONTROL_OFF);

		// 壁補正PIDリセット
		initialize_led_pid();

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
		if (request_move_mm(SLALOM_300_L_OUT_OFFSET, TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
		slalom_flag = 1;

	} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

		LED_numeric_lighting(9);
		initialize_distance();

		//距離を計測
		if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
			float distance_wall = 0;
			distance_wall =
					((FRONT_L_REFER - AD_Sensor_Distance_Get(FRONT_LEFT))
							+ (FRONT_R_REFER
									- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
			change_body_velocity_parameters(distance_wall, DISTANCE,
					SUBSTITUTION);
			SET_front_WALL_CONTROL(CONTROL_ON);
		}

		//１区画まで前進
		if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
			return;
		}
		SET_front_WALL_CONTROL(CONTROL_OFF);
		// 左90度旋回x2
		if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
		SLALOM_150_ANGULAR_VELOCITY,
		SLALOM_150_ANGULAR_ACCEL)) {
			return;
		}
		//角度を初期化
		change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
				SUBSTRACTION);

		//距離を計測
		if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
			float distance_wall = 0;
			distance_wall = ((FRONT_L_CENTER_REFER
					- AD_Sensor_Distance_Get(FRONT_LEFT))
					+ (FRONT_R_CENTER_REFER
							- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
			change_body_velocity_parameters(distance_wall, DISTANCE,
					SUBSTITUTION);
			SET_front_WALL_CONTROL(CONTROL_ON);
		}

		if (request_move_mm(0, 0.0F, 0.0F, TRY_TURN_150mm_SPEED,
		TRY_TURN_150mm_ACCEL, CONTROL_OFF)) {
			return;
		}

		delay(400);

		SET_front_WALL_CONTROL(CONTROL_OFF);
		//距離リセット
		initialize_distance();

		if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
		SLALOM_150_ANGULAR_VELOCITY,
		SLALOM_150_ANGULAR_ACCEL)) {
			return;
		}
		//角度を初期化
		change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
				SUBSTRACTION);

		// 距離カウンタリセット
		initialize_distance();

		// 壁補正PIDリセット
		initialize_led_pid();

		if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
		TRY_TURN_300mm_SPEED,
		TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
			return;
		}
		//距離を引く
		change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

		LED_numeric_lighting(0);
		slalom_flag = 1;

		// 壁補正PIDリセット
		initialize_led_pid();
	}
	// 進行方向更新
	Maze.head = (Maze.head + mouse_head0) & 3;
} //end straight:750mm/s, turn:300mm/s
///######################################################################

//		Try（足立法）STRAIGHT:600mm, TURN:300mm

///######################################################################
void mouse_try_STRAIGHT_4TH_TURN_300mm_Adachi(int tx, int ty) {

	short temp0;
	short num_block = 0;
	short s0, s1;
//探査モードの切り替え(mode_num)

	short mouse_head0 = 0;		// 機体の相対進行方向

	change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
			SUBSTITUTION);
//	slalom_flag = 0;

	make_smap(tx, ty, T_MODE); /* 等高線マップを作る			*/

	while (1) {
		change_body_velocity_parameters(TRY_TURN_300mm_ACCEL, TARGET_ACCEL,
				SUBSTITUTION);

		//前もって次の区間の移動方向を確定
		num_block = 0;		//直進区画数のリセット
		while (1) {

			// 座標更新
			if (Maze.head == 0)				//北
				Maze.my += 1;
			else if (Maze.head == 1)		//東
				Maze.mx += 1;
			else if (Maze.head == 2)		//南
				Maze.my -= 1;
			else if (Maze.head == 3)		//西
				Maze.mx -= 1;

			/* 回りの4区間の内、その方向に壁がなくて、目的地に一番近い		*/
			/* 区間に移動する。ただし、移動できる一番近い区間が複数ある		*/
			/* 場合は(未探索区間,直進)(未探索区間,旋回)(既探索区間,直進）	*/
			/* (既探索区間,旋回）の順で選択する								*/
			temp0 = Maze_Map[Maze.my][Maze.mx];
			s0 = 1024;
			if ((temp0 & 1) == 0) { /*	北方向の区間の確認		*/
				s1 = smap[Maze.my + 1][Maze.mx] * 4 + 4;
				if ((Maze_Map[Maze.my + 1][Maze.mx] & 0x0f0) != 0x0f0) {
					s1 = s1 - 2;
				}
				if (Maze.head == 0) {
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					mouse_head0 = GO_FORWARD;
				}
			}
			if ((temp0 & 2) == 0) { /*	東方向の区間の確認		*/
				s1 = smap[Maze.my][Maze.mx + 1] * 4 + 4;
				if ((Maze_Map[Maze.my][Maze.mx + 1] & 0x0f0) != 0x0f0) {
					s1 = s1 - 2;
				}
				if (Maze.head == 1) {
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					mouse_head0 = GO_RIGHT;
				}
			}
			if ((temp0 & 4) == 0) { /*	南方向の区間の確認		*/
				s1 = smap[Maze.my - 1][Maze.mx] * 4 + 4;
				if ((Maze_Map[Maze.my - 1][Maze.mx] & 0x0f0) != 0x0f0) {
					s1 = s1 - 2;
				}
				if (Maze.head == 2) {
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					mouse_head0 = GO_BACK;
				}
			}
			if ((temp0 & 8) == 0) { /*	西方向の区間の確認		*/
				s1 = smap[Maze.my][Maze.mx - 1] * 4 + 4;
				if ((Maze_Map[Maze.my][Maze.mx - 1] & 0x0f0) != 0x0f0) {
					s1 = s1 - 2;
				}
				if (Maze.head == 3) {
					s1 = s1 - 1;
				}
				if (s1 < s0) {
					s0 = s1;
					mouse_head0 = GO_LEFT;
				}
			}

			mouse_head0 = (mouse_head0 - Maze.head) & 3; /* 移動する方向を決定		*/

			if (mouse_head0 != GO_FORWARD) {
				break;
			} else {
				num_block++;
			}
		}

		if (num_block != 0) {		// ---- そのまま前進 ----

			float request_distance = num_block * 90;

			if (request_move_mm(request_distance, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_STRAIGHT_4TH,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}

			//距離を引く
			change_body_velocity_parameters(request_distance, DISTANCE,
					SUBSTRACTION);

			slalom_flag = 1;
			LED_numeric_lighting(0);
		}
		/* ---- 決定した方向に移動する --------------------------------	*/
		if ((Maze.mx == tx) && (Maze.my == ty)) {

			// ---- 目的地の場合 ----
			LED_numeric_lighting(0);
			initialize_distance();
			//　半区画前進
			if (slalom_flag == 1) {
				if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
				TRY_TURN_300mm_SPEED,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			} else {
				if (request_move_mm(45, TRY_STRAIGHT_3RD, 0.0F,
				TRY_STRAIGHT_3RD,
				TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
					return;
				}
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			slalom_flag = 0;
			// 壁補正PIDリセット
			initialize_led_pid();

			initialize_angle_pid();

			// 進行方向更新
			mouse_head0 = GO_BACK;
			Maze.head = (Maze.head + mouse_head0) & 3;
			return;

		} else if (mouse_head0 == GO_RIGHT) { // ---- 右に旋回する ----
			LED_numeric_lighting(8);

			initialize_angle_pid();

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

			if (request_move_mm(SLALOM_300_R_IN_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

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
			if (request_move_mm(SLALOM_300_R_OUT_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_LEFT) { // ---- 左に旋回する ----

			LED_numeric_lighting(1);

			initialize_angle_pid();

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

			if (request_move_mm(SLALOM_300_L_IN_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}

			SET_front_WALL_CONTROL(CONTROL_OFF);

			// 壁補正PIDリセット
			initialize_led_pid();

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
			if (request_move_mm(SLALOM_300_L_OUT_OFFSET, TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_SPEED, TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
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
			slalom_flag = 1;

		} else if (mouse_head0 == GO_BACK) { // ---- 反転して戻る ----

			LED_numeric_lighting(9);
			initialize_distance();

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

			//１区画まで前進
			if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			SET_front_WALL_CONTROL(CONTROL_OFF);
			// 左90度旋回x2
			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			//距離を計測
			if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
				float distance_wall = 0;
				distance_wall = ((FRONT_L_CENTER_REFER
						- AD_Sensor_Distance_Get(FRONT_LEFT))
						+ (FRONT_R_CENTER_REFER
								- AD_Sensor_Distance_Get(FRONT_RIGHT))) / 2;
				change_body_velocity_parameters(distance_wall, DISTANCE,
						SUBSTITUTION);
				SET_front_WALL_CONTROL(CONTROL_ON);
			}

			if (request_move_mm(0, 0.0F, 0.0F, TRY_TURN_150mm_SPEED,
			TRY_TURN_150mm_ACCEL, CONTROL_OFF)) {
				return;
			}

			delay(400);

			SET_front_WALL_CONTROL(CONTROL_OFF);
			//距離リセット
			initialize_distance();

			if (request_slalom(COUNTER_ROTATION_TURN, 0.0F, 0.0F,
			SLALOM_150_ANGULAR_VELOCITY,
			SLALOM_150_ANGULAR_ACCEL)) {
				return;
			}
			//角度を初期化
			change_body_angular_velocity_parameters(COUNTER_ROTATION_TURN, DEG,
					SUBSTRACTION);

			// 距離カウンタリセット
			initialize_distance();

			// 壁補正PIDリセット
			initialize_led_pid();

			if (request_move_mm(45, TRY_TURN_300mm_SPEED, 0.0F,
			TRY_TURN_300mm_SPEED,
			TRY_TURN_300mm_ACCEL, CONTROL_ON)) {
				return;
			}
			//距離を引く
			change_body_velocity_parameters(45, DISTANCE, SUBSTRACTION);

			LED_numeric_lighting(0);
			slalom_flag = 1;

			// 壁補正PIDリセット
			initialize_led_pid();
		}
		// 進行方向更新
		Maze.head = (Maze.head + mouse_head0) & 3;
	}
}
void ENABLE_SLALOM_FLAG(void) {
	slalom_flag = 1;
}
void DISABLE_SLALOM_FLAG(void) {
	slalom_flag = 0;
}
/* ====================================================================	*/
/*	ＭＡＰ																	*/
/* 壁データ記録方法															*/
/* 壁の位置 0bit:上 1bit:右 2bit:下 3bit:左	 1:壁有り 0:壁無し					*/
/* 壁の探索 4bit:上 5bit:右 6bit:下 7bit:左	 1:済み	 0:未探索					*/
/* ====================================================================	*/
/******************************************************
 * マップデータ初期化
 */
void MAZE_Map_Clear(void) {
	short x, y;
	char d;

	/* すべてのマップデータを未探索状態にする */
	for (y = 0; y < 16; y++) {
		for (x = 0; x < 16; x++) {
			d = 0x0000;
			if ((x == 0) && (y == 0))
				d = 0xfe;
			else if ((x == 1) && (y == 0))
				d = 0xcc;
			else if ((x == 15) && (y == 0))
				d = 0x66;
			else if ((x == 0) && (y == 15))
				d = 0x99;
			else if ((x == 15) && (y == 15))
				d = 0x33;
			else if (x == 0)
				d = 0x88;
			else if (x == 15)
				d = 0x22;
			else if (y == 0)
				d = 0x44;
			else if (y == 15)
				d = 0x11;
			Maze_Map[y][x] = d;
		}
	}
}
/* ====================================================================	*/
/* 等高線作成モジュール														*/
/* ====================================================================	*/
void make_smap(int gx, int gy, int mode) {
	const short initial_value = MAX_SMAP;
	short pt0, pt1, ct;
	short x, y, z;
	char wdata;

	for (x = 0; x < NUM_BLOCK; x++) { /* 等高線マップを初期化する				*/
		for (y = 0; y < NUM_BLOCK; y++) {
			smap[x][y] = initial_value;
		}
	}

	smap[gy][gx] = 0; /* 目標地点に距離０を書き込む			*/

	pt0 = 0;
	do {
		ct = 0;
		pt1 = pt0 + 1; //全区画を計算するたび1増える
		for (y = 0; y < NUM_BLOCK; y++) {
			for (x = 0; x < NUM_BLOCK; x++) {
				if (smap[y][x] == pt0) {
					wdata = Maze_Map[y][x];
					if (mode == T_MODE) { //壁がある前提
						if (((wdata & 0x11) == 0x10)
								&& (y != (NUM_BLOCK - 1))) { //N壁なし、探査済み
							if (smap[y + 1][x] == initial_value) {
								smap[y + 1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x22) == 0x20)
								&& (x != (NUM_BLOCK - 1))) { //E壁なし、探査済み
							if (smap[y][x + 1] == initial_value) {
								smap[y][x + 1] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x44) == 0x40) && (y != 0)) { //S壁なし、探査済み
							if (smap[y - 1][x] == initial_value) {
								smap[y - 1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x88) == 0x80) && (x != 0)) { //W壁なし、探査済み
							if (smap[y][x - 1] == initial_value) {
								smap[y][x - 1] = pt1;
								ct++;
							}
						}
					} else { //S-MODE壁がない前提
						if (((wdata & 0x01) == 0x00)
								&& (y != (NUM_BLOCK - 1))) { //N壁なし
							if (smap[y + 1][x] == initial_value) {
								smap[y + 1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x02) == 0x00)
								&& (x != (NUM_BLOCK - 1))) { //E壁なし
							if (smap[y][x + 1] == initial_value) {
								smap[y][x + 1] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x04) == 0x00) && (y != 0)) { //S壁なし
							if (smap[y - 1][x] == initial_value) {
								smap[y - 1][x] = pt1;
								ct++;
							}
						}
						if (((wdata & 0x08) == 0x00) && (x != 0)) { //W壁なし
							if (smap[y][x - 1] == initial_value) {
								smap[y][x - 1] = pt1;
								ct += 1;
							}
						}
					}
				}
			}
		}
		pt0 = pt0 + 1;
	} while (ct != 0);
}
/* ====================================================================	*/
/* センサ情報から迷路データを作りマップに書き込むモジュール								*/
/* ====================================================================	*/
void make_map_data(void) {
	char wall;

	/* 走行中にセンサから得た壁情報をＭＡＰデータに書きこむ */
	if ((Maze.mx == 0) && (Maze.my == 0)) {
		wall = 0x0fe; //0b 1111 1110
	} else {
		wall = get_wall_data();
	}
	Maze_Map[Maze.my][Maze.mx] = wall;

	/* 隣の区間のＭＡＰデータも更新する */
	if (Maze.mx != 15) {
		Maze_Map[Maze.my][Maze.mx + 1] = (Maze_Map[Maze.my][Maze.mx + 1] & 0x77)
				| 0x80 | ((wall << 2) & 0x08);
	}
	if (Maze.mx != 0) {
		Maze_Map[Maze.my][Maze.mx - 1] = (Maze_Map[Maze.my][Maze.mx - 1] & 0xdd)
				| 0x20 | ((wall >> 2) & 0x02);
	}
	if (Maze.my != 15) {
		Maze_Map[Maze.my + 1][Maze.mx] = (Maze_Map[Maze.my + 1][Maze.mx] & 0xbb)
				| 0x40 | ((wall << 2) & 0x04);
	}
	if (Maze.my != 0) {
		Maze_Map[Maze.my - 1][Maze.mx] = (Maze_Map[Maze.my - 1][Maze.mx] & 0xee)
				| 0x10 | ((wall >> 2) & 0x01);
	}
}

unsigned char get_wall_data(void) {
	unsigned char wall;

	/* センサデータの入力し閾値と比較し壁の有無を判定する */
	wall = 0;
	if (AD_Wall_Look(WALL_FORWARD) == 1) {
		wall = wall | 0x11; //0b 1000 1000(前壁あり、探査済み)
	}
	if (AD_Wall_Look(WALL_LEFT) == 1) {
		wall = wall | 0x88; //0b 0001 0001(左壁あり、探査済み)
	}
	if (AD_Wall_Look(WALL_RIGHT) == 1) {
		wall = wall | 0x22; //0b 0100 0100(右壁あり、探査済み)
	}

	/* マウスの進行方向にあわせてセンサデータを移動し壁データとする */
	if (Maze.head == 1) { //東
		wall = wall >> 3; //
	} else if (Maze.head == 2) { //南
		wall = wall >> 2;
	} else if (Maze.head == 3) { //西
		wall = wall >> 1;
	} else {
	}
	/* 探索済みフラグを立てる */
	return (wall | 0xf0);

}
