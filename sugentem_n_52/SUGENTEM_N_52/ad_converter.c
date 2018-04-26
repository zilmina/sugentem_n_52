/*
 * ad_converter.c
 *
 *  Created on: 2017/09/27
 *      Author: 廣明
 */
#include "math.h"
#include "adjust.h"
#include "iodefine.h"
#include "ad_converter.h"
#include "sci.h"
#include "hardware_infomation_for_sugentem_n_52.h"
/*==============================================================*/
/*このソースコードはRX631用のCMT0モジュールを操作して							*/
/*電圧の取得、更新												*/
/*を実現する内容です。												*/
/*																*/
/*前提条件は														*/
/*	(マニュアルないではPCLKはPCLKBを意味してる。)							*/
/*PCLK = 50MHz													*/
/*																*/
/*																*/
/*																*/
/*																*/
/****************************************************************/
/*==============================================================================
 *
 * グローバル変数
 *
 *============================================================================== */
volatile static struct Machine_sensor mouse;
volatile static struct Opt_sensor led_front;
volatile static struct Opt_sensor led_side;

#define FILLERING_TIME 3//mill seconds
static char front_ring_buffer_window = 0;
static char side_ring_buffer_window = 0;
static short side_right_sensor_log[FILLERING_TIME] = { 0 };
static short front_right_sensor_log[FILLERING_TIME] = { 0 };
static short front_left_sensor_log[FILLERING_TIME] = { 0 };
static short side_left_sensor_log[FILLERING_TIME] = { 0 };

/*==============================================================================
 *
 * グローバル関数
 *
 *============================================================================== */

void ad_converter_initialize(void) {

	PORTB.PDR.BIT.B1 = 1;			//LED_FRONT
	PORTB.PDR.BIT.B3 = 1;			//LED_SIDE
	LED_FRONT = AD_LED_OFF;
	LED_SIDE = AD_LED_OFF;
	led_front.flag = 1;
	led_side.flag = 1;

	SYSTEM.PRCR.WORD = 0xA502;		// Release Protect
	MSTP(S12AD) = 0;				// Wake up S12AD
	SYSTEM.PRCR.WORD = 0xA500;		// Protect

	//Set PDR
	PORTE.PDR.BIT.B3 = 0;			// Set PE3: Input
	PORTE.PDR.BIT.B1 = 0;			// Set PE1: Input
	PORT4.PDR.BIT.B0 = 0;			// Set P40: Input
	PORT4.PDR.BIT.B1 = 0;			// Set P41: Input
	PORT4.PDR.BIT.B2 = 0;			// Set P42: Input

	// Set MPC
	PORTE.PMR.BIT.B3 = 1;			// Set PE3: Peripheral
	PORTE.PMR.BIT.B1 = 1;			// Set PE1: Peripheral
	PORT4.PMR.BIT.B0 = 1;			// Set P40: Peripheral
	PORT4.PMR.BIT.B1 = 1;			// Set P41: Peripheral
	PORT4.PMR.BIT.B2 = 1;			// Set P42: Peripheral

	MPC.PWPR.BIT.B0WI = 0;			// Release protect
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PE3PFS.BIT.ASEL = 1;		// Set PE3: Analog Input
	MPC.PE1PFS.BIT.ASEL = 1;		// Set PE1: Analog Input
	MPC.P40PFS.BIT.ASEL = 1;		// Set P40: Analog Input
	MPC.P41PFS.BIT.ASEL = 1;		// Set P41: Analog Input
	MPC.P42PFS.BIT.ASEL = 1;		// Set P42: Analog Input
	MPC.PWPR.BIT.PFSWE = 0;			// Protect
	MPC.PWPR.BIT.B0WI = 1;

	// S12AD Settings software trigger
	S12AD.ADCSR.BIT.CKS = 0;		// PCLK/8 = 50MHz/8 = 6.25MHz 160us
	S12AD.ADCSR.BIT.ADIE = 0;		// Enable S12ADI0
	S12AD.ADCSR.BIT.ADCS = 0;		// Single scanning
	S12AD.ADCSR.BIT.ADST = 0;		// Stop S12AD
	S12AD.ADANS0.WORD = 0x0000;		// S12ADC: no
	S12AD.ADADS0.WORD = 0x0000;		// Disable ADD result Mode
	S12AD.ADCER.BIT.ACE = 1;		// Auto clearing
	S12AD.ADCER.BIT.ADRFMT = 0;		// Right -justified
	S12AD.ADSSTR01.BIT.SST1 = 5;	// 800us

	//基準値の代入
	led_front.right.lim = FRONT_R_LIMIT;
	led_front.left.lim = FRONT_L_LIMIT;
	led_side.right.lim = SIDE_R_LIMIT;
	led_side.left.lim = SIDE_L_LIMIT;

	led_front.right.refer = FRONT_R_REFER;
	led_front.left.refer = FRONT_L_REFER;
	led_side.right.refer = SIDE_R_REFER;
	led_side.left.refer = SIDE_L_REFER;

}
void update_battery_voltage(void) {	///バッテリー電圧の更新---------------------------------------------
	S12AD.ADANS0.WORD = ANALOG_PORT_AD_BATTERY;

//AD変換開始
	S12AD.ADCSR.BIT.ADST = 1;
	while (S12AD.ADCSR.BIT.ADST == 1)
		;
//電圧に変換したいとき　0.80586081Fを掛ける
//バッテリーは分圧しているので2倍
	mouse.raw_battery_voltage = S12AD.ADDR11;
	mouse.battery_voltage = (short) ((float) (mouse.raw_battery_voltage)
			* 1.61172161F);
}
void update_front_sensor_voltage(void) {///正面センサー電圧の更新---------------------------------------------

	if (led_front.flag == 1) {					//　許可があればフロントLEDを点灯

		LED_FRONT = AD_LED_OFF;					// フロントLED消灯

		S12AD.ADANS0.WORD =
		ANALOG_PORT_LED_LEFT_FRONT | ANALOG_PORT_LED_RIGHT_FRONT;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		// 計測した値を変数に格納
		led_front.right.dark_raw_value = S12AD.ADDR1;
		led_front.left.dark_raw_value = S12AD.ADDR2;

		LED_FRONT = AD_LED_ON;					// フロントLED点灯

		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）

		S12AD.ADANS0.WORD =
		ANALOG_PORT_LED_LEFT_FRONT | ANALOG_PORT_LED_RIGHT_FRONT;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		LED_FRONT = AD_LED_OFF;					// フロントLED消灯

		// 計測した値を変数に格納
		led_front.right.bright_raw_value = S12AD.ADDR1;
		led_front.left.bright_raw_value = S12AD.ADDR2;

		front_right_sensor_log[front_ring_buffer_window] =
				led_front.right.bright_raw_value
						- led_front.right.dark_raw_value;

		front_left_sensor_log[front_ring_buffer_window] =
				led_front.left.bright_raw_value - led_front.left.dark_raw_value;
		if (front_ring_buffer_window == (FILLERING_TIME - 1)) {	//ring buffer windowの移動
			front_ring_buffer_window = 0;
		} else {
			front_ring_buffer_window++;
		}
		short front_right_difference_raw_value = 0;
		short front_left_difference_raw_value = 0;
		for (int i = 0; i < FILLERING_TIME; i++) {
			front_right_difference_raw_value += front_right_sensor_log[i];
			front_left_difference_raw_value += front_left_sensor_log[i];
		}
		led_front.right.difference_raw_value = (front_right_difference_raw_value
				/ FILLERING_TIME);
		led_front.left.difference_raw_value = (front_left_difference_raw_value
				/ FILLERING_TIME);
	} else {
		LED3 = LED_ON;

		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）

		S12AD.ADANS0.WORD =
		ANALOG_PORT_LED_LEFT_FRONT | ANALOG_PORT_LED_RIGHT_FRONT;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		LED3 = LED_OFF;					// フロントLED消灯

		// 計測した値を変数に格納
		led_front.right.bright_raw_value = S12AD.ADDR1;
		led_front.left.bright_raw_value = S12AD.ADDR2;
	}
}
void update_side_sensor_voltage(void) {	///左右センサー電圧の更新---------------------------------------------
//	if (led_front.flag == 1) {					//　許可があればフロントLEDを点灯
//
//		LED_FRONT = AD_LED_OFF;					// フロントLED消灯
//
//		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
//				| ANALOG_PORT_LED_RIGHT_SIDE;
//		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始
//
//		//　変換終了まで待機
//		while (S12AD.ADCSR.BIT.ADST == 1)
//			;
//		S12AD.ADANS0.WORD = 0x00;				// 設定初期化
//
//		// 計測した値を変数に格納
//		led_side.right.dark_raw_value = S12AD.ADDR0;
//		led_side.left.dark_raw_value = S12AD.ADDR9;
//
//		LED_SIDE = AD_LED_ON;					// フロントLED点灯
//
//		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）
//
//		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
//				| ANALOG_PORT_LED_RIGHT_SIDE;
//		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始
//
//		//　変換終了まで待機
//		while (S12AD.ADCSR.BIT.ADST == 1)
//			;
//		S12AD.ADANS0.WORD = 0x00;				// 設定初期化
//
//		LED_FRONT = AD_LED_OFF;					// フロントLED消灯
//
//		// 計測した値を変数に格納
//		led_side.right.bright_raw_value = S12AD.ADDR0;
//		led_side.left.bright_raw_value = S12AD.ADDR9;
//
//		led_side.right.difference_raw_value = led_side.right.bright_raw_value
//				- led_front.right.dark_raw_value;
//		led_side.left.difference_raw_value = led_side.left.bright_raw_value
//				- led_side.left.dark_raw_value;
//	} else {
//		LED4 = LED_ON;
//		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）
//
//		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
//				| ANALOG_PORT_LED_RIGHT_SIDE;
//		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始
//
//		//　変換終了まで待機
//		while (S12AD.ADCSR.BIT.ADST == 1)
//			;
//		S12AD.ADANS0.WORD = 0x00;				// 設定初期化
//
//		LED4 = LED_OFF;					// フロントLED消灯
//
//		// 計測した値を変数に格納
//		led_side.right.dark_raw_value = S12AD.ADDR0;
//		led_side.left.dark_raw_value = S12AD.ADDR9;
//	}
	if (led_front.flag == 1) {					//　許可があればフロントLEDを点灯
		LED_SIDE = AD_LED_OFF;
		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
				| ANALOG_PORT_LED_RIGHT_SIDE;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		// 計測した値を変数に格納
		led_side.right.dark_raw_value = S12AD.ADDR0;
		led_side.left.dark_raw_value = S12AD.ADDR9;
		LED_SIDE = AD_LED_ON;					// フロントLED点灯

		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）

		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
				| ANALOG_PORT_LED_RIGHT_SIDE;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		// 計測した値を変数に格納
		led_side.right.bright_raw_value = S12AD.ADDR0;
		led_side.left.bright_raw_value = S12AD.ADDR9;
	} else {

		TIMER_Wait_For_Loop(100);				//　少し待機（LEDの発光が強くなるまで）

		S12AD.ADANS0.WORD = ANALOG_PORT_LED_LEFT_SIDE
				| ANALOG_PORT_LED_RIGHT_SIDE;
		S12AD.ADCSR.BIT.ADST = 1;				//　変換開始

		//　変換終了まで待機
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADANS0.WORD = 0x00;				// 設定初期化

		LED4 = LED_OFF;					// フロントLED消灯

		// 計測した値を変数に格納
		led_side.right.dark_raw_value = S12AD.ADDR0;
		led_side.left.dark_raw_value = S12AD.ADDR9;
	}
	side_right_sensor_log[side_ring_buffer_window] =
			led_side.right.bright_raw_value - led_side.right.dark_raw_value;

	side_left_sensor_log[side_ring_buffer_window] =
			led_side.left.bright_raw_value - led_side.left.dark_raw_value;
	if (side_ring_buffer_window == (FILLERING_TIME - 1)) {//ring buffer windowの移動
		side_ring_buffer_window = 0;
	} else {
		side_ring_buffer_window++;
	}
	short side_right_difference_raw_value = 0;
	short side_left_difference_raw_value = 0;
	for (int i = 0; i < FILLERING_TIME; i++) {
		side_right_difference_raw_value += side_right_sensor_log[i];
		side_left_difference_raw_value += side_left_sensor_log[i];
	}
	led_side.right.difference_raw_value = (side_right_difference_raw_value
			/ FILLERING_TIME);
	led_side.left.difference_raw_value = (side_left_difference_raw_value
			/ FILLERING_TIME);
}
short battery_voltage(void) {
	return mouse.battery_voltage;
}

/*****************************************************
 * 壁のセンサ値取得
 * @param status
 * SIDE_LEFT:左壁
 * SIDE_RIGHT:右壁
 * FRONT_LEFT:左前壁
 * FRONT_RIGHT:右前壁
 *
 * return : 取得値
 */
short AD_Sensor_Value_Get(enum Opto_dir status) {
	short ans = 0;

	switch (status) {

	case SIDE_LEFT:		// 左壁補正値
		ans = led_side.left.difference_raw_value;
		break;
	case SIDE_RIGHT:		// 右壁補正値
		ans = led_side.right.difference_raw_value;
		break;
	case FRONT_LEFT:		// 左前壁補正値
		ans = led_front.left.difference_raw_value;
		break;
	case FRONT_RIGHT:		// 右前壁補正値
		ans = led_front.right.difference_raw_value;
		break;
	default:
		break;
	}
	return ans;

}

/*****************************************************
 * 壁の距離値取得
 * @param status
 * SIDE_LEFT:左壁
 * SIDE_RIGHT:右壁
 * FRONT_LEFT:左前壁
 * FRONT_RIGHT:右前壁
 *
 * return : 取得値
 */
float AD_Sensor_Distance_Get(enum Opto_dir status) {
	float ans = 0;

	switch (status) {

	case SIDE_LEFT:		// 左壁補正値
		ans = (545.66F * pow(led_side.left.difference_raw_value, -0.624F));
		break;
	case SIDE_RIGHT:		// 右壁補正値
		ans = (224.2F * pow(led_side.right.difference_raw_value, -0.61F));
		break;
	case FRONT_LEFT:		// 左前壁補正値
		ans = (2076.5F * pow(led_front.left.difference_raw_value, -0.758F));
		break;
	case FRONT_RIGHT:		// 右前壁補正値
		ans = (616.21F * pow(led_front.right.difference_raw_value, -0.592F));
		break;
	default:
		break;
	}
	return ans;

}

/******************************************************::
 * 横壁の補正値
 * @param status
 * SIDE_LEFT:左壁
 * SIDE_RIGHT:右壁
 * FRONT_LEFT:左前壁
 * FRONT_RIGHT:右前壁
 *
 * return: 補正値
 */
float AD_Line_Error_Get(void) {

	float ans = 0;

	if (AD_Wall_ref_Look(WALL_RIGHT) == WALL_EXIT
			&& AD_Wall_ref_Look(WALL_LEFT) == WALL_EXIT) {
		ans = (AD_Sensor_Distance_Get(SIDE_RIGHT) - led_side.right.refer)
				- (AD_Sensor_Distance_Get(SIDE_LEFT) - led_side.left.refer);
	} else if (AD_Wall_ref_Look(WALL_RIGHT) == WALL_EXIT) {
		ans = (AD_Sensor_Distance_Get(SIDE_RIGHT) - led_side.right.refer);
		ans *= 2;
	} else if (AD_Wall_ref_Look(WALL_LEFT) == WALL_EXIT) {
		ans = -(AD_Sensor_Distance_Get(SIDE_LEFT) - led_side.left.refer);
		ans *= 2;
	}

	return ans;
}
/******************************************************::
 * 前壁の補正値
 * @param status
 * SIDE_LEFT:左壁
 * SIDE_RIGHT:右壁
 * FRONT_LEFT:左前壁
 * FRONT_RIGHT:右前壁
 *
 * return: 補正値
 */
float AD_front_Error_Get(void) {

	float ans = 0;

	if (AD_Wall_Look(WALL_FORWARD) == WALL_EXIT) {
		ans = (AD_Sensor_Distance_Get(FRONT_RIGHT) - FRONT_R_REFER)
				- (AD_Sensor_Distance_Get(FRONT_LEFT) - FRONT_L_REFER);
	}

	return ans;
}
/**************************************************************
 * raw値の表示
 */
void AD_Line_Difference_Print(void) {
	SCI_Str_println("RAW:");
	SCI_Str_print("LS:");
	SCI_Value_print(led_side.left.difference_raw_value, 4);
	SCI_Str_print(" | LF:");
	SCI_Value_print(led_front.left.difference_raw_value, 4);
	SCI_Str_print(" | RF");
	SCI_Value_print(led_front.right.difference_raw_value, 4);
	SCI_Str_print(" | RS:");
	SCI_Value_println(led_side.right.difference_raw_value, 4);
}
/**************************************************************
 * DISTANCE値の表示
 */
void AD_Line_Distance_Print(void) {

	SCI_Str_println("DISTANCE");
	SCI_Str_print("LS:");
	SCI_Value_print(AD_Sensor_Distance_Get(SIDE_LEFT), 4);
	SCI_Str_print(" | LF:");
	SCI_Value_print(AD_Sensor_Distance_Get(FRONT_LEFT), 4);
	SCI_Str_print(" | RF");
	SCI_Value_print(AD_Sensor_Distance_Get(FRONT_RIGHT), 4);
	SCI_Str_print(" | RS:");
	SCI_Value_println(AD_Sensor_Distance_Get(SIDE_RIGHT), 4);
}

/*********************************************************
 * 壁確認関数
 * @return
 * WALL_EXIT	壁あり
 * WALL_NON		壁なし
 *
 * 0: FORWARD
 * 1: RIGHT
 * 2: なし
 * 3: LEFT
 */
enum Wall_decide AD_Wall_Look(enum Wall_dir dir) {

	enum Wall_decide ans = WALL_NON;

	switch (dir) {

	case WALL_FORWARD:		// 前壁
		if ((led_front.left.difference_raw_value > led_front.left.lim)
				|| (led_front.right.difference_raw_value > led_front.right.lim)) {
			ans = WALL_EXIT;
		}
		break;
	case WALL_RIGHT:		// 右壁
		if (led_side.right.difference_raw_value > led_side.right.lim) {
			ans = WALL_EXIT;
		}
		break;
	case WALL_LEFT:			//　左壁
		if (led_side.left.difference_raw_value > led_side.left.lim) {
			ans = WALL_EXIT;
		}
		break;
	default:
		break;
	}
	return ans;
}
//壁確認補正用
enum Wall_decide AD_Wall_ref_Look(enum Wall_dir dir) {

	enum Wall_decide ans = WALL_NON;

	switch (dir) {

	case WALL_FORWARD:		// 前壁
		if ((led_front.left.difference_raw_value > FRONT_L_ref_LIMIT)
				|| (led_front.right.difference_raw_value > FRONT_R_ref_LIMIT)) {
			ans = WALL_EXIT;
		}
		break;
	case WALL_RIGHT:		// 右壁
		if (led_side.right.difference_raw_value > SIDE_R_ref_LIMIT) {
			ans = WALL_EXIT;
		}
		break;
	case WALL_LEFT:			//　左壁
		if (led_side.left.difference_raw_value > SIDE_L_ref_LIMIT) {
			ans = WALL_EXIT;
		}
		break;
	default:
		break;
	}
	return ans;
}

/* WAIT関数(微)						*/
/* --------------------------------	*/
void TIMER_Wait_For_Loop(unsigned long n) {

	volatile long i;

	for (i = 0; i < n; i++)
		;
}
