/*
 * periodic_interrupt_function.c
 *
 *  Created on: 2017/09/28
 *      Author: 廣明
 */

#include "periodic_interrupt_function.h"
#include "iodefine.h"
#include "motor_control.h"
#include "ad_converter.h"
#include "loging.h"
volatile static short flag_interrupt = 1; //1:ON, 0:OFF

void CMT1_initialize(void) {
	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
	MSTP(CMT1) = 0;				// Wake up CMT1//CMT1の運用開始
	SYSTEM.PRCR.WORD = 0xa500;/*クロックソース選択の保護*/

	CMT.CMSTR0.BIT.STR1 = 0;	// Disable CMT1 count
	CMT1_INTERRUPT_PERMISSION = 0;	//CMI1(コンペアマッチ割り込み)禁止

	CMT1_PRESCALER = 0;	//8分周
	CMT1.CMCNT = 0;                         //カウンタのリセット
	CMT1_CMCOR = 1562 - 1;          //で割り込み 50MHz/8/250 = 4kHz すなはち 250us周期の割り込み

	CMT1_INTERRUPT_PERMISSION = 1;                         //CMI0(コンペアマッチ割り込み)許可
	CMT.CMSTR0.BIT.STR1 = 1;	// Enable CMT1 count

	IEN(CMT1,CMI1)= 1;                     //割り込みを許可する
	IPR(CMT1,CMI1)= 5;                     //CMI1の割り込み優先度7を設定(MAX15,MIN1)
}
void TPU0_initialize(void) {
	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
	MSTP(TPU0) = 0;				// Wake up TPU0//TPU0の運用開始
	SYSTEM.PRCR.WORD = 0xa500; /*クロックソース選択の保護*/

	TPUA.TSTR.BIT.CST0 = 0;		//クロックカウント停止
	TPU0.TCR.BIT.TPSC = 1;		//タイマプリスケーラ選択：PCLK/4
	TPU0.TCR.BIT.CKEG = 0;		//クロックカウント要因：内部クロック（立ち下がり）
	TPU0.TMDR.BIT.MD = 0;		// 通常モード
	TPU0.TCR.BIT.CCLR = 1;  	// tcnt = tgra でカウントクリア
	TPU0.TIER.BYTE = 0;			// TPU0.TGRA,B,C,Dの割り込み禁止
	TPU0.TCNT = 0;
	TPU0.TGRA = 3125 - 1;		//50MHz/4/3125 = 4kHz

	IPR(TPU0,TGI0A)= 5;         //TPU0_TGI0Aの割り込み優先度5を設定(MAX15,MIN1)
	IEN(TPU0,TGI0A)= 1;			//割り込みを許可する

	TPU0.TIER.BIT.TGIEA = 1;	//
	TPUA.TSTR.BIT.CST0 = 1;	//クロックカウント開始

}

void interrupt_function(void) {
	static signed char sprit_num = 0;

	if (flag_interrupt == 1) {
		switch (sprit_num) {

		case 0: ///バッテリの電圧確認モーターの出力--------------------------------------------------
			update_battery_voltage();
			motor_power_change();
			update_velocity_parameters();
			update_angular_velocity_parameters();
			sprit_num++;
			break;

		case 1: ///正面のセンサー値の更新--------------------------------------------------
			update_front_sensor_voltage();
			calculate_duty_with_pid();
			sprit_num++;
			break;

		case 2: ///左右のセンサー値の更新--------------------------------------------------
			update_side_sensor_voltage();
			update_encoder();
			sprit_num++;
			break;

		case 3:            ///--------------------------------------------------
//			motor_montrol内に記述中
			logging(check_angular_velocity_PID(I_GAIN));
//			logging(check_side_led_PID(I_GAIN));
			logging(check_body_angular_velocity_parameters(NOW_ANGULAR_VELOCITY));
			update_gyro();
			sprit_num = 0;
			break;

		}
	}

}

void enable_interrupt(void) {
	flag_interrupt = 1;
}
void disable_interrupt(void) {
	flag_interrupt = 0;
}
