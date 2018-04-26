/************************************************************************/
/*						SUGENTEM_N_52ボード用プログラム							*/
/*								命名規則　　								*/
/*ファイル名称:内容を記す名詞または動詞、アンダーバーで区切る。すべて小文字					*/
/*ローカル変数:内容を記す名詞または動詞、アンダーバーで区切る。すべて小文字					*/
/*ローカル関数:(変数型)_内容を記す名詞または動詞、アンダーバーで区切る。すべて小文字			*/
/*グローバル変数	:内容を記す名詞または動詞、アンダーバーで区切る。すべて大文字					*/
/*グローバル関数	:(変数型)_内容を記す名詞または動詞、アンダーバーで区切る。すべて小文字			*/
/*																		*/
/*																		*/
/************************************************************************/

/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#ifdef __cplusplus
#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

/*　 =============================================================================
 *
 * インクルードファイル
 *
 *使用した機能初期化ファイル
 *hardware_infomation.c .h, timer_for_rx631.c .h
 *
 * =============================================================================　*/

#ifdef __cplusplus
extern "C" {
#include <stdio.h>
#include "iodefine.h"
#include "hardware_infomation_for_sugentem_n_52.h"
#include "timer_for_sugentem_n_52.h"
#include "sci.h"
#include <machine.h>
#include "motor_control.h"
#include "ad_converter.h"
#include "periodic_interrupt_function.h"
#include "spi.h"
#include "mode.h"
#include "adjust.h"
}
#endif

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void main(void);

#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

void main(void) {
	short mode_number = 0;
	io_initialize(); //ピン設定

	main_clock_initialize(); //クロック設定

	TPU1_initialize(); //タイマー割り込み設定(10kHz)

	SPI_init();
	delay(100);
	SCI1_initialize(115200);
	SCI_Str_println("");

	SCI_Value_println(SPI_Mpu6500_initialize(), 5);
	SCI_Str_println("Hello!");
	TPU0_initialize(); //タイマー割り込み設定(4kHz)
	motor_initialize();
	motor_driver_disable();

	ad_converter_initialize();

	update_battery_voltage();
	SCI_Str_print("sugentem_n_52: ");
	SCI_Value_print(battery_voltage(), 4);
	SCI_Str_println(" mV");
	delay(1000);
	if (battery_voltage() < BATTERY_LIMIT_VOLTAGE) { //バッテリー電圧監視用
		while (1) { //led警告表示(ledが行ったり来たり)
			LED1 = LED_ON;
			LED2 = LED_OFF;
			LED3 = LED_OFF;
			LED4 = LED_OFF;
			delay(100);
			LED1 = LED_OFF;
			LED2 = LED_ON;
			LED3 = LED_OFF;
			LED4 = LED_OFF;
			delay(100);
			LED1 = LED_OFF;
			LED2 = LED_OFF;
			LED3 = LED_ON;
			LED4 = LED_OFF;
			delay(100);
			LED1 = LED_OFF;
			LED2 = LED_OFF;
			LED3 = LED_OFF;
			LED4 = LED_ON;
			delay(100);
			LED1 = LED_OFF;
			LED2 = LED_OFF;
			LED3 = LED_ON;
			LED4 = LED_OFF;
			delay(100);
			LED1 = LED_OFF;
			LED2 = LED_ON;
			LED3 = LED_OFF;
			LED4 = LED_OFF;
			delay(100);
		}
	}
	while (1) { //ほんとのメイン文

		//　スイッチを押す
		if (SW_MODE == SW_ON) {
			// モードを更新
			mode_number = MODE_Mode_Change(+1, DISP);
			// スイッチの処理
			TIMER_Wait_Key_Off();
			disable_suction();
		}

		// 決定スイッチを押す
		if (AD_Sensor_Value_Get(FRONT_LEFT) > 600) {
			// スイッチの処理
			TIMER_Wait_Key_Off();
			// モードを実行
			mode_number = MODE_Mode_Change(0, EXEC);
			// モードを再表示
			mode_number = MODE_Mode_Change(0, DISP);
			// スイッチの処理
			TIMER_Wait_Key_Off();
			disable_suction();
		}
	}
}

#ifdef __cplusplus
void abort(void) {

}
#endif
