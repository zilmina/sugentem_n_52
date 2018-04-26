/*
 * hardware_infomation_for_sugentem_n_52.c
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

/****************************************************************/
/*include Files															*/
/****************************************************************/
#include "hardware_infomation_for_sugentem_n_52.h"
#include "iodefine.h"

/****************************************************************/
/*functions															*/
/****************************************************************/
int test(void){
	return 1;
}
///-汎用ピンの設定--------------------------------------------------
void io_initialize(void){
	PORT1.PDR.BIT.B4 = 1;	//P14を出力と定義
	PORT1.PODR.BIT.B4 = 0;	//P14をLOWに

	PORT1.PDR.BIT.B5 = 1;	//P15を出力と定義
	PORT1.PODR.BIT.B5 = 0;	//P15をLOWに

	PORT2.PDR.BIT.B7 = 1;	//P27を出力と定義
	PORT2.PODR.BIT.B7 = 0;	//P27をLOWに

	PORT3.PDR.BIT.B1 = 1;	//P31を出力と定義
	PORT3.PODR.BIT.B1 = 0;	//P31をLOWに

	PORTB.PDR.BIT.B5 = 1;	//P31を出力と定義
	PORTB.PODR.BIT.B5 = 0;	//P31をLOWに

//	PORT3.PDR.BIT.B5 = 0;	//P35は入力専用（これはコメントアウトを外すと機能しないバグとなる）

}

///-クロック初期化関数--------------------------------------------------
void main_clock_initialize(void){

//	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
//	SYSTEM.PLLCR.WORD = 0x0F00; /*PLL 逓倍×16 入力1分周 (12.000MHz * 16 = 192MHz)*/
//	SYSTEM.PLLCR2.BYTE = 0x00; /*PLL ENABLE */
//	SYSTEM.SCKCR.LONG = 0x21C21211; /*FCK1/4 ICK1/2 BCLK停止 SDCLK停止 BCK1/4 PCLKA1/2 PCLKB1/4*/
//	SYSTEM.SCKCR2.WORD = 0x0032; /*UCLK1/4 IEBCK1/4 */
//	SYSTEM.BCKCR.BYTE = 0x01; /*BCLK = 1/2 */
//	SYSTEM.SCKCR3.WORD = 0x0400; /*PLL回路選択*/
//	SYSTEM.PRCR.WORD = 0xa500; /*クロックソース選択の保護*/


	//高速オンチップオシレータ（HOCO)を使用する
	//PLL(Phase Locked Loop:発振器からの周波数を逓倍)の動作周波数はMAX:200MHz
	//動作周期は100MHz
	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
	SYSTEM.PLLCR.WORD = 0x1300;
	/* PLL回路 20逓倍× 入力1分周 (10.000MHz * 20 = 200MHz)
	 * 0b00 010011 000000 00 = 0x1300*/
	SYSTEM.PLLCR2.BYTE = 0x00; /*PLL ENABLE */

	SYSTEM.SCKCR.LONG = 0x21C12211;
	/* BCLK端子出力1/4(MAX:50MHz)(BCLKとBCLK端子出力は別)0010b
	 * PCLKB1/4(MAX:50MHz)0010b
	 * PCLKA=ICLK1/2(MAX:100MHz)0001b
	 * SDCLK停止 1
	 * BCLK停止 1
	 * ICK1/2(MAX:100MHz) 0001b
	 * FCK1/4(MAX:50MHz) 0010b
	 * 0b 0010 0001 1 1 00 0001 0010 0010 0001 0001 = 0x21C12211
	 * */

	SYSTEM.SCKCR2.WORD = 0x0012;
	/* IEBCK1/4(MAX:50MHz)0010b
	 * UCLK1/4(使わない場合0001b)
	 * 00000000 0001 0010b = 0x0012  */

	SYSTEM.BCKCR.BYTE = 0x01;
	/*BCLK = (1/2) * PLL(MAX:100MHz) */

	SYSTEM.SCKCR3.WORD = 0x0400;
	/* クロックソース選択：PLL回路選択100
	 * 00000 100 00000000b = 0x0400*/

	SYSTEM.PRCR.WORD = 0xa500;
	/*クロックソース選択の保護*/
}

void LED_numeric_lighting(char temp){
	LED1 = LED_OFF;
	LED2 = LED_OFF;
	LED3 = LED_OFF;
	LED4 = LED_OFF;

	if( (temp&1) == 1){
		LED1 = LED_ON;
	}
	if( (temp&2) == 2){
		LED2 = LED_ON;
	}
	if( (temp&4) == 4){
		LED3 = LED_ON;
	}
	if( (temp&8) == 8){
		LED4 = LED_ON;
	}
}

