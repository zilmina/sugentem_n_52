/*
 * timer_for_sugentem_n_52.c
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

/*==============================================================*/
/*このソースコードはRX631用のCMT0モジュールを操作して							*/
/*周期割り込み、　遅延、												*/
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

/*　 =============================================================================
 *
 * インクルードファイル
 *
 * =============================================================================　*/
#include "iodefine.h"

#include "hardware_infomation_for_sugentem_n_52.h"
#include "timer_for_sugentem_n_52.h"

static volatile unsigned int runtime_raw; //staticはこのファイルだけのグローバル関数であることを示す
static volatile unsigned int runtime_ms; //staticはこのファイルだけのグローバル関数であることを示す
static volatile unsigned int runtime_s; //staticはこのファイルだけのグローバル関数であることを示す

static volatile unsigned int time_stamp = 0;

///functions
void CMT0_initialize(void) {
	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
	MSTP(CMT0) = 0;				// Wake up CMT0//CMT0の運用開始
	SYSTEM.PRCR.WORD = 0xa500;/*クロックソース選択の保護*/

	CMT.CMSTR0.BIT.STR0 = 0;	// Disable CMT0 count
	CMT0_INTERRUPT_PERMISSION = 0;	//CMI0(コンペアマッチ割り込み)禁止

	CMT0_PRESCALER = 0;	//8分周
	CMT0.CMCNT = 0;                         //カウンタのリセット
	CMT0_CMCOR = 625 - 1;  //カウントが4999で割り込み 50MHz/8/625 = 10kHz すなはち 10us周期の割り込み

	CMT0_INTERRUPT_PERMISSION = 1;                         //CMI0(コンペアマッチ割り込み)許可
	CMT.CMSTR0.BIT.STR0 = 1;	// Enable CMT0 count

	IEN(CMT0,CMI0)= 1;                     //割り込みを許可する
	IPR(CMT0,CMI0)= 6;                     //CMI0の割り込み優先度6を設定(MAX15,MIN1)

	runtime_raw = 0;
}

void TPU1_initialize(void) {
	SYSTEM.PRCR.WORD = 0xa50b; /*クロックソース選択の保護の解除*/
	MSTP(TPU1) = 0;				// Wake up TPU1//TPU1の運用開始
	SYSTEM.PRCR.WORD = 0xa500; /*クロックソース選択の保護*/

	TPUA.TSTR.BIT.CST1 = 0;		//クロックカウント停止
	TPU1.TCR.BIT.TPSC = 2;		//タイマプリスケーラ選択：PCLK/16
	TPU1.TCR.BIT.CKEG = 0;		//クロックカウント要因：内部クロック（立ち下がり）
	TPU1.TMDR.BIT.MD = 0;		// 通常モード
	TPU1.TCR.BIT.CCLR = 1;  	// tcnt = tgra でカウントクリア
	TPU1.TIER.BYTE = 0;			// TPU1.TGRA,B,C,Dの割り込み禁止
	TPU1.TCNT = 0;
	TPU1.TGRA = 313 - 1;		//50MHz/16/313 = 10kHz

	IPR(TPU1,TGI1A)= 1;         //TPU1_TGI01の割り込み優先度1を設定(MAX15,MIN1)
	IEN(TPU1,TGI1A)= 1;			//割り込みを許可する

	TPU1.TIER.BIT.TGIEA = 1;	//
	TPUA.TSTR.BIT.CST1 = 1;	//クロックカウント開始

}

void timer_countup(void) {
	runtime_raw++;
	runtime_ms = (int) (runtime_raw / 10);
	runtime_s = (int) (runtime_ms / 1000);
}

void delay(volatile unsigned int temp_ms) {
	unsigned int time_stamp_start = runtime_raw;
	while ((time_stamp_start + temp_ms * 10) > runtime_raw) {

	}
}

/* タクトスイッチWAIT関数					*/
/* --------------------------------	*/
void TIMER_Wait_Key_Off(void) {

	delay(100); /*100msecまつ					*/
	while (SW_MODE == SW_ON)
		;
	delay(100); /*100msecまつ					*/
	while (SW_MODE == SW_ON)
		;
}

void SET_time_stamp(void){
	time_stamp = runtime_raw;
}

char flag_time_out(unsigned int temp_100us) {
	char ans = 0;
	if(runtime_raw -time_stamp > temp_100us){
		ans = 1;
	}
	return ans;
}

