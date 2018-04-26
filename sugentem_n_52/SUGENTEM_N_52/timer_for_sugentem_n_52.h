/*
 * timer_for_sugentem_n_52.h
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

#ifndef TIMER_FOR_SUGENTEM_N_52_H_
#define TIMER_FOR_SUGENTEM_N_52_H_

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void CMT0_initialize(void);
void TPU1_initialize(void);
void timer_countup(void);
void delay(unsigned int);
void TIMER_Wait_Key_Off(void);

void SET_time_stamp(void);
char flag_time_out(unsigned int temp_100us);
/*　 =============================================================================
 *
 * マクロ定義(大文字)
 *
 * =============================================================================　*/

#define FLAG_CMT0_STERT CMT.CMSTR0.BIT.STR0//コンペアマッチタイマスタートレジスタ0
//モジュールの運用開始、停止
#define CMT0_PRESCALER CMT0.CMCR.BIT.CKS//分周器
#define CMT0_INTERRUPT_PERMISSION CMT0.CMCR.BIT.CMIE
#define CMT0_CMCOR CMT0.CMCOR

#endif /* TIMER_FOR_SUGENTEM_N_52_H_ */
