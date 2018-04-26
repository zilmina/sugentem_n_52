/*
 * periodic_interrupt_function.h
 *
 *  Created on: 2017/09/28
 *      Author: 廣明
 */

#ifndef PERIODIC_INTERRUPT_FUNCTION_H_
#define PERIODIC_INTERRUPT_FUNCTION_H_

void CMT1_initialize(void);
void TPU0_initialize(void);

void interrupt_function(void);
void enable_interrupt(void);
void disable_interrupt(void);
#define FLAG_CMT1_STERT CMT.CMSTR0.BIT.STR1//コンペアマッチタイマスタートレジスタ0
//モジュールの運用開始、停止
#define CMT1_PRESCALER CMT1.CMCR.BIT.CKS//分周器
#define CMT1_INTERRUPT_PERMISSION CMT1.CMCR.BIT.CMIE
#define CMT1_CMCOR CMT1.CMCOR


#endif /* PERIODIC_INTERRUPT_FUNCTION_H_ */
