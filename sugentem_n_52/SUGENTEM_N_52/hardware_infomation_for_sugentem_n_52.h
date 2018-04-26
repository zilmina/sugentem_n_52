/*
 * hardware_infomation_for_sugentem_n_52.h
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

#ifndef HARDWARE_INFOMATION_FOR_SUGENTEM_N_52_H_
#define HARDWARE_INFOMATION_FOR_SUGENTEM_N_52_H_

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void io_initialize(void);
void main_clock_initialize(void);
int test(void);
void LED_numeric_lighting(char temp);
/*　 =============================================================================
 *
 * マクロ定義
 *
 * =============================================================================　*/
#define LED_ON 1
#define LED_OFF 0
//表示用LED
#define LED1 PORT1.PODR.BIT.B4
#define LED2 PORT1.PODR.BIT.B5
#define LED3 PORT2.PODR.BIT.B7
#define LED4 PORT3.PODR.BIT.B1

#define SW_MODE PORT3.PIDR.BIT.B5

#define SW_ON 1
#define SW_OFF 0


#define SUCTION PORTB.PODR.BIT.B5

#endif /* HARDWARE_INFOMATION_FOR_SUGENTEM_N_52_H_ */
