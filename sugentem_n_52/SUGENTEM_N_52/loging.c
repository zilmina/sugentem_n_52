/*
 * loging.c
 *
 *  Created on: 2017/10/31
 *      Author: 廣明
 */

#include "loging.h"
#include "adjust.h"
#include "sci.h"
#include "timer_for_sugentem_n_52.h"
#include "hardware_infomation_for_sugentem_n_52.h"
//=============================//

//			グローバル変数

//=============================//
volatile static char flag_logging;

unsigned int top_ring_buffer = 0;
unsigned int bottom_ring_buffer = 0;

short log_data[LOG_MAX] = { 0 };

void logging(int temp) {

	if (flag_logging == 1) {

		top_ring_buffer = bottom_ring_buffer;

		log_data[top_ring_buffer] = temp;

		if (bottom_ring_buffer == LOG_MAX) {
			bottom_ring_buffer = 0;
		} else {
			bottom_ring_buffer++;
		}
	}
}

void print_log_data(void) {
	int i;
	i = bottom_ring_buffer;
	while (i != top_ring_buffer) {
		if ((i % LOG_ITEM) == 0) {
			SCI_Str_print("\r\n");//項目数で割れたら改行
		}
		SCI_Value_print(log_data[i], 8);
		delay(1);
		i++;
		if (i >= LOG_MAX) {
			i = 0;
		}
	}
	LED_numeric_lighting(15);
	delay(1000);
	LED_numeric_lighting(0);
}

void enable_flag_logging(void) {
	flag_logging = 1;
}
void disable_flag_logging(void) {
	flag_logging = 0;
}
