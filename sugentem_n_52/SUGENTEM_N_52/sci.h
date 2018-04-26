/*
 * sci.h
 *
 *  Created on: 2017/09/25
 *      Author: 廣明
 */

#ifndef SCI_H_
#define SCI_H_

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void SCI1_initialize(unsigned long);
void INT_SCI1_RXI1(void);
void INT_SCI1_TXI1(void);
void INT_SCI1_ERI1(void);
void INT_SCI1_TEI1(void);

void SCI_Str_print(char *str);
void SCI_Str_println(char *str);
void SCI_Value_print(long value, char n);
void SCI_Value_println(long value, char n);
void SCI_Value0x_print(long value, char n);

char SCI_Str_readline(char *data, char *sentence, const char *endline);
/*　 =============================================================================
 *
 * マクロ定義(大文字)
 *
 * =============================================================================　*/
#define PCLK 50//MHz
#define SCI_INT_LEVEL 10
#define BUFFER_SIZE			1024

// SCI1
struct Sci_fifo {
	// 読み取り
	short read;
	// 書き込み
	short write;
	// データ格納
	unsigned char buff[BUFFER_SIZE];

};

#endif /* SCI_H_ */
