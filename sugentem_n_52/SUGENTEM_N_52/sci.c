/*
 * sci.c
 *
 *  Created on: 2017/09/25
 *      Author: 廣明
 */
#include "sci.h"
#include <math.h>
#include "iodefine.h"
/*==============================================================*/
/*このソースコードはRX631用のSCI1モジュールを操作して							*/
/*printf, println,												*/
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
/* ==================================================================== */
/* プロトタイプ宣言															*/
/* ==================================================================== */

static short SCI_data_putc(unsigned char);
/* ==================================================================== */
/* グローバル変数															*/
/* ==================================================================== */

static struct Sci_fifo rx_fifo;
static struct Sci_fifo tx_fifo;
/* ==================================================================== */
/* SCI初期化関数															*/
/* ==================================================================== */

void SCI1_initialize(unsigned long bps){
	// Set MPC
	PORT2.PMR.BIT.B6 = 1;			// P26: peripheral
	PORT3.PMR.BIT.B0 = 1;			// P30: peripheral

	MPC.PWPR.BIT.B0WI = 0;			// Release protect
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		// Set P26: TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		// Set P30: RXD1
	MPC.PWPR.BIT.PFSWE = 0;			// Protect
	MPC.PWPR.BIT.B0WI = 1;


	unsigned char smr;
	unsigned char brr;
	int i;

	// RX FIFO
	rx_fifo.read = 0;
	rx_fifo.write = 0;

	// TX FIFO
	tx_fifo.read = 0;
	tx_fifo.write = 0;

	// Serial Mode Register
	smr = 0;

	// Bit Rate Register
	brr = (PCLK * 1000 * 1000 / (32 * bps)) - 1;

	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(SCI1) = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	SCI1.SCR.BYTE = 0x00;
	SCI1.SMR.BYTE = smr;
	SCI1.BRR = brr;

	for (i = 0; i < bps + 100; i++)
		;

	IEN(SCI1, TXI1)= 1;
	IEN(SCI1, RXI1)= 1;
	IEN(SCI1, TEI1)= 1;
//	IEN(SCI1, ERI1)= 1;

	IPR(SCI1,)= SCI_INT_LEVEL; //割り込みレベ10

	SCI1.SCR.BYTE |= 0x70;

//	// SCI1
//	SYSTEM.PRCR.WORD = 0xA502;		// Release protect
//	MSTP(SCI1) = 0;					// Wake up SCI1
//	SYSTEM.PRCR.WORD = 0xA500;		// Protect
//	// Set MPC
//	PORT2.PMR.BIT.B6 = 1;			// P26: peripheral
//	PORT3.PMR.BIT.B0 = 1;			// P30: peripheral
//
//	MPC.PWPR.BIT.B0WI = 0;			// Release protect
//	MPC.PWPR.BIT.PFSWE = 1;
//	MPC.P26PFS.BIT.PSEL = 0x0A;		// Set P26: TXD1
//	MPC.P30PFS.BIT.PSEL = 0x0A;		// Set P30: RXD1
//	MPC.PWPR.BIT.PFSWE = 0;			// Protect
//	MPC.PWPR.BIT.B0WI = 1;
//
//	// SCI1 Settings
//	SCI1.SCR.BIT.TEIE = 0;			// Disable TEIE
//	SCI1.SCR.BIT.MPIE = 0;			// Normal Mode
//	SCI1.SCR.BIT.RE = 0;			// Disable RX
//	SCI1.SCR.BIT.TE = 0;			// Disable TX
//	SCI1.SCR.BIT.RIE = 0;			// Disable RXI, ERI
//	SCI1.SCR.BIT.TIE = 0;			// Disable TXI
//	SCI1.SCR.BIT.CKE = 0;			// built in baud rate
//	SCI1.SMR.BIT.CKS = 0;			// PCLK
//	SCI1.SMR.BIT.MP= 0;				// Disable multiple processor
//	SCI1.SMR.BIT.STOP = 0;			// 1 stop bit
//	SCI1.SMR.BIT.PM = 0;			// none parity
//	SCI1.SMR.BIT.CHR = 0;			// 8bit data length
//	SCI1.SMR.BIT.CM = 0;			// 調歩同期式
//	SCI1.SEMR.BIT.ABCS = 0;			//　基本クロック16サイクルの期間が1ビット期間の転送レート
//	SCI1.SEMR.BIT.NFEN = 1;			// RXDn入力信号のノイズ除去機能有効
//	SCI1.BRR = 13;//(char)(PCLK * 1000 * 1000)/ (64 * 2^(-1) * bps) -1 ;
//	SCI1.SCR.BIT.RE = 1;			// Enable RX
//	SCI1.SCR.BIT.TE = 1;			// Enable TX
//	SCI1.SCR.BIT.RIE = 1;			// Enable RXI, ERI
//	SCI1.SCR.BIT.TIE = 1;			// Enable TXI
//
//	IPR(SCI1, )= INT_SCI_LEVEL;
}

/************************************/
/*	受信割り込み						*/
/*********************************** */
void INT_SCI1_RXI1(void) {
	unsigned char data;
	short wp;

	data = SCI1.RDR; // 受信

	wp = rx_fifo.write + 1;
	if (wp >= BUFFER_SIZE) {
		wp = 0;
	}

	if (wp != rx_fifo.read) {
		rx_fifo.buff[rx_fifo.write] = data;
		rx_fifo.write = wp;
	}
}

/************************************/
/* 	送信割り込み					*/
/***********************************/
void INT_SCI1_TXI1(void) {
	unsigned char data;

	if (tx_fifo.read != tx_fifo.write) {
		data = tx_fifo.buff[tx_fifo.read];
		tx_fifo.read++;

		if ( BUFFER_SIZE <= tx_fifo.read) {
			tx_fifo.read = 0;
		}
		SCI1.TDR = data;

	} else {
		SCI1.SCR.BYTE &= ~0x80; // 送信割り込み禁止
	}
}
//
///************************************/
///*	受信エラー割り込み				*/
///***********************************/
//void INT_SCI1_ERI1(void) {
//	SCI1.SSR.BYTE &= ~0x38;
//}

/************************************/
/*	送信終了割り込み				*/
/***********************************/
void INT_SCI1_TEI1(void) {
	SCI1.SSR.BYTE &= ~0x04;
}

/* 文字列系															*/
/* ================================================================ */

/*********************************
 * 文字列送信
 * @param str
 */
void SCI_Str_print(char *str) {
	while (*str != '\0') {               	// 文字が\0になるまで繰り返す
		SCI_data_putc(*str);               // 1文字送信
		str++;                        // 次の文字に移る
	}
}
/*********************************
 * 文字列送信
 * @param str
 */
void SCI_Str_println(char *str) {

	SCI_Str_print(str);
	SCI_Str_print("\r\n");
}
/*********************
 SCI1 10進数字送信
 引数：数字,桁数
 返値：なし
 longまでの数
 **********************/
void SCI_Value_print(long value, char n) {

	char buf[16] = { 0 }, flag = 0;

	for (char i = 0; i < 15; i++)
		buf[i] = ' ';

	if (value < 0) {
		flag = 1;
		value = -value;
	}

	char i = 0;
	do {		//とりあえず下のケタから埋めていく
		buf[i] = ((value % 10) | '0');
		i++;
		if (i > 15)
			value = 0;
		value /= 10;	//その桁を1の位に持ってくる

	} while (value != 0);

	if (flag == 1)
		buf[i] = '-';

	while (n > 0) {			//bufの中身を後ろから出力
		SCI_data_putc(buf[n - 1]);
		n--;
	}
}
/*********************
 SCI1 10進数字送信
 引数：数字,桁数
 返値：なし
 longまでの数
 **********************/
void SCI_Value_println(long value, char n) {
	SCI_Value_print(value, n);
	SCI_Str_print("\r\n");
}
/*********************
 SCI1 16進数字送信
 引数：数字,桁数
 返値：なし
 longまでの数
 **********************/
void SCI_Value0x_print(long value, char n) {

	char buf[16] = { 0 }, flag = 0;

	for (char i = 0; i < 15; i++)
		buf[i] = ' ';

	if (value < 0) {
		flag = 1;
		value = -value;
	}

	char i = 0;
	do {		//とりあえず下のケタから埋めていく
		if (value % 16 > 9) {
			buf[i] = (((value % 16) - 9) | 0x60);
		} else {
			buf[i] = ((value % 16) | '0');
		}
		i++;
		if (i > 15)
			value = 0;
		value /= 10;	//その桁を1の位に持ってくる

	} while (value != 0);

	if (flag == 1)
		buf[i] = '-';

	while (n > 0) {			//bufの中身を後ろから出力
		SCI_data_putc(buf[n - 1]);
		n--;
	}
}

/***********************************
 1文字格納
 ***********************************/
static short SCI_data_putc(unsigned char data) {
	short ret = 0;
	short wp = 0;

	ret = -1;

	wp = tx_fifo.write + 1;
	if (wp >= BUFFER_SIZE) {
		wp = 0;
	}

	if (wp != tx_fifo.read) {
		tx_fifo.buff[tx_fifo.write] = data;
		tx_fifo.write = wp;
		ret = data;

		if ((SCI1.SCR.BYTE & 0x80) != 0x80) {  //送信割り込み禁止の時
			data = tx_fifo.buff[tx_fifo.read];
			tx_fifo.read++;

			if ( BUFFER_SIZE <= tx_fifo.read) {
				tx_fifo.read = 0;
			}
			SCI1.SCR.BYTE |= 0x80; /* 送信割込み許可 */
			SCI1.TDR = data;
		}
	}

	return (ret);
}

/***********************************
 1文字取得	0
 ************************************/
static short SCI_data_getc(void) {
	short data;

	data = -1;

	if (rx_fifo.read != rx_fifo.write) {
		data = rx_fifo.buff[rx_fifo.read];
		rx_fifo.read++;
		if (BUFFER_SIZE <= rx_fifo.read) {
			rx_fifo.read = 0;
		}
	}

	return (data);
}

/************************************
 *	指定の文字まで文字列取得
 * Data:取得データ
 * Sentence:前のEndlineから次のEndlineまでのデータ
 * Endline:取得したいデータの区切り1byte
 * return:区切れたかどうか
 *
 ************************************/
char SCI_Str_readline(char *data, char *sentence, const char *endline) {

	static short num = -1;
	short n = 0, flag = 0;

	while (rx_fifo.read != rx_fifo.write) {  // データが格納されている
		*data = rx_fifo.buff[rx_fifo.read];		// データを読み込みむ

		// 読み込んだデータが区切りデータの場合
		if (*data == *endline) {
			n = num + 1;	//　全開の区切りデータの１つ後の番号
			if (n >= BUFFER_SIZE) {
				n = 0;
			}
			// 区切りデータ間のデータをSentenceに格納
			while (n != rx_fifo.read) {
				*sentence = rx_fifo.buff[n];
				n++;
				if (n >= BUFFER_SIZE) {
					n = 0;
				}
				sentence++;
			}
			num = rx_fifo.read;
			flag = 1;
		}
		rx_fifo.read++;
		data++;
		if (BUFFER_SIZE <= rx_fifo.read) {
			rx_fifo.read = 0;
		}
	}
	*data = '\0';
	*sentence = '\0';
	return flag;
}

