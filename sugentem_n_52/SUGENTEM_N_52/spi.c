/*
 * spi.c
 *
 *  Created on: 2017/09/28
 *      Author: 廣明
 */

/* ==================================================================== */
/* #includeファイル														*/
/* ==================================================================== */
#include <machine.h>
#include "iodefine.h"
#include "spi.h"
#include "sci.h"
#include "timer_for_sugentem_n_52.h"
#include "hardware_infomation_for_sugentem_n_52.h"
#include "adjust.h"

//#include <machine.h>
/* ==================================================================== */
/* プロトタイプ宣言															*/
/* ==================================================================== */
static uint16_t SPI_communicate(uint16_t data);
static uint16_t SPI_communicate8bit(uint16_t Data, uint8_t address,
		enum Spi_ssl status);
static uint32_t SPI_communicate_full(uint16_t data1, uint16_t address1,
		enum Spi_ssl status1, uint16_t data2, uint16_t address2,
		enum Spi_ssl status2);
static uint16_t SPI_communicate16bit(uint16_t data, uint16_t address,
		enum Spi_ssl status);
static void SPI_write(uint8_t adress, uint8_t data, enum Spi_ssl status);
static uint16_t SPI_read(uint8_t adress, enum Spi_ssl status);
static void valid_ssl(enum Spi_ssl state);
static void invalid_ssl(enum Spi_ssl state);
static void valid_spi(void);
static void invalid_spi(void);
/* ==================================================================== */
/* グローバル変数															*/
/* ==================================================================== */
//static RSPI_FIFO rx_fifo;
//static RSPI_FIFO tx_fifo;
volatile static char Spi_Receive_Flag = 0;

long Spi_log_timer[5];
/* ==================================================================== */
/* メインプログラム															*/
/* ==================================================================== */
void SPI_init(void) {

	PORTB.PDR.BIT.B0 = 1;			//CLK
	PORTC.PDR.BIT.B6 = 1;			//MOSI
	PORTC.PDR.BIT.B7 = 0;			//MISO

	PORTB.PMR.BIT.B0 = 1;			//CLK
	PORTC.PMR.BIT.B6 = 1;			//MOSI
	PORTC.PMR.BIT.B7 = 1;			//MISO

	//mpcの書き込み許可
	MPC.PWPR.BIT.B0WI = 0;			//PFSWEビット書き込み許可
	MPC.PWPR.BIT.PFSWE = 1;			//PFSレジスタへ書き込み許可

	//SPI端子設定
	MPC.PB0PFS.BIT.PSEL = 0x0D;		//RSPCLK
	MPC.PC6PFS.BIT.PSEL = 0x0D;		//MOSI
	MPC.PC7PFS.BIT.PSEL = 0x0D;		//MISO

	MPC.PWPR.BIT.PFSWE = 0;			//PFSレジスタへ書き込み禁止
	MPC.PWPR.BIT.B0WI = 1;			//PFSWEビット書き込み禁止

	PORTA.PDR.BIT.B4 = 1;	//CS: ENC1
	PORTA.PDR.BIT.B6 = 1;	//CS: ENC2
	PORTA.PDR.BIT.B3 = 1;	//CS: MPU

	MPU_SSL = 1;
	ENCR_SSL = 1;
	ENCL_SSL = 1;

// RX FIFO
//	rx_fifo.read = 0;
//	rx_fifo.write = 0;
//
//// TX FIFO
//	tx_fifo.read = 0;
//	tx_fifo.write = 0;

// Wake up RSPI0
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(RSPI0) = 0;
	SYSTEM.PRCR.WORD = 0xA500;

// Stop SPI
	RSPI0.SPCR.BIT.SPE = 0;

// MOSI idle set high
	RSPI0.SPPCR.BIT.MOIFE = 1;
	RSPI0.SPPCR.BIT.MOIFV = 1;

// set for bitrate
//	RSPI0.SPBR = 5;
	RSPI0.SPBR = 2;

	// SPDR is accessed in words
	RSPI0.SPDCR.BIT.SPLW = 1;

	//set sequence
	RSPI0.SPSCR.BIT.SPSLN = 4;
	//set frame
	RSPI0.SPDCR.BIT.SPFC = 4;

// SPCMD0 setting
//	RSPI0.SPCMD0.WORD = 0x078B;
//	RSPI0.SPCMD1.WORD = 0x0F0B;
//	RSPI0.SPCMD2.WORD = 0x0F09;
//	RSPI0.SPCMD3.WORD = 0x0F09;

	RSPI0.SPCMD0.WORD = 0x078F;
	RSPI0.SPCMD1.WORD = 0x0F0F;
	RSPI0.SPCMD2.WORD = 0x0F0D;
	RSPI0.SPCMD3.WORD = 0x0F0D;

	// priority RSPI0 interrupter
	IPR(RSPI0,)= INT_RPI1_LEVEL;

	// valid receive interrupt controller
	IEN(RSPI0, SPRI0)= 1;
	// valid transmit interrupt controller
	IEN(RSPI0, SPTI0)= 1;
	// valid error interrupt controller
//	IEN(RSPI0, SPII0)= 1;

	// permit error interrupter
	RSPI0.SPCR.BIT.SPEIE = 1;
	//ban transmit interrupter
	RSPI0.SPCR.BIT.SPTIE = 0;
	// permit receive interrupter
	RSPI0.SPCR.BIT.SPRIE = 1;
	// set to master mode
	RSPI0.SPCR.BIT.MSTR = 1;

	//dummy read
	uint16_t dummy = RSPI0.SPCR.BIT.MSTR;

	// valid RSPI Function
//	RSPI0.SPCR.BIT.SPE = 1;
}
/*******************************
 * MPU6500初期化
 * @return
 */
uint16_t SPI_Mpu6500_initialize(void) {

	// reset the device
	SPI_write( MPU6500_RA_PWR_MGMT_1, 0x80, MPU6500);

	delay(100);

	// reset gyro, accel, temp
	SPI_write( MPU6500_RA_SIGNAL_PATH_RESET, 0x05, MPU6500);

	delay(100);

	// set SPI mode by setting I2C_IF_DIS
	// reset DMP, FIFO, SIG
	SPI_write( MPU6500_RA_USER_CTRL, 0x10 | 0x8 | 0x4 | 0x1, MPU6500);
//	SPI_write( MPU6500_RA_USER_CTRL, 0x10, MPU6500);

	// who_am_i
	uint16_t receive = SPI_read( MPU6500_RA_WHO_AM_I, MPU6500);

	// gyro range
	SPI_write(MPU6500_RA_GYRO_CONFIG, MPU6500_GYRO_FS_2000, MPU6500);
	// accel range
	SPI_write(MPU6500_RA_ACCEL_CONFIG, MPU6500_ACCEL_FS_4, MPU6500);

	delay(30);

	RSPI0.SPCMD0.WORD = 0x0783;
	RSPI0.SPCMD1.WORD = 0x0F03;
	RSPI0.SPCMD2.WORD = 0x0F01;
	RSPI0.SPCMD3.WORD = 0x0F01;

	return receive;
}
/********************************************:
 * MPU6500からZ軸ジャイロの値を取得
 * @return
 */
int16_t SPI_Gyro_Zaxis_get(void) {
//	LED_numeric_lighting(1);
//	uint8_t address_H = MPU6500_RA_GYRO_ZOUT_H;
//	address_H |= 0x80;

//	uint16_t receive = SPI_communicate8bit(0x00, address_H, MPU6500);
//	LED_numeric_lighting(2);
//	receive = receive << 8;

//	uint8_t address_L = MPU6500_RA_GYRO_ZOUT_L;
//
//	uint16_t receive_L = SPI_communicate8bit(0x00, address_L, MPU6500);
//	receive |= receive_L;

	uint16_t receive = SPI_read( MPU6500_RA_GYRO_ZOUT_H, MPU6500);
	receive = receive << 8;
	uint16_t receive_L = SPI_read( MPU6500_RA_GYRO_ZOUT_L, MPU6500);
	receive |= receive_L;

	return receive;
}

/***********************************************
 * MPU6500からY軸加速度の値を取得
 * @return
 */
int16_t SPI_Accel_Yout_get(void) {

	uint8_t address = MPU6500_RA_ACCEL_YOUT_H;
	address |= 0x80;

	uint16_t receive = SPI_communicate8bit(0x00, address, MPU6500);

	return receive;
}
/***********************************************
 * MPU6500からY軸加速度の値を取得
 * @return
 */
int16_t SPI_Accel_Zout_get(void) {

	uint8_t address = MPU6500_RA_ACCEL_ZOUT_H;
	address |= 0x80;

	uint16_t receive = SPI_communicate8bit(0x00, address, MPU6500);

	return receive;
}
/********************************************:
 * AS5047からの絶対角度値の取得
 * @return
 */
uint16_t SPI_Encoder_R_Angle_get(void) {

	uint16_t address = 0x3FFF;
	address |= 0x400;
//
	uint16_t receive = SPI_communicate16bit(0x00, address, ENCR);

//	uint16_t receive = SPI_read( address, ENCR);

	return receive & 0x3FFF;
}
/********************************************:
 * AS5047からの絶対角度値の取得
 * @return
 */
uint16_t SPI_Encoder_L_Angle_get(void) {

	uint16_t address = 0x3FFF;
	address |= 0x400;
//
	uint16_t receive = SPI_communicate16bit(0x00, address, ENCL);
//	uint16_t receive = SPI_read( address, ENCL);
	return receive & 0x3FFF;
}
/***********************************
 *
 * @return
 * GyroA : 16-32bit
 * ENcoder1 : 0-15bit
 */
uint32_t SPI_GyroZ_and_Encoder_R_get(void) {

	uint16_t address1 = MPU6500_RA_GYRO_ZOUT_H;
	address1 |= 0x80;

	uint16_t address2 = 0x3FFF;
	address2 |= 0x400;

	uint32_t receive = SPI_communicate_full(0x00, address1, MPU6500, 0x00,
			address2, ENCR);

	return receive & 0xFFFF3FFF;
}
/**********************************
 *
 * @return
 * AccelY : 16-32bit
 * Encoder2 : 0-15bit
 */
uint32_t SPI_AccelY_and_Encoder_L_get(void) {

	uint16_t address1 = MPU6500_RA_ACCEL_YOUT_H;
	address1 |= 0x80;

	uint16_t address2 = 0x3FFF;
	address2 |= 0x400;

	uint32_t receive = SPI_communicate_full(0x00, address1, MPU6500, 0x00,
			address2, ENCL);

	return receive & 0xFFFF3FFF;
}

/************************************/
/*	受信割り込み						*/
/*********************************** */
void INT_RSPI0_SPRI0(void) {
//	LED_numeric_lighting(3);
	Spi_Receive_Flag = 1;

}

/***********************************
 *
 * @param Data
 * @return
 */
static uint16_t SPI_communicate(uint16_t data) {

	Spi_Receive_Flag = 0;
	// start communication

	// write send data
	RSPI0.SPDR.LONG = (uint32_t) data;
//	setpsw_i();
//	LED_numeric_lighting(4);
	//wait for end of communication
	SET_time_stamp();
	while (Spi_Receive_Flag == 0) {
		if (flag_time_out(SPI_TIME_OUT) == 1) {
			LED_numeric_lighting(10);
			break;
		}
	}
//	LED_numeric_lighting(1);
	// read receive data
	RSPI0.SPDCR.BIT.SPRDTD = 0;
	uint16_t receive = RSPI0.SPDR.LONG; // 受信

	return receive;
}
/*************************************
 * １モジュールSPI通信
 * @param Data
 * @return
 */
static uint16_t SPI_communicate8bit(uint16_t Data, uint8_t address,
		enum Spi_ssl status) {

	uint16_t Receive = 0;

	// start spi
	valid_spi();
//	LED_numeric_lighting(3);
	// valid SSL
	valid_ssl(status);
	TIMER_Wait_for(20);
	// send address
	SPI_communicate(address);

	TIMER_Wait_for(20);
	// send and receive data
	Receive = SPI_communicate(Data);

	TIMER_Wait_for(60);
	// invalid ssl
	invalid_ssl(status);

	// end spi
	invalid_spi();
//	LED_numeric_lighting(7);
	return Receive;

}
/*************************************
 * １モジュールSPI通信
 * @param data
 * @return
 * data1:32-16bit
 * data2:15-0bit
 *
 */

static uint32_t SPI_communicate_full(uint16_t data1, uint16_t address1,
		enum Spi_ssl status1, uint16_t data2, uint16_t address2,
		enum Spi_ssl status2) {

	uint32_t receive = 0;
//	Spi_log_timer[0] = TPU0.TCNT;
	// --------------- Start Spi--------------------------
	// start spi
	valid_spi();
	// --------------- Translation 1--------------------------
	// valid SSL
	valid_ssl(status1);

//	TIMER_Wait_for(20);

	// send Address
	SPI_communicate(address1);

	TIMER_Wait_for(5);

	// send and receive Data
	receive = (uint32_t) (SPI_communicate(data1) << 16);

	TIMER_Wait_for(5);

	// invalid ssl
	invalid_ssl(status1);

	// --------------- Translation 2--------------------------
	valid_ssl(status2);

	TIMER_Wait_for(5);

	SPI_communicate(address2);

	invalid_ssl(status2);

	TIMER_Wait_for(5);

	valid_ssl(status2);
	TIMER_Wait_for(5);

	receive |= SPI_communicate(data2);

	invalid_ssl(status2);
	// SPI end
	invalid_spi();
	return receive;

}
/*************************************
 * １モジュールSPI通信
 * @param Data
 * @return
 */
static uint16_t SPI_communicate16bit(uint16_t data, uint16_t address,
		enum Spi_ssl status) {

	uint16_t Receive = 0;

	valid_spi();

	SPI_communicate(0x00);
	SPI_communicate(0x00);

	valid_ssl(status);
	TIMER_Wait_for(50);

	SPI_communicate(address);

	TIMER_Wait_for(50);
	invalid_ssl(status);

	TIMER_Wait_for(50);

	valid_ssl(status);
	TIMER_Wait_for(50);

	Receive = SPI_communicate(data);

	TIMER_Wait_for(50);
	invalid_ssl(status);
	invalid_spi();

	return Receive;

}
/********************************
 * SPI通信8bit通信書き込み
 * @param Adress
 * @param Data
 * @param Status
 */
static void SPI_write(uint8_t adress, uint8_t data, enum Spi_ssl status) {

	SPI_communicate8bit((uint16_t) (data << 8), adress, status);
}
/*******************************
 * SPI通信8bit読み込み
 * @param Adress
 * @param Status
 * @return
 */
static uint16_t SPI_read(uint8_t adress, enum Spi_ssl status) {

	return SPI_communicate8bit(0x00, adress | 0x80, status) >> 8;
}
/**********************************
 *	SSL有効化
 * @param State
 * MPU6500
 * ENC1
 * ENC2
 */
static void valid_ssl(enum Spi_ssl state) {

	switch (state) {
	case MPU6500:
		MPU_SSL = 0;
		break;
	case ENCR:
		ENCR_SSL = 0;
		break;
	case ENCL:
		ENCL_SSL = 0;
		break;
	case NON:
		break;
	}
}
/**********************************
 *	SSL無効化
 * @param State
 * MPU6500
 * ENC1
 * ENC2
 */
static void invalid_ssl(enum Spi_ssl state) {
	switch (state) {
	case MPU6500:
		MPU_SSL = 1;
		break;
	case ENCR:
		ENCR_SSL = 1;
		break;
	case ENCL:
		ENCL_SSL = 1;
		break;
	case NON:
		break;
	}
}
/****************
 * SPI通信開始
 */
static void valid_spi(void) {

	RSPI0.SPCR.BIT.SPE = 1;

	while (RSPI0.SPCR.BIT.SPE == 0)
		;
//	LED_numeric_lighting(7);
}
/****************
 * SPI通信終了
 */
static void invalid_spi(void) {

	RSPI0.SPCR.BIT.SPE = 0;

	while (RSPI0.SPCR.BIT.SPE == 1)
		;

}
/* WAIT関数(微)						*/
/* --------------------------------	*/
void TIMER_Wait_for(unsigned long n) {

	volatile long i;

	for (i = 0; i < n; i++)
		;
}

