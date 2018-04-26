/*
 * ad_converter.h
 *
 *  Created on: 2017/09/27
 *      Author: 廣明
 */

#ifndef AD_CONVERTER_H_
#define AD_CONVERTER_H_

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void ad_converter_initialize(void);
void update_battery_voltage(void);
void update_front_sensor_voltage(void);
void update_side_sensor_voltage(void);

short battery_voltage(void);
short AD_Sensor_Value_Get(enum Opto_dir status);
float AD_Sensor_Distance_Get(enum Opto_dir status);
float AD_Line_Error_Get(void);
float AD_front_Error_Get(void);
void AD_Line_Difference_Print(void);
void AD_Line_Distance_Print(void);
enum Wall_decide AD_Wall_Look(enum Wall_dir dir);
enum Wall_decide AD_Wall_ref_Look(enum Wall_dir dir);

void TIMER_Wait_For_Loop(unsigned long n);
/*　 =============================================================================
 *
 * マクロ定義(大文字)
 *
 * =============================================================================　*/
#define ANALOG_PORT_AD_BATTERY 0x800			//AD011
#define ANALOG_PORT_LED_LEFT_SIDE 0x200			//AD009
#define ANALOG_PORT_LED_LEFT_FRONT 0x04		//AD002
#define ANALOG_PORT_LED_RIGHT_FRONT 0x02		//AD001
#define ANALOG_PORT_LED_RIGHT_SIDE 0x01		//AD000

#define LED_FRONT PORTB.PODR.BIT.B1
#define LED_SIDE PORTB.PODR.BIT.B3

#define AD_LED_ON 1
#define AD_LED_OFF 0

enum Wall_dir {
	WALL_PRE_FORWARD, WALL_FORWARD, WALL_RIGHT, WALL_BACK, WALL_LEFT
};

enum Opto_dir {
	SIDE_RIGHT, SIDE_LEFT, FRONT_RIGHT, FRONT_LEFT
};
enum Wall_decide {
	WALL_NON = 0, WALL_EXIT
};

#define AD_CONVERT_INT_LEVEL		8

/*　 =============================================================================
 *
 * 構造体
 *
 * =============================================================================　*/
//mouse
struct Machine_sensor {

	//　バッテリー電圧(raw)
	short raw_battery_voltage;
	//　バッテリー電圧(mV)
	short battery_voltage;

	//left side voltage(raw)
	float left_side;
	//　目標加速度(mm/s^2)
	float target_accel;
	//現在の角度
	float deg;
	//現在の角速度(deg/s)
	float deg_speed_now;
	//目標角速度(deg/s)
	float target_speed_deg;
	//目標角加速度(deg/s^2)
	float target_deg_accel;
	// 機体の制御モード
	// 0:横壁制御なし　1:横壁制御あり
	char control_mode;
	// （進んだ距離）
	volatile long distance_mm;

	//　右モータ構造体を作成
	struct {
		// duty rate of motor(+:forward, -:backward)
		volatile float duty;
	} r_motor;

	//　左モータ構造体を作成
	struct {
		// duty rate of motor(+:forward, -:backward)
		volatile float duty;
	} l_motor;

	// 吸引用モーター構造体を作成
	struct {
		// duty rate of motor(+:forward, -:ban)
		float duty;
	} suction_motor;
};
/* ======	光センサ  ==========*/
struct Opt_sensor {

	// LEDスイッチ
	char flag;

	//　右センサ構造体を作成
	struct {
		//　暗センサ値(raw)
		short dark_raw_value;
		//　明センサ値(raw)
		short bright_raw_value;
		//センサ差分
		short difference_raw_value;
		//　センサリファレンス値
		long refer;
		//　壁有無の閾値
		short lim;
		//　壁有無の閾値（長距離）
		short pre_lim;
	} right;

	//　左センサ構造体を作成
	struct {
		//　暗センサ値(raw)
		long dark_raw_value;
		//　明センサ値(raw)
		long bright_raw_value;
		//センサ差分
		short difference_raw_value;
		//　センサリファレンス値
		long refer;
		//　壁有無の閾値
		short lim;
		//　壁有無の閾値（長距離）
		short pre_lim;
	} left;
};
#endif /* AD_CONVERTER_H_ */
