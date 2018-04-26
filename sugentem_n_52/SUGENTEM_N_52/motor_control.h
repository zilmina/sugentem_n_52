/*
 * motor_control.h
 *
 *  Created on: 2017/09/17
 *      Author: 廣明
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_
/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void motor_initialize(void);
void motor_power_change(void);

void duty_override(float temp_duty, enum Motor temp);
void change_duty(float temp_duty, enum Motor temp);

void calculate_duty_with_pid(void);

void motor_driver_enable(void);
void motor_driver_disable(void);

void update_gyro(void);
void update_encoder(void);
void update_motor_parameters(void);
void update_velocity_parameters(void);
void update_angular_velocity_parameters(void);

int SPI_Gyro_Zaxis_Offset_get(void);

float check_body_velocity_parameters(enum Body_Velocity_St temp);
float check_body_angular_velocity_parameters(enum Body_Angular_Velocity_St temp);
float check_r_motor_parameters(enum Motor_St temp);
float check_r_encoder_parameters(enum Encoder_St temp);
float check_l_motor_parameters(enum Motor_St temp);
float check_l_encoder_parameters(enum Encoder_St temp);

void change_body_velocity_parameters(float temp_num,
		enum Body_Velocity_St temp_mode,
		enum Numetric_Operator_St temp_operator);
void change_body_angular_velocity_parameters(float temp_num,
		enum Body_Angular_Velocity_St temp_mode,
		enum Numetric_Operator_St temp_operator);
void change_r_motor_parameters(float temp_num, enum Motor_St temp_mode);
void change_l_motor_parameters(float temp_num, enum Motor_St temp_mode);

void initialize_distance(void);
void initialize_all_pid(void);
void initialize_led_pid(void);
void initialize_angle_pid(void);
void initialize_velocity_pid(void);
char request_move_mm(long temp_mm, float start_speed, float last_speed,
		float max_speed, float accel, enum Run_Control control);
enum Run_St CONTROL_Distance_Watch(volatile long distance,
		enum Run_Control control);
enum Run_St CONTROL_Degree_Watch(volatile long temp_deg,
		enum Run_Control control);
char request_slalom(long temp_mm, float start_speed, float last_speed,
		float max_angular_velocity, float accel);

void SET_side_WALL_CONTROL(enum Run_Control temp);
void SET_front_WALL_CONTROL(enum Run_Control temp);

float check_angular_velocity_PID(enum Gain temp);
float check_velocity_PID(enum Gain temp);
float check_side_led_PID(enum Gain temp);

void enable_suction(void);
void disable_suction(void);

void enable_pid_calculate(void);
void disable_pid_calculate(void);

void change_P_gain_mode(char temp);
/*　 =============================================================================
 *
 * マクロ定義(大文字)
 *
 * =============================================================================　*/
#define period 500

#define R_MOTOR PORTE.PODR.BIT.B2
#define L_MOTOR PORTE.PODR.BIT.B4

#define R_FORWARD 1
#define R_BACKWARD 0
#define L_FORWARD 0
#define L_BACKWARD 1

enum Gain {
	P_GAIN, I_GAIN, D_GAIN
};

enum Motor {
	r_motor, l_motor
};
enum Com_Turn {
	TURN90R, TURN90L, TURN180
};
enum Machine_Dir {
	MACHINE_FORWARD, MACHINE_RIGHT, MACHINE_BACK, MACHINE_LEFT
};
enum Motor_Dir {
	MOTOR_BACK = 0, MOTOR_AHEAD = 1
};
enum Numetric_Operator_St {
	SUBSTITUTION, ADDITION, SUBSTRACTION, MULTIPLICATION, DIVITION
};
enum Body_Velocity_St {
	DISTANCE,
	NOW_VELOCITY,
	REQUEST_VELOCITY,
	TARGET_VELOCITY,
	NOW_ACCEL,
	TARGET_ACCEL
};
enum Body_Angular_Velocity_St {
	ANGLE_GAIN,
	DEG,
	TARGET_DEG,
	NOW_ANGULAR_VELOCITY,
	REQUEST_ANGULAR_VELOCITY,
	TARGET_ANGULAR_VELOCITY,
	NOW_ANGULAR_ACCEL,
	TARGET_ANGULAR_ACCEL
};
enum Motor_St {
	DUTY,
	MOTOR_DEG,
	MOTOR_ANGULAR_VELOCITY,
	MOTOR_DISTANCE,
	MOTOR_VELOCITY,
	MOTOR_REQUEST_VELOCITY,
	MOTOR_TARGET_VELOCITY
};
enum Encoder_St {
	VALUE, RAW
};
enum Run_St {
	RUNNING, FINISH
};
enum Observe_St {
	OBSERVE_OFF, OBSERVE_ON
};
enum Run_Control {
	CONTROL_OFF, CONTROL_ON
};
enum Substract_Motor_timer {
	SUBSTRACT_OFF, SUBSTRACT_ON
};

/*　 =============================================================================
 *
 * 構造体
 *
 * =============================================================================　*/
//mouse
struct Machine {

	//pid計算カットオフフラグ(0:計算しない,1:計算する)
	char flag_pid_calculate;
	//フェイルセーフflag
	char flag_fail_safe;
	//led PIDリセット用flag
	char flag_reset_led_PID;
	//angle PIDリセット用flag
	char flag_reset_angle_PID;
	//velocity PIDリセット用flag
	char flag_reset_velocity_PID;
	//GYRO補正フラグ
	char flag_GYRO_CONTROL;
	//壁センサー補正フラグ
	char flag_side_WALL_CONTROL;
	//前壁センサー補正フラグ
	char flag_front_WALL_CONTROL;

	//横壁速度P切替(0:LOW, 1:2000mm/s)
	char mode_side_led_P_gain;

	//横壁センサーPIDゲイン
	float led_side_gain;
	//前壁センサーPIDゲイン
	float led_front_gain;

	// （進んだ距離）
	float distance_mm;
	//現在の速度(mm/s)
	float now_velocity;
	//現在要求の速度(mm/s)
	float request_velocity;
	//　目標速度(mm/s)
	float target_velocity;
	//　現在の加速度(mm/s^2)
	float now_accel;
	//　目標加速度(mm/s^2)
	float target_accel;

	//現在の角度
	float deg;
	//GYRO補正で要求する角度
	float target_deg;
	//GYRO計算したゲイン
	float angle_gain;
	//現在の角速度(deg/s)
	float now_angular_velocity;
	//現在の要求角速度(deg/s)
	float request_angular_velocity;
	//目標角速度(deg/s)
	float target_angular_velocity;
	//現在の角加速度(deg/s^2)
	float now_angular_accel;
	//目標角加速度(deg/s^2)
	float target_angular_accel;
	// 機体の制御モード
	// 0:横壁制御なし　1:横壁制御あり
	char control_mode;

	//PID構造体
	struct {
		float angle_e_p;
		float angle_e_i;
		float angle_e_d;
		float past_angle_e_p;

		float velocity_e_p;
		float velocity_e_i;
		float velocity_e_d;
		float past_velocity_e_p;

		float led_side_e_p;
		float led_side_e_i;
		float led_side_e_d;
		float past_led_side_e_p;
	} PID;

	//　右モータ構造体を作成
	struct {
		// duty rate of motor(+:forward, -:backward)
		float duty;
		//degree of encoder
		float deg;
		// degree/second of encoder
		float angular_velocity;
		//accel
		float accel;
		//目標速度
		float target_velocity;
		//PID制御時の目標速度(中間処理用)(accelをここに加算する)
		float request_velocity;
		//velocity
		float velocity;
		//distance from initialize position
		long distance;
	} r_motor;

	//　左モータ構造体を作成
	struct {
		// duty rate of motor(+:forward, -:backward)
		float duty;
		//degree of encoder
		long deg;
		// degree/second of encoder
		float angular_velocity;
		//目標速度
		float target_velocity;
		//PID制御時の目標速度(中間処理用)(accelをここに加算する)
		float request_velocity;
		//velocity
		float velocity;
		//distance from initialize position
		long distance;
	} l_motor;

	// 吸引用モーター構造体を作成
	struct {
		// duty rate of motor(+:forward, -:ban)
		float duty;
	} suction_motor;
	// 吸引用モーター構造体を作成

	struct {
		//ジャイロのドリフト対策
		short offset_raw;
		//ジャイロのraw値保管
		short z_out_raw;

	} gyro;

	//エンコーダRのraw保管用
	struct {
		short value;
		int raw;
		short raw_offset;

		//積算用
		long raw_integral;
		long raw_integral_offset;

		//角速度算出用
		short raw_angular_velocity;
	} encr;

	//エンコーダLのraw保管用
	struct {
		short value;
		short raw;
		short raw_offset;
		//積算用
		long raw_integral;
		long raw_integral_offset;

		//角速度算出用
		short raw_angular_velocity;
	} encl;

};

#endif /* MOTOR_CONTROL_H_ */
