/*
 * adjust.h
 *
 *  Created on: 2017/09/30
 *      Author: 廣明
 */

#ifndef ADJUST_H_
#define ADJUST_H_
///安全対策==================================================================
#define BATTERY_LIMIT_VOLTAGE 3600
#define SPI_TIME_OUT 1//x100us
#define GYRO_I_THRESHOLD 100000000000//許容要求角ずれ(deg)*1000
#define ENCODER_I_THRESHOLD 10000000000//許容要求距離ずれ(mm)*1000
#define TREAD 41.5			//単位:mm
#define TIRE_RADIUS 13.42F	//単位:mm
///等高線マップ最大値（初期化用）
#define MAX_SMAP 32760
///迷路区画数(一辺)
#define NUM_BLOCK 16
///迷路ゴール区画設定
#define GOAL_X 3
#define GOAL_Y 0

///ログとり変数の数(1kHz 20s 0000)
#define LOG_MAX 20000
#define LOG_ITEM 2

///足立法(SEARCH)400mm
#define SEARCH_SPEED 400
#define SEARCH_ACCEL 3000

///足立法(SEARCH)150mm#######################
#define SEARCH_150_SPEED 150
#define SEARCH_150_ACCEL 3000

///足立法(SEARCH)250mm#######################
#define SEARCH_250_SPEED 250
#define SEARCH_250_ACCEL 5000

///足立法(SEARCH)300mm#######################
#define SEARCH_300_SPEED 300
#define SEARCH_300_ACCEL 8000

///足立法(Try)STRAIGHT:700mm, TURN:150mm#####
#define TRY_TURN_150mm_SPEED 150
#define TRY_STRAIGHT_1ST 700
#define TRY_TURN_150mm_ACCEL 3500

///足立法(Try)STRAIGHT:600mm, TURN:300mm#####
#define TRY_TURN_300mm_SPEED 300
#define TRY_STRAIGHT_2ND 600
#define TRY_TURN_300mm_ACCEL 8000

///足立法(Try)STRAIGHT:750mm, TURN:300mm#####
//#define TRY_TURN_300mm_SPEED 300
#define TRY_STRAIGHT_3RD 700
//#define TRY_TURN_300mm_ACCEL 8000

///足立法(Try)STRAIGHT:2000mm, TURN:300mm#####
//#define TRY_TURN_300mm_SPEED 300
#define TRY_STRAIGHT_4TH 900
//#define TRY_TURN_300mm_ACCEL 8000

///#########################################

#define Z_GYRO_GAIN_FORWARD 1.0F
#define Z_GYRO_GAIN_BACKWARD 1.0F
///######スラローム走行(150mm/s)###########
#define COUNTER_ROTATION_TURN 90.53F

#define SLALOM_150_L_90_DEG 90.6F
#define SLALOM_150_L_IN_OFFSET 9.0F
#define SLALOM_150_L_OUT_OFFSET 12.5F

#define SLALOM_150_R_90_DEG 90.61F
#define SLALOM_150_R_IN_OFFSET 9.0F
#define SLALOM_150_R_OUT_OFFSET 12.5F

#define SLALOM_150_ANGULAR_VELOCITY 343.774F
#define SLALOM_150_ANGULAR_ACCEL 2626.245F
///######スラローム走行(250mm/s)###########
#define SLALOM_250_L_90_DEG 90.72F
#define SLALOM_250_L_IN_OFFSET 10.0F
#define SLALOM_250_L_OUT_OFFSET 10.0F

#define SLALOM_250_R_90_DEG 90.75F
#define SLALOM_250_R_IN_OFFSET 9.5F
#define SLALOM_250_R_OUT_OFFSET 10.5F

#define SLALOM_250_ANGULAR_VELOCITY 573.0F
#define SLALOM_250_ANGULAR_ACCEL 7295.1F

///######スラローム走行(300mm/s)###########
#define SLALOM_300_L_90_DEG 90.675F
#define SLALOM_300_L_IN_OFFSET 9.5F
#define SLALOM_300_L_OUT_OFFSET 13.5F

#define SLALOM_300_R_90_DEG 90.675F
#define SLALOM_300_R_IN_OFFSET 9.5F
#define SLALOM_300_R_OUT_OFFSET 13.5F

#define SLALOM_300_ANGULAR_VELOCITY 613.9F
#define SLALOM_300_ANGULAR_ACCEL 12561.8F
///######スラローム走行(500mm/s)###########
#define SLALOM_500_L_90_DEG 90.675F
#define SLALOM_500_L_IN_OFFSET 9.5F
#define SLALOM_500_L_OUT_OFFSET 13.5F

#define SLALOM_500_R_90_DEG 90.675F
#define SLALOM_500_R_IN_OFFSET 9.5F
#define SLALOM_500_R_OUT_OFFSET 13.5F

#define SLALOM_500_ANGULAR_VELOCITY 613.9F
#define SLALOM_500_ANGULAR_ACCEL 12561.8F

#define GYRO_BIT_TO_DEG 0.061035156F //2000/32767

//==========PIDゲイン=========================
//モータ用速度PID
#define MOTOR_VELOCITY_GAIN_P 3.5F
#define MOTOR_VELOCITY_GAIN_I 0.385F
#define MOTOR_VELOCITY_GAIN_D 0.1F

//GYRO用角度PID
#define ANGLE_GAIN_P 3.60F
#define ANGLE_GAIN_I_SIDE_WALL_OFF 0.5F
#define ANGLE_GAIN_I_SIDE_WALL_ON 0.095F
#define ANGLE_GAIN_D 0.03F

//横壁センサーPID
#define LED_SIDE_GAIN_P_LOW_SPEED 20.0F
#define LED_SIDE_GAIN_P_HIGH_SPEED 8.0F
#define LED_SIDE_GAIN_I 0.0F
#define LED_SIDE_GAIN_D 0.0F

//前壁センサーPID
#define LED_FRONT_GAIN_P 30.0F
#define LED_FRONT_GAIN_I 0.0F
#define LED_FRONT_GAIN_D 0.0F

// =========== 横壁閾値	　===================================

#define  SIDE_R_LIMIT 10	//raw
#define  FRONT_R_LIMIT 40	//車軸が柱の間
#define  FRONT_L_LIMIT 80
#define  SIDE_L_LIMIT 20

// =========== 横壁補正時の閾値	　===================================

#define  SIDE_R_ref_LIMIT	50	//raw
#define  FRONT_R_ref_LIMIT 45	//車軸が柱の間
#define  FRONT_L_ref_LIMIT 90
#define  SIDE_L_ref_LIMIT	165

#define  FRONT_R_PRE_LIMIT 150//車軸が1区間前
#define  FRONT_L_PRE_LIMIT 70//

#define DEG_TO_RAD(deg) (((deg) * PI) / 180.0)
#define RAD_TO_DEG(rad) (((rad) * 180.0) / PI)
#define ABS(IN) ((IN) < 0 ? - (IN) : (IN))

// =========== 初期リファレンス値	　===================================

#define  SIDE_R_REFER	14.1F	//mm
#define  SIDE_L_REFER	15.1F
#define  FRONT_R_REFER	56.0F	//車軸が中心に存在する
#define  FRONT_L_REFER	57.0F	//場合の正面までの距離(スラローム)

#define FRONT_R_CENTER_REFER 14.9F
#define FRONT_L_CENTER_REFER 13.9F

#endif /* ADJUST_H_ */
