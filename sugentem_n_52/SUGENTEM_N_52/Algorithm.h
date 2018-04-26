/*
 * Algorithm.h
 *
 *  Created on: 2017/10/19
 *      Author: 廣明
 */

#ifndef ALGORITHM_H_
#define ALGORITHM_H_

/*　 =============================================================================
 *
 * プロトタイプ宣言
 *
 * =============================================================================　*/
void SEARCH_Para_Set(char mx, char my, char head);

void MAZE_Map_Clear(void);
void make_smap(int gx, int gy, int mode);

void mouse_search_150mm_Adachi_Normal(int tx, int ty, int mode);
void mouse_search_250mm_Adachi_Normal(int tx, int ty, int mode);
void mouse_search_300mm_Adachi_Normal(int tx, int ty, int mode);

void mouse_try_STRAIGHT_1ST_TURN_150mm_Adachi(int tx, int ty);
void mouse_try_STRAIGHT_2ND_TURN_300mm_Adachi(int tx, int ty);
void mouse_try_STRAIGHT_3RD_TURN_300mm_Adachi(int tx, int ty);
void mouse_try_STRAIGHT_4TH_TURN_300mm_Adachi(int tx, int ty);

void ENABLE_SLALOM_FLAG(void);
void DISABLE_SLALOM_FLAG(void);

void decide_direction(short *mouse_head0, short *s0, short *s1);
/*　 =============================================================================
 *
 * マクロ定義
 *
 * =============================================================================　*/

enum Search_Mode {
	T_MODE = 0, S_MODE
};

enum Search_Dir {
	GO_FORWARD, GO_RIGHT, GO_BACK, GO_LEFT
};
/*　 =============================================================================
 *
 * 構造体
 *
 * =============================================================================　*/
// maze
struct Maze {

	// 機体の現在座標
	char mx;
	char my;

	// 機体の向き
	//0:North 1:East 2:South 3:West
	char head;

};

/* ======	ステップ数  ==========*/
//step
struct Step {
	// １区間前進
	short straight;
	// １８０度ターン
	short turn;
};

#endif /* ALGORITHM_H_ */
