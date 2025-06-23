#ifndef PLAYER_RECT
#define PLAYER_RECT
#include "matrix.h"
#include "dctime_lcd.h"
#include "game.h"
#define PLAYERRECTPOINTNUM 8

enum PlayerLastControl {
	UP, DOWN, LEFT, RIGHT
};

typedef struct RectPlayer {
	Matrix* modelMatrix;
	int levelPosX1;
	int levelPosY1;
	int levelPosX2;
	int levelPosY2;
	int aniAngleProcess; // 0 <- 90
	enum PlayerLastControl lastControl;
	int usedStep;
} RectPlayer;

void initPlayer(RectPlayer* player, int startLocX, int startLocY);
void freePlayerModel(RectPlayer* player);

#endif