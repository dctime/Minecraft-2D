#ifndef PLAYER_RECT
#define PLAYER_RECT
#include "matrix.h"
#include "dctime_lcd.h"
#include "game.h"
#define PLAYERRECTPOINTNUM 8


typedef struct RectPlayer {
	Matrix* modelMatrix;
	int levelPosX1;
	int levelPosY1;
	int levelPosX2;
	int levelPosY2;
} RectPlayer;

void initPlayer(RectPlayer* player, int startLocX, int startLocY);
void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotY, double tZ, double fov, double near, double far);

#endif