#include "playerRect.h"
#include "dctimegl.h"
#include "stm324xg_lcd_sklin.h"





void initPlayer(RectPlayer* player, int startLocX, int startLocY) {
	player->modelMatrix = create_matrix(4, PLAYERRECTPOINTNUM);
	if (player->modelMatrix == NULL) {
		while(1);
	}
	
	player->aniAngleProcess = 0;
	player->lastControl = UP;
	player->levelPosX1 = startLocX;
	player->levelPosY1 = startLocY;
	// direction of the laydown
	player->levelPosX2 = startLocX;
	player->levelPosY2 = startLocY;
	
	
	uint8_t modelPlayer[8][3] = {
		{0, 0, 0},
		{0, 1, 0},
		{1, 1, 0},
		{1, 0, 0},
		{0, 0, 2},
		{0, 1, 2},
		{1, 1, 2},
		{1, 0, 2},
	};
	
	for (int pointIndex = 0; pointIndex < PLAYERRECTPOINTNUM; pointIndex++) {
		player->modelMatrix->data[PLAYERRECTPOINTNUM*0+pointIndex] = modelPlayer[pointIndex][0];
		player->modelMatrix->data[PLAYERRECTPOINTNUM*1+pointIndex] = modelPlayer[pointIndex][1];
		player->modelMatrix->data[PLAYERRECTPOINTNUM*2+pointIndex] = modelPlayer[pointIndex][2];
		player->modelMatrix->data[PLAYERRECTPOINTNUM*3+pointIndex] = 1;
	}
	
	player->usedStep = 0;
}

void freePlayerModel(RectPlayer* player) {
	free_matrix(player->modelMatrix);
}
	