#include "game.h"
#include <stdlib.h>
#include "stdbool.h"
#include "playerRect.h"
#include "touch_module.h"

static TS_StateTypeDef tsState;

void generateLevel1(Level* level) {
	bool temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};
	
	for (int y = 0; y < FLOOR_HEIGHT; y++) {
		for (int x = 0; x < FLOOR_WIDTH; x++) {
			level->floor[y][x] = temp[y][x];
		}
	}
	
	float temp2[LEVEL1POINTSNUM][LEVEL1POINTSNUM] = {
		{-5, 3},
		{-2, 3},
		{-2, 2},
		{1, 2},
		{1, 1},
		{4, 1},
		{4, 0},
		{5, 0},
		{5, -2},
		{4, -2},
		{4, -3},
		{1, -3},
		{1, -2},
		{0, -2},
		{0, -1},
		{-4, -1},
		{-4, 0},
		{-5, 0},
		{-5, 3}
	};
	
	level->floorMatrix = create_matrix(4, LEVEL1POINTSNUM);
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM; pointIndex++) {
		level->floorMatrix->data[level->floorMatrix->cols*0+pointIndex] = (temp2[pointIndex][0]);
		level->floorMatrix->data[level->floorMatrix->cols*1+pointIndex] = (temp2[pointIndex][1]);
		level->floorMatrix->data[level->floorMatrix->cols*2+pointIndex] = 0;
		level->floorMatrix->data[level->floorMatrix->cols*3+pointIndex] = 1;
	}
}

Level* initLevel() {
	Level* level = (struct Level* ) malloc(sizeof(struct Level));
	return level;
}

void freeLevel(Level* level) {
	free(level);
}

void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	// level screen coord must be freed first
	Matrix* scaledMatrix = scaleMatrix(level->floorMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisZ);
	free_matrix(translatedMatrix);

	
	ScreenCoord coords[LEVEL1POINTSNUM];
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		coords[columnIndex] = coord;
	}
	
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM-1; pointIndex++) {
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y, buffer, WHITE);
	}
	
	free_matrix(projectedMatrix);
}

bool inRect(int touchX, int touchY, int x0, int y0, int deltaX, int deltaY) {
	if (touchX >= x0 && touchX <= x0+deltaX) {
		if (touchY >= y0 && touchY <= y0+deltaY) {
			return true;
		}
	}
	return false;
}

void initButton(Button* button, void (*triggerFunc)(RectPlayer*)) {
	button->firstTickInButton = false;
	button->isTouchingScreenLastTick = false;
	button->lastTouchX = -1;
	button->lastTouchY = -1;
	button->isHolding = false;
	button->triggerFunc = triggerFunc;
}

void buttonToBuffer(Button* button, Buffer* buffer) {
	if (button->triggered) {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, BLACK);
	} else if (button->isHolding) {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, DARKGRAY);
	} else {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, GRAY);
	}
	
}

void buttonTick(Button* button, TS_StateTypeDef* state, RectPlayer* player) {
	button->triggered = false;
	if (state->TouchDetected) {
		int x = state->x;
		int y = state->y;
		
		if (inRect(x, y, button->x0, button->y0, button->wX, button->wY)) {
			if (!button->isTouchingScreenLastTick) {
				button->firstTickInButton = true;
			}
		}
		if (button->firstTickInButton) {
			// holding func
			button->isHolding = true;
		}
		button->isTouchingScreenLastTick = true;
		button->lastTouchX = x;
		button->lastTouchY = y;
	} else {
		button->isHolding = false;
		if (button->firstTickInButton) {
			if (inRect(button->lastTouchX, button->lastTouchY, button->x0, button->y0, button->wX, button->wY)) {
				button->triggerFunc(player);
				player->aniAngleProcess = 90;
				button->triggered = true;
			}
			button->firstTickInButton = false;
		}
		button->isTouchingScreenLastTick = false;
	}
}

void upButtonTrigger(RectPlayer* player) {
	player->lastControl = UP;
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) {
		player->levelPosY1 += 1;
		player->levelPosY2 += 2;
	} else if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 != player->levelPosY2) {
		// lay up/down
		int targetY;
		if (player->levelPosY1 > player->levelPosY2) {
			targetY = player->levelPosY1+1;
		} else {
			targetY = player->levelPosY2+1;
		}
		
		player->levelPosY1 = targetY;
		player->levelPosY2 = targetY;
	} else if (player->levelPosY1 == player->levelPosY2) {
		// lay left/right
		player->levelPosY1 += 1;
		player->levelPosY2 += 1;
	}
}

void downButtonTrigger(RectPlayer* player) {
	player->lastControl = DOWN;
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) {
		player->levelPosY1 -= 1;
		player->levelPosY2 -= 2;
	} else if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 != player->levelPosY2) {
		// lay up/down
		if (player->levelPosY1 > player->levelPosY2) {
			player->levelPosY1 -= 2;
			player->levelPosY2 -= 1;
		} else {
			player->levelPosY1 -= 1;
			player->levelPosY2 -= 2;
		}
	} else if (player->levelPosY1 == player->levelPosY2) {
		// lay left/right
		player->levelPosY1 -= 1;
		player->levelPosY2 -= 1;
	}
}

void leftButtonTrigger(RectPlayer* player) {
	player->lastControl = LEFT;
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) {
		player->levelPosX1 -= 1;
		player->levelPosX2 -= 2;
	} else if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 != player->levelPosY2) {
		// lay up/down
		player->levelPosX1 -= 1;
		player->levelPosX2 -= 1;
	} else if (player->levelPosY1 == player->levelPosY2) {
		// lay left/right
		if (player->levelPosX1 < player->levelPosX2) {
			player->levelPosX1 -= 1;
			player->levelPosX2 -= 2;
		} else {
			player->levelPosX2 -= 1;
			player->levelPosX1 -= 2;
		}
	}
}

void rightButtonTrigger(RectPlayer* player) {
	player->lastControl = RIGHT;
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) {
		player->levelPosX1 += 1;
		player->levelPosX2 += 2;
	} else if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 != player->levelPosY2) {
		// lay up/down
		player->levelPosX1 += 1;
		player->levelPosX2 += 1;
	} else if (player->levelPosY1 == player->levelPosY2) {
		// lay left/right
		if (player->levelPosX1 < player->levelPosX2) {
			player->levelPosX1 += 2;
			player->levelPosX2 += 1;
		} else {
			player->levelPosX2 += 2;
			player->levelPosX1 += 1;
		}
	}
}

void play() {
	Buffer* buffer = createBuffer();
	Level* level = initLevel();
	
	Button upButton = {.x0=LCD_Width/2-BUTTON_WIDTH/2, .y0=0, .wX=BUTTON_WIDTH, .wY=BUTTON_HEIGHT};
	Button downButton = {.x0=LCD_Width/2-BUTTON_WIDTH/2, .y0=LCD_Height-BUTTON_HEIGHT, .wX=BUTTON_WIDTH, .wY=BUTTON_HEIGHT};
	Button leftButton = {.x0=0, .y0=LCD_Height/2-BUTTON_WIDTH/2, .wX=BUTTON_HEIGHT, .wY=BUTTON_WIDTH};
	Button rightButton = {.x0=LCD_Width-BUTTON_HEIGHT, .y0=LCD_Height/2-BUTTON_WIDTH/2, .wX=BUTTON_HEIGHT, .wY=BUTTON_WIDTH};
	
	initButton(&upButton, upButtonTrigger);
	initButton(&downButton, downButtonTrigger);
	initButton(&leftButton, leftButtonTrigger);
	initButton(&rightButton, rightButtonTrigger);
	
	RectPlayer player;
	initPlayer(&player, 0, 0);
	
	generateLevel1(level);
	
	double worldRotX = -60;
	double worldRotZ = 0;
	int touchX;
	int touchY;
	while (1) {
		TS_GetState(&tsState);
		worldRotZ = 10;
		clearBuffer(BLACK, buffer, NULL);
		projectPlayerRectToBuffer(&player, buffer, 0.5, worldRotX, worldRotZ, -8, 50, 0.1, 100.0);
		projectLevelToBuffer(level, buffer, 0.5, worldRotX, worldRotZ, -8, 50, 0.1, 100.0);
		
		buttonTick(&upButton, &tsState, &player);
		buttonTick(&downButton, &tsState, &player);
		buttonTick(&leftButton, &tsState, &player);
		buttonTick(&rightButton, &tsState, &player);
		
		buttonToBuffer(&upButton, buffer);
		buttonToBuffer(&downButton, buffer);
		buttonToBuffer(&leftButton, buffer);
		buttonToBuffer(&rightButton, buffer);
		
		drawBuffer(buffer);
	}

	freeLevel(level);
	freeBuffer(buffer);
}
	