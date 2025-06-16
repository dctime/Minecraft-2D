#include "game.h"
#include <stdlib.h>
#include "stdbool.h"
#include "playerRect.h"
#include "touch_module.h"

static TS_StateTypeDef tsState;

void generateLevel1(Level* level) {
	uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};
	
	for (int y = 0; y < FLOOR_HEIGHT; y++) {
		for (int x = 0; x < FLOOR_WIDTH; x++) {
			level->floor[y][x] = temp[y][x];
		}
	}
	
	float temp2[LEVEL1POINTSNUM][2] = {
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
	
	float tempGoalVec[GOALPOINTSNUM][2] = {
		{2, -1},
		{3, -1},
		{3, -2},
		{2, -2}
	};
	
	level->goalMatrix = create_matrix(4, GOALPOINTSNUM);
	for (int i = 0; i < GOALPOINTSNUM; i++) {
		level->goalMatrix->data[level->goalMatrix->cols*0+i] = tempGoalVec[i][0];
		level->goalMatrix->data[level->goalMatrix->cols*1+i] = tempGoalVec[i][1];
		level->goalMatrix->data[level->goalMatrix->cols*2+i] = 0;
		level->goalMatrix->data[level->goalMatrix->cols*3+i] = 1;
	}
}

int getLevelTile(Level* level, int x, int y) {
	// debugText(level->floor[-y+3][x+8]);
	return level->floor[-y+3][x+8];
}

bool posValid(Level* level, int x1, int x2, int y1, int y2) {
	if (getLevelTile(level, x1, y1) == 0) return false;
	if (getLevelTile(level, x2, y2) == 0) return false;
	return true;
}

bool posWinning(Level* level, int x1, int x2, int y1, int y2) {
	if (getLevelTile(level, x1, y1) == 2 && getLevelTile(level, x2, y2) == 2) return true;
	// debugText(-1);
	return false;
}

Level* initLevel() {
	Level* level = (struct Level* ) malloc(sizeof(struct Level));
	return level;
}

void freeLevel(Level* level) {
	free(level);
}

void projectGoalToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	Matrix* scaledMatrix = scaleMatrix(level->goalMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisZ);
	free_matrix(translatedMatrix);
	
	ScreenCoord coords[GOALPOINTSNUM];
	
	for (int i = 0; i < GOALPOINTSNUM; i++) {
		struct ScreenCoord coord = getCoordFromMatrix(i, LCD_Width, LCD_Height, projectedMatrix);
		coords[i] = coord;
	}
	
	for (int pointIndex = 0; pointIndex < GOALPOINTSNUM-1; pointIndex++) {
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y, buffer, LCD_COLOR_DARKMAGENTA);
		// LCD_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y);
	}
	
	Buffer_DrawLine(coords[0].x, coords[0].y, coords[3].x, coords[3].y, buffer, LCD_COLOR_DARKMAGENTA);
	Buffer_DrawLine(coords[0].x, coords[0].y, coords[2].x, coords[2].y, buffer, LCD_COLOR_DARKMAGENTA);
	Buffer_DrawLine(coords[1].x, coords[1].y, coords[3].x, coords[3].y, buffer, LCD_COLOR_DARKMAGENTA);
	
	free_matrix(projectedMatrix);
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

void initButton(Button* button, void (*triggerFunc)(RectPlayer*, Level*)) {
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

void buttonTick(Button* button, TS_StateTypeDef* state, RectPlayer* player, Level* level) {
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
			if (inRect(button->lastTouchX, button->lastTouchY, button->x0, button->y0, button->wX, button->wY) && player->aniAngleProcess == 0) {
				button->triggerFunc(player, level);
				button->triggered = true;
			}
			button->firstTickInButton = false;
		}
		button->isTouchingScreenLastTick = false;
	}
}

void upButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;
	
	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetY1 += 1;
		targetY2 += 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		int targetY;
		if (targetY1 > targetY2) {
			targetY = targetY1+1;
		} else {
			targetY = targetY2+1;
		}
		
		targetY1 = targetY;
		targetY2 = targetY;
	} else if (targetY1 == targetY2) {
		// lay left/right
		targetY1 += 1;
		targetY2 += 1;
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = UP;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
	}
}

void downButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetY1 -= 1;
		targetY2 -= 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		if (targetY1 > targetY2) {
			targetY1 -= 2;
			targetY2 -= 1;
		} else {
			targetY1 -= 1;
			targetY2 -= 2;
		}
	} else if (targetY1 == targetY2) {
		// lay left/right
		targetY1 -= 1;
		targetY2 -= 1;
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = DOWN;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
	}
}

void leftButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetX1 -= 1;
		targetX2 -= 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		targetX1 -= 1;
		targetX2 -= 1;
	} else if (targetY1 == targetY2) {
		// lay left/right
		if (targetX1 < targetX2) {
			targetX1 -= 1;
			targetX2 -= 2;
		} else {
			targetX2 -= 1;
			targetX1 -= 2;
		}
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = LEFT;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
	}
}

void rightButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetX1 += 1;
		targetX2 += 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		targetX1 += 1;
		targetX2 += 1;
	} else if (targetY1 == targetY2) {
		// lay left/right
		if (targetX1 < targetX2) {
			targetX1 += 2;
			targetX2 += 1;
		} else {
			targetX2 += 2;
			targetX1 += 1;
		}
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = RIGHT;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
	}
}

bool checkWinning(Level* level, RectPlayer* player) {
	if (player->aniAngleProcess != 0) return false;
//	debugText(player->levelPosX1);
//	debugText(player->levelPosX2);
//	debugText(player->levelPosY1);
//	debugText(player->levelPosY2);
	if (posWinning(level, player->levelPosX1, player->levelPosX2, player->levelPosY1, player->levelPosY2)) return true;
	return false;
}

volatile static bool gameReset;

void youWinScreen() {
	LCD_Clear(BLACK);
	char c[10] = "YOU WIN!";
	LCD_SaveFont();
	LCD_SaveColors();
	LCD_SetFont(&Font16);	
	LCD_SetColors(WHITE, BLACK); // Text = red; back = white
	LCD_DisplayStringLineCol(6, 10, c);
	LCD_RestoreColors();
	LCD_RestoreFont();
	
		while(!gameReset)
			continue;
}

volatile static bool gameReset = 0;

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {  // ??? EXTI0 ???
        EXTI->PR = EXTI_PR_PR0;    // ??????
				gameReset = 1;
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
	while (!gameReset) {
		TS_GetState(&tsState);
		worldRotZ = 30;
		
		if (checkWinning(level, &player)) {
			youWinScreen();
			break;
		}
		
		buttonTick(&upButton, &tsState, &player, level);
		buttonTick(&downButton, &tsState, &player, level);
		buttonTick(&leftButton, &tsState, &player, level);
		buttonTick(&rightButton, &tsState, &player, level);
		
		clearBuffer(BLACK, buffer, NULL);
		
		
		projectLevelToBuffer(level, buffer, 0.5, worldRotX, worldRotZ, -8, 50, 0.1, 100.0);
		projectGoalToBuffer(level, buffer, 0.5, worldRotX, worldRotZ, -8, 50, 0.1, 100.0);
		projectPlayerRectToBuffer(&player, buffer, 0.5, worldRotX, worldRotZ, -8, 50, 0.1, 100.0);
		
		buttonToBuffer(&upButton, buffer);
		buttonToBuffer(&downButton, buffer);
		buttonToBuffer(&leftButton, buffer);
		buttonToBuffer(&rightButton, buffer);
		
		drawBuffer(buffer);
	}

	freeLevel(level);
	freeBuffer(buffer);
	gameReset = 0;
}


	