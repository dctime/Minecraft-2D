#include "game.h"
#include <stdlib.h>
#include "stdbool.h"
#include "playerRect.h"
#include "touch_module.h"

static TS_StateTypeDef tsState;


void projectLevelButtonToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	Matrix* scaledMatrix = scaleMatrix(level->buttons->levelButtonMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisZ);
	free_matrix(translatedMatrix);
	
//	debugText(level->buttonCount);
	
	ScreenCoord* coords = malloc(sizeof(ScreenCoord) * level->buttonCount * 4);
	
	for (int i = 0; i < level->buttonCount*4; i++) {
		*(coords+i) = getCoordFromMatrix(i, LCD_Width, LCD_Height, projectedMatrix);
//		debugText((*(coords+i)).x);
//		debugText((*(coords+i)).y);
	}
	// one button has 4 points
	for (int buttonIndex = 0; buttonIndex < level->buttonCount; buttonIndex++) {
//		LCD_DrawLine((coords+4*buttonIndex+0)->x, (coords+4*buttonIndex+0)->y, (coords+4*buttonIndex+2)->x, (coords+4*buttonIndex+2)->y);
//		LCD_DrawLine((coords+4*buttonIndex+1)->x, (coords+4*buttonIndex+1)->y, (coords+4*buttonIndex+3)->x, (coords+4*buttonIndex+3)->y);
		Buffer_DrawLine((coords+4*buttonIndex+0)->x, (coords+4*buttonIndex+0)->y, (coords+4*buttonIndex+2)->x, (coords+4*buttonIndex+2)->y, buffer, YELLOW);
		Buffer_DrawLine((coords+4*buttonIndex+1)->x, (coords+4*buttonIndex+1)->y, (coords+4*buttonIndex+3)->x, (coords+4*buttonIndex+3)->y, buffer, YELLOW);
	}
	
	free(coords);
	free_matrix(projectedMatrix);
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

	ScreenCoord* coords = malloc(sizeof(ScreenCoord) * level->floorPointsNum);
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		coords[columnIndex] = coord;
	}
	
	for (int pointIndex = 0; pointIndex < level->floorPointsNum-1; pointIndex++) {
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y, buffer, WHITE);
	}
	
	free(coords);
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

void initScreenControlObject(ScreenControlObject* object) {
	object->firstTickOnScreen = false;
	object->isTouchingScreenLastTick = false;
	object->lastTouchX = -1;
	object->lastTouchY = -1;
	object->isHolding = false;
	object->triggered = false;
	object->screenRotZDeg = 30;
}

void initButton(Button* button, void (*triggerFunc)(RectPlayer*, Level*)) {
	button->firstTickInButton = false;
	button->isTouchingScreenLastTick = false;
	button->lastTouchX = -1;
	button->lastTouchY = -1;
	button->isHolding = false;
	button->triggered = false;
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

void screenHolding(ScreenControlObject* object, int currentX, int currentY) {
	if (object->lastTouchX == -1 || object->lastTouchY == -1) return;
	int deltaX = currentX - object->lastTouchX;
	object->screenRotZDeg += deltaX;
}

void screenControlTick(ScreenControlObject* object, TS_StateTypeDef* state, Button buttons[4]) {
	object->triggered = false;
	if (state->TouchDetected) {
		int x = state->x;
		int y = state->y;
		
		bool notInButton = true;
		for (int buttonID = 0; buttonID < 4; buttonID++) {
			Button* button = buttons+buttonID;
			if (inRect(x, y, button->x0, button->y0, button->wX, button->wY)) notInButton = false;
		}
		
		if (notInButton) {
			if (!object->isTouchingScreenLastTick) {
				object->firstTickOnScreen = true;
				object->lastTouchX = -1;
				object->lastTouchY = -1;
			}
		}
		if (object->firstTickOnScreen) {
			// holding func
			object->isHolding = true;
			// screen moving
			screenHolding(object, x, y);
			
		}
		object->isTouchingScreenLastTick = true;
		object->lastTouchX = x;
		object->lastTouchY = y;
	} else {
		object->isHolding = false;
		if (object->firstTickOnScreen) {
			bool notInButton = true;
			for (int buttonID = 0; buttonID < 4; buttonID++) {
				Button* button = buttons+buttonID;
				if (inRect(button->lastTouchX, button->lastTouchY, button->x0, button->y0, button->wX, button->wY)) notInButton = false;
			}
			if (notInButton) {
				object->triggered = true;
			}
			object->firstTickOnScreen = false;
			object->lastTouchX = -1;
			object->lastTouchY = -1;
		}
		object->isTouchingScreenLastTick = false;
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
	Button buttons[4] = {
		upButton,
		downButton,
		leftButton,
		rightButton
	};
	ScreenControlObject screenControlObject;
	
	initButton(&upButton, upButtonTrigger);
	initButton(&downButton, downButtonTrigger);
	initButton(&leftButton, leftButtonTrigger);
	initButton(&rightButton, rightButtonTrigger);
	initScreenControlObject(&screenControlObject);
	
	RectPlayer player;
	initPlayer(&player, 0, 0);
	
	generateLevel2(level);
	
	double worldRotX = -60;
	int touchX;
	int touchY;
	while (!gameReset) {
		TS_GetState(&tsState);
		
		if (checkWinning(level, &player)) {
			youWinScreen();
			break;
		}
		
		buttonTick(&upButton, &tsState, &player, level);
		buttonTick(&downButton, &tsState, &player, level);
		buttonTick(&leftButton, &tsState, &player, level);
		buttonTick(&rightButton, &tsState, &player, level);
		screenControlTick(&screenControlObject, &tsState, buttons);
		
		// if not button holding
		// x coord move 
		
		clearBuffer(BLACK, buffer, NULL);
		
		projectLevelToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectGoalToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectLevelButtonToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectPlayerRectToBuffer(&player, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		
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


	