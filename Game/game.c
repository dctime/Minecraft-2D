#include "game.h"
#include <stdlib.h>
#include "stdbool.h"
#include "playerRect.h"
#include "touch_module.h"
#include "uiButton.h"
#include "levelButton.h"
#include <stdio.h>
#include "stm324xg_lcd_sklin.h"
#include <limits.h>
#include "user_defined.h"

static TS_StateTypeDef tsState;

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

void projectLevelFixToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	if (level->fixPairsCount == 0) return;
	Matrix* scaledMatrix = scaleMatrix(level->fixMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisZ);
	free_matrix(translatedMatrix);
	
	ScreenCoord* coords = malloc(sizeof(ScreenCoord) * level->fixPairsCount*2);
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		coords[columnIndex] = coord;
	}
	
	for (int pairIndex = 0; pairIndex < level->fixPairsCount; pairIndex++) {
		Buffer_DrawLine(coords[pairIndex*2].x, coords[pairIndex*2].y, coords[pairIndex*2+1].x, coords[pairIndex*2+1].y, buffer, BLACK);
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

bool checkWinning(Level* level, RectPlayer* player) {
	if (player->aniAngleProcess != 0) return false;
//	debugText(player->levelPosX1);
//	debugText(player->levelPosX2);
//	debugText(player->levelPosY1);
//	debugText(player->levelPosY2);
	if (posWinning(level, player->levelPosX1, player->levelPosX2, player->levelPosY1, player->levelPosY2)) return true;
	return false;
}

volatile bool key1Triggered;

void nextLevelScreen(int usedStep) {
	LCD_Clear(BLACK);
	accTotalUsedStep(usedStep);
	char c[22];
	char c2[23] = "Press KEY1 to continue";
	sprintf(c, "Total Used Step: %d", getTotalUsedStep());
	LCD_SaveFont();
	LCD_SaveColors();
	LCD_SetFont(&Font16);	
	LCD_SetColors(WHITE, BLACK); // Text = red; back = white
	LCD_DisplayStringLineCol(6, 3, c);
	LCD_DisplayStringLineCol(10, 3, c2);
	LCD_RestoreColors();
	LCD_RestoreFont();
	
	key1Triggered = 0;
	while(!key1Triggered);
	key1Triggered = 0;
}

volatile bool key1Triggered = 0;

//void Wait_PressPA0(uint16_t Cnum)
//{
//	uint16_t count = Cnum;
//	while(1)
//	{	
//    if(GPIOA->IDR & 0x01)						// normally low
//    {
//			if (--(count)==0){
//				return;
//			}
//		} else count = Cnum;

//    delay_ms(10);
//	}
//}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {  //
        EXTI->PR = EXTI_PR_PR0;    //
				key1Triggered = 1;
    }
}

bool play(void (*genLevelFunc)(Level*)) {
	key1Triggered = false;
	
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
	initPlayer(&player, -2, 0);
	
	genLevelFunc(level);
	
	double worldRotX = -60;
	int touchX;
	int touchY;
	
	key1Triggered = false;
	while (!key1Triggered) {
		TS_GetState(&tsState);
		
		if (checkWinning(level, &player)) {
			nextLevelScreen(player.usedStep);
			
			freeLevel(level);
			freeBuffer(buffer);
			freePlayerModel(&player);
			return true;
		}
		
		buttonTick(&upButton, &tsState, &player, level);
		buttonTick(&downButton, &tsState, &player, level);
		buttonTick(&leftButton, &tsState, &player, level);
		buttonTick(&rightButton, &tsState, &player, level);
		screenControlTick(&screenControlObject, &tsState, buttons);
		levelButtonTick(level->buttons, &player);
		
		// if not button holding
		// x coord move 
		
		clearBuffer(BLACK, buffer, NULL);
		
		projectLevelToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectLevelFixToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectGoalToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectLevelButtonToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectLevelAllButtonTriggeredFloorToBuffer(level, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		projectPlayerRectToBuffer(&player, buffer, 0.5, worldRotX, screenControlObject.screenRotZDeg, -8, 50, 0.1, 100.0);
		
		buttonToBuffer(&upButton, buffer);
		buttonToBuffer(&downButton, buffer);
		buttonToBuffer(&leftButton, buffer);
		buttonToBuffer(&rightButton, buffer);
		
		char stepCountText[15];
		sprintf(stepCountText, "Steps Used: %d", player.usedStep);
		Buffer_DisplayStringAt(0, 0, stepCountText, LEFT_MODE, buffer, &Font12, WHITE, BLACK);
		
		drawBuffer(buffer);
	}

	freeLevel(level);
	freeBuffer(buffer);
	freePlayerModel(&player);
	
	return false;
}

static int leastTotalUsedStep = INT_MAX;
static int totalUsedStep = 0;

int getTotalUsedStep() {
	return totalUsedStep;
}

void accTotalUsedStep(int levelUsedStep) {
	totalUsedStep += levelUsedStep;
}

void resetTotalUsedStep() {
	totalUsedStep = 0;
}

bool trySetLeastTotalUsedStep(int stepsCount) {
	if (leastTotalUsedStep > stepsCount) {
		leastTotalUsedStep = stepsCount;
		return true;
	} 
	return false;
}

int getLeastTotalUsedStep() {
	return leastTotalUsedStep;
}


	