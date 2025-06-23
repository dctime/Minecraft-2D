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
#include "screenControlObject.h"
#include "projectToBuffer.h"

static TS_StateTypeDef tsState;

bool inRect(int touchX, int touchY, int x0, int y0, int deltaX, int deltaY) {
	if (touchX >= x0 && touchX <= x0+deltaX) {
		if (touchY >= y0 && touchY <= y0+deltaY) {
			return true;
		}
	}
	return false;
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


	