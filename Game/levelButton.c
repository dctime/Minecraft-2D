#include "levelButton.h"
#include "game.h"


void initLevelButton(LevelButton* button, int levelX, int levelY, int* floorPoints, int floorCount) {
	button->isOn = false;
	button->levelX = levelX;
	button->levelY = levelY;
	button->levelButtonMatrix = create_matrix(4, 4);
	button->levelTriggerFloorMatrix = create_matrix(4, floorCount*4);
	
	for (int buttonPointIndex = 0; buttonPointIndex < 4; buttonPointIndex++) {
		button->levelButtonMatrix->data[4*0+buttonPointIndex] = levelX;
		button->levelButtonMatrix->data[4*1+buttonPointIndex] = levelY;
		button->levelButtonMatrix->data[4*2+buttonPointIndex] = 0;
		button->levelButtonMatrix->data[4*3+buttonPointIndex] = 1;
	}
	button->levelButtonMatrix->data[4*0+1] += 1;
	button->levelButtonMatrix->data[4*1+2] += 1;
	button->levelButtonMatrix->data[4*0+2] += 1;
	button->levelButtonMatrix->data[4*1+3] += 1;
	
	for (int floorPointIndex = 0; floorPointIndex < floorCount*4; floorPointIndex++) {
		button->levelTriggerFloorMatrix->data[4*0+floorPointIndex] = *(floorPoints+floorPointIndex/4*2);
		button->levelTriggerFloorMatrix->data[4*1+floorPointIndex] = *(floorPoints+floorPointIndex/4*2+1);
		button->levelTriggerFloorMatrix->data[4*2+floorPointIndex] = 0;
		button->levelTriggerFloorMatrix->data[4*3+floorPointIndex] = 1;
	}
	
	for (int floorTileIndex = 0; floorTileIndex < floorCount; floorTileIndex++) {
		button->levelTriggerFloorMatrix->data[4*0+floorTileIndex*4+1] += 1;
		button->levelTriggerFloorMatrix->data[4*1+floorTileIndex*4+2] += 1;
		button->levelTriggerFloorMatrix->data[4*0+floorTileIndex*4+2] += 1;
		button->levelTriggerFloorMatrix->data[4*1+floorTileIndex*4+3] += 1;
	}
	
//	for (int floorPointIndex = 0; floorPointIndex < floorCount*4; floorPointIndex++) {
//		debugText(button->levelTriggerFloorMatrix->data[4*0+floorPointIndex]);
//		debugText(button->levelTriggerFloorMatrix->data[4*1+floorPointIndex]);
//		debugText(button->levelTriggerFloorMatrix->data[4*2+floorPointIndex]);
//		debugText(button->levelTriggerFloorMatrix->data[4*3+floorPointIndex]);
//	}
	
	button->floorCount = floorCount;
	button->levelButtonAlreadyPressed = false;
}

void freeLevelButtonMatrices(LevelButton* button) {
	free_matrix(button->levelButtonMatrix);
	free_matrix(button->levelTriggerFloorMatrix);
}


void levelButtonTick(LevelButton* button, RectPlayer* player) {
	if (!(player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2)) {
		button->levelButtonAlreadyPressed = false;
		return;
	}
	
	if (!(player->levelPosX1 == button->levelX && player->levelPosY1 == button->levelY)) return;
	if (player->aniAngleProcess != 0) return;
	if (!button->levelButtonAlreadyPressed) {
		button->levelButtonAlreadyPressed = true;
		button->isOn = !button->isOn;
		// do press logic
	}
}