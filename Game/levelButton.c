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
	

	
	// button triggered floor
	
	free_matrix(projectedMatrix);
}
void projectButtonTriggeredFloorToBuffer(LevelButton* button, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	// assume there is only one button
	// fixed level->buttons->levelTriggerFloorMatrix
	Matrix* scaledMatrix = scaleMatrix(button->levelTriggerFloorMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisZ);
	free_matrix(translatedMatrix);
	
	ScreenCoord* coords = malloc(sizeof(ScreenCoord) * button->floorCount * 4);
	
	for (int i = 0; i < button->floorCount*4; i++) {
		*(coords+i) = getCoordFromMatrix(i, LCD_Width, LCD_Height, projectedMatrix);
//		debugText((*(coords+i)).x);
//		debugText((*(coords+i)).y);
	}
	
	for (int floorIndex = 0; floorIndex < button->floorCount; floorIndex++) {
//		LCD_DrawLine((coords+4*floorIndex+0)->x, (coords+4*floorIndex+0)->y, (coords+4*floorIndex+2)->x, (coords+4*floorIndex+2)->y);
//		LCD_DrawLine((coords+4*floorIndex+1)->x, (coords+4*floorIndex+1)->y, (coords+4*floorIndex+3)->x, (coords+4*floorIndex+3)->y);
		Buffer_DrawLine((coords+4*floorIndex+0)->x, (coords+4*floorIndex+0)->y, (coords+4*floorIndex+1)->x, (coords+4*floorIndex+1)->y, buffer, YELLOW);
		Buffer_DrawLine((coords+4*floorIndex+1)->x, (coords+4*floorIndex+1)->y, (coords+4*floorIndex+2)->x, (coords+4*floorIndex+2)->y, buffer, YELLOW);
		Buffer_DrawLine((coords+4*floorIndex+2)->x, (coords+4*floorIndex+2)->y, (coords+4*floorIndex+3)->x, (coords+4*floorIndex+3)->y, buffer, YELLOW);
		Buffer_DrawLine((coords+4*floorIndex+3)->x, (coords+4*floorIndex+3)->y, (coords+4*floorIndex+0)->x, (coords+4*floorIndex+0)->y, buffer, YELLOW);
	}
	
	free(coords);
	free_matrix(projectedMatrix);
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