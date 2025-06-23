#include "projectToBuffer.h"

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
void projectLevelButtonToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	if (level->buttonCount == 0) return;
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

void projectLevelAllButtonTriggeredFloorToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	for (int levelButtonIndex = 0; levelButtonIndex < level->buttonCount; levelButtonIndex++) {
		if ((level->buttons+levelButtonIndex)->isOn)
			projectButtonTriggeredFloorToBuffer(level->buttons+levelButtonIndex, buffer, scale, rotX, rotZ, tZ, fov, near, far);
	}
}

void standToLayLeft(RectPlayer* player, Matrix** translatedMatrixXY);
void layLeftToStand(RectPlayer* player, Matrix** translatedMatrixXY);
void standToLayRight(RectPlayer* player, Matrix** translatedMatrixXY);
void layRightToStand(RectPlayer* player, Matrix** translatedMatrixXY);
void standToLayDown(RectPlayer* player, Matrix** translatedMatrixXY);
void layDownToStand(RectPlayer* player, Matrix** translatedMatrixXY);
void standToLayUp(RectPlayer* player, Matrix** translatedMatrixXY);
void layUpToStand(RectPlayer* player, Matrix** translatedMatrixXY);
void layXRollUp(RectPlayer* player, Matrix** translatedMatrixXY);
void layXRollDown(RectPlayer* player, Matrix** translatedMatrixXY);
void layYRollLeft(RectPlayer* player, Matrix** translatedMatrixXY);
void layYRollRight(RectPlayer* player, Matrix** translatedMatrixXY);

void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	// level screen coord must be freed first
	Matrix* translatedMatrixXY;
	
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) { // stand still
		if (player->lastControl == RIGHT) {
			layLeftToStand(player, &translatedMatrixXY);
		} else if (player->lastControl == LEFT) {
			layRightToStand(player, &translatedMatrixXY);
		} else if (player->lastControl == UP) {
			layDownToStand(player, &translatedMatrixXY);
		} else if (player->lastControl == DOWN) {
			layUpToStand(player, &translatedMatrixXY);
		} else {
			translatedMatrixXY = translateMatrix(player->modelMatrix, player->levelPosX1, player->levelPosY1, 0);
		}
	} else if (player->levelPosX1 > player->levelPosX2) { // left flip 0, 0, -1, 0
		if (player->lastControl == LEFT) {
			standToLayLeft(player, &translatedMatrixXY);
		} else if (player->lastControl == UP) {
			layXRollUp(player, &translatedMatrixXY);
		} else if (player->lastControl == DOWN) {
			layXRollDown(player, &translatedMatrixXY);
		}
	} else if (player->levelPosX1 < player->levelPosX2) { // right flip 0, 0, 1, 0
		if (player->lastControl == RIGHT) {
			standToLayRight(player, &translatedMatrixXY);
		} else if (player->lastControl == UP) {
			layXRollUp(player, &translatedMatrixXY);
		} else if (player->lastControl == DOWN) {
			layXRollDown(player, &translatedMatrixXY);
		}
	} else if (player->levelPosY1 > player->levelPosY2) { // 0, 0, 0, -1 down flip
		if (player->lastControl == DOWN) {
			standToLayDown(player, &translatedMatrixXY);
		} else if (player->lastControl == LEFT) {
			layYRollLeft(player, &translatedMatrixXY);
		} else if (player->lastControl == RIGHT) {
			layYRollRight(player, &translatedMatrixXY);
		}
		
	} else if (player->levelPosY1 < player->levelPosY2) { // 0, 0, 0, 1 up flip
		if (player->lastControl == UP) {
			standToLayUp(player, &translatedMatrixXY);
		} else if (player->lastControl == LEFT) {
			layYRollLeft(player, &translatedMatrixXY);
		} else if (player->lastControl == RIGHT) {
			layYRollRight(player, &translatedMatrixXY);
		}
	}
	
	Matrix* scaledMatrix = scaleMatrix(translatedMatrixXY, scale, scale, scale);
	free_matrix(translatedMatrixXY);
	Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(scaledMatrix, rotZ/360.0*2.0*M_PI);
	free_matrix(scaledMatrix);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(rotatedMatrixAxisZ, rotX/360.0*2.0*M_PI);	
	free_matrix(rotatedMatrixAxisZ);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisX, 0, 0, tZ);
	free_matrix(rotatedMatrixAxisX);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	free_matrix(translatedMatrix);
	processProjectedMatrix(projectedMatrix);

	
	ScreenCoord coords[PLAYERRECTPOINTNUM];
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		coords[columnIndex] = coord;
	}
	
	uint16_t color = LCD_COLOR_GREEN;
	if (player->lastControl == UP) {
		color = RED;
	} else if (player->lastControl == DOWN) {
		color = GREEN;
	} else if (player->lastControl == LEFT) {
		color = YELLOW;
	} else if (player->lastControl == RIGHT) {
		color = BLUE;
	}
	
	for (int pointIndex = 0; pointIndex < 3; pointIndex++) {
		// LCD_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y);
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y, buffer, color);
	}
	
	for (int pointIndex = 4; pointIndex < 7; pointIndex++) {
//		LCD_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y);
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y, buffer, color);
	}
	
//	LCD_DrawLine(coords[3].x, coords[3].y, coords[0].x, coords[0].y);
//	LCD_DrawLine(coords[4+3].x, coords[4+3].y, coords[4+0].x, coords[4+0].y);
	Buffer_DrawLine(coords[3].x, coords[3].y, coords[0].x, coords[0].y, buffer, color);
	Buffer_DrawLine(coords[4+3].x, coords[4+3].y, coords[4+0].x, coords[4+0].y, buffer, color);
	
	for (int pointIndex = 0; pointIndex < 4; pointIndex++) {
//		LCD_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+4].x, coords[pointIndex+4].y);
		Buffer_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+4].x, coords[pointIndex+4].y, buffer, color);
	}
	
	free_matrix(projectedMatrix);
	player->aniAngleProcess -= 2;
	if (player->aniAngleProcess < 0) player->aniAngleProcess = 0;
}

void standToLayLeft(RectPlayer* player, Matrix** translatedMatrixXY) { // lay left pos
		Matrix* laydownMatrixY = rotateMatrixAxisY(player->modelMatrix, -(90-player->aniAngleProcess)/360.0*2.0*M_PI);
		*translatedMatrixXY = translateMatrix(laydownMatrixY, player->levelPosX1+1, player->levelPosY1, 0);
		free_matrix(laydownMatrixY);
}

void layLeftToStand(RectPlayer* player, Matrix** translatedMatrixXY) { // stand player pos
	Matrix* laydownMatrixY = rotateMatrixAxisY(player->modelMatrix, -(player->aniAngleProcess)/360.0*2.0*M_PI);
	*translatedMatrixXY = translateMatrix(laydownMatrixY, player->levelPosX1, player->levelPosY1, 0);
	free_matrix(laydownMatrixY);
}

void standToLayRight(RectPlayer* player, Matrix** translatedMatrixXY) { // lay right pos
		Matrix* tempMoveX = translateMatrix(player->modelMatrix, -1, 0, 0);
		Matrix* laydownMatrixY = rotateMatrixAxisY(tempMoveX, (90-player->aniAngleProcess)/360.0*2.0*M_PI);
		free_matrix(tempMoveX);
		*translatedMatrixXY = translateMatrix(laydownMatrixY, player->levelPosX1, player->levelPosY1, 0);
		free_matrix(laydownMatrixY);
}

void layRightToStand(RectPlayer* player, Matrix** translatedMatrixXY) { // stand pos
	Matrix* tempMoveX = translateMatrix(player->modelMatrix, -1, 0, 0);
	Matrix* laydownMatrixY = rotateMatrixAxisY(tempMoveX, (player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempMoveX);
	Matrix* tempMoveX2 = translateMatrix(laydownMatrixY, 1, 0, 0);
	free_matrix(laydownMatrixY);
	*translatedMatrixXY = translateMatrix(tempMoveX2, player->levelPosX1, player->levelPosY1, 0);
	free_matrix(tempMoveX2);
}

void standToLayDown(RectPlayer* player, Matrix** translatedMatrixXY) { // laydown pos
		Matrix* laydownMatrixX = rotateMatrixAxisX(player->modelMatrix, (90-player->aniAngleProcess)/360.0*2.0*M_PI);
		*translatedMatrixXY = translateMatrix(laydownMatrixX, player->levelPosX1, player->levelPosY1+1, 0);
		free_matrix(laydownMatrixX);
}

void layDownToStand(RectPlayer* player, Matrix** translatedMatrixXY) { // stand pos
	Matrix* laydownMatrixX = rotateMatrixAxisX(player->modelMatrix, (player->aniAngleProcess)/360.0*2.0*M_PI);
	*translatedMatrixXY = translateMatrix(laydownMatrixX, player->levelPosX1, player->levelPosY1, 0);
	free_matrix(laydownMatrixX);
}

void standToLayUp(RectPlayer* player, Matrix** translatedMatrixXY) { // layup pos
		Matrix* tempMoveY = translateMatrix(player->modelMatrix, 0, -1, 0);
		Matrix* laydownMatrixX = rotateMatrixAxisX(tempMoveY, -(90-player->aniAngleProcess)/360.0*2.0*M_PI);
		free_matrix(tempMoveY);
		*translatedMatrixXY = translateMatrix(laydownMatrixX, player->levelPosX1, player->levelPosY1, 0);
		free_matrix(laydownMatrixX);
}

void layUpToStand(RectPlayer* player, Matrix** translatedMatrixXY) { // stand pos
	Matrix* tempMoveY = translateMatrix(player->modelMatrix, 0, -1, 0);
	Matrix* laydownMatrixX = rotateMatrixAxisX(tempMoveY, -(player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempMoveY);
	Matrix* tempMoveY2 = translateMatrix(laydownMatrixX, 0, 1, 0);
	free_matrix(laydownMatrixX);
	*translatedMatrixXY = translateMatrix(tempMoveY2, player->levelPosX1, player->levelPosY1, 0);
	free_matrix(tempMoveY2);
}

void layXRollUp(RectPlayer* player, Matrix** translatedMatrixXY) { // 0+a, 0+b, -1+a, 0+b or -1, 0, 0, 0
	Matrix* laydownMatrixY = rotateMatrixAxisY(player->modelMatrix, -90/360.0*2.0*M_PI);
	Matrix* tempXYMove = translateMatrix(laydownMatrixY, 1, -1, 0);
	free_matrix(laydownMatrixY);
	Matrix* rollMatrixX = rotateMatrixAxisX(tempXYMove, -(90-player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempXYMove);
	if (player->levelPosX1 > player->levelPosX2) {
		*translatedMatrixXY = translateMatrix(rollMatrixX, player->levelPosX1, player->levelPosY1, 0);
	} else {
		*translatedMatrixXY = translateMatrix(rollMatrixX, player->levelPosX2, player->levelPosY2, 0);
	}
	free_matrix(rollMatrixX);
}

void layXRollDown(RectPlayer* player, Matrix** translatedMatrixXY) { // 0, 0, -1, 0 or -1, 0, 0, 0
	Matrix* laydownMatrixY = rotateMatrixAxisY(player->modelMatrix, -90/360.0*2.0*M_PI);
	Matrix* tempXYMove = translateMatrix(laydownMatrixY, 1, 0, 0);
	free_matrix(laydownMatrixY);
	Matrix* rollMatrixX = rotateMatrixAxisX(tempXYMove, (90-player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempXYMove);
	if (player->levelPosX1 > player->levelPosX2) {
		*translatedMatrixXY = translateMatrix(rollMatrixX, player->levelPosX1, player->levelPosY1+1, 0);
	} else {
		*translatedMatrixXY = translateMatrix(rollMatrixX, player->levelPosX2, player->levelPosY2+1, 0);
	}
	free_matrix(rollMatrixX);
}

void layYRollLeft(RectPlayer* player, Matrix** translatedMatrixXY) { // 0, -1, 0, 0 // 0, 0, 0,-1
	Matrix* laydownMatrixX = rotateMatrixAxisX(player->modelMatrix, 90/360.0*2.0*M_PI);
	Matrix* tempYMove = translateMatrix(laydownMatrixX, 0, 1, 0);
	free_matrix(laydownMatrixX);
	Matrix* rollLeftMatrixY = rotateMatrixAxisY(tempYMove, -(90-player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempYMove);
	if (player->levelPosY1 > player->levelPosY2) {
		*translatedMatrixXY = translateMatrix(rollLeftMatrixY, 1+player->levelPosX1, 0+player->levelPosY1, 0);
	} else {
		*translatedMatrixXY = translateMatrix(rollLeftMatrixY, 1+player->levelPosX2, 0+player->levelPosY2, 0);
	}
	free_matrix(rollLeftMatrixY);
}

void layYRollRight(RectPlayer* player, Matrix** translatedMatrixXY) { // 0, -1, 0, 0 // 0, 0, 0,-1
	Matrix* laydownMatrixX = rotateMatrixAxisX(player->modelMatrix, 90/360.0*2.0*M_PI);
	Matrix* tempXYMove = translateMatrix(laydownMatrixX, -1, 1, 0);
	free_matrix(laydownMatrixX);
	Matrix* rollLeftMatrixY = rotateMatrixAxisY(tempXYMove, (90-player->aniAngleProcess)/360.0*2.0*M_PI);
	free_matrix(tempXYMove);
	if (player->levelPosY1 > player->levelPosY2) {
		*translatedMatrixXY = translateMatrix(rollLeftMatrixY, player->levelPosX1, player->levelPosY1, 0);
	} else {
		*translatedMatrixXY = translateMatrix(rollLeftMatrixY, player->levelPosX2, player->levelPosY2, 0);
	}
	free_matrix(rollLeftMatrixY);
}
