#include "playerRect.h"
#include "dctimegl.h"
#include "stm324xg_lcd_sklin.h"

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

}