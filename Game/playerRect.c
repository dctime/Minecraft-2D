#include "playerRect.h"
#include "dctimegl.h"
#include "stm324xg_lcd_sklin.h"

void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far) {
	// level screen coord must be freed first
	Matrix* translatedMatrixXY;
	
	if (player->levelPosX1 == player->levelPosX2 && player->levelPosY1 == player->levelPosY2) {
		translatedMatrixXY = translateMatrix(player->modelMatrix, player->levelPosX1, player->levelPosY1, 0);

	} else if (player->levelPosX1 > player->levelPosX2) { // left flip 0, 0, -1, 0
		Matrix* laydownMatrixY = rotateMatrixAxisY(player->modelMatrix, -90.0/360*2.0*M_PI);
		translatedMatrixXY = translateMatrix(laydownMatrixY, player->levelPosX1+1, player->levelPosY1, 0);
		free_matrix(laydownMatrixY);
	} else if (player->levelPosX1 < player->levelPosX2) { // right flip 0, 0, 1, 0
		Matrix* tempMoveX = translateMatrix(player->modelMatrix, -1, 0, 0);
		Matrix* laydownMatrixY = rotateMatrixAxisY(tempMoveX, 90.0/360.0*2.0*M_PI);
		free_matrix(tempMoveX);
		translatedMatrixXY = translateMatrix(laydownMatrixY, player->levelPosX1, player->levelPosY1, 0);
		free_matrix(laydownMatrixY);
	} else if (player->levelPosY1 > player->levelPosY2) { // 0, 0, 0, -1 down flip
		Matrix* laydownMatrixX = rotateMatrixAxisX(player->modelMatrix, 90.0/360.0*2.0*M_PI);
		translatedMatrixXY = translateMatrix(laydownMatrixX, player->levelPosX1, player->levelPosY1+1, 0);
		free_matrix(laydownMatrixX);
	} else if (player->levelPosY1 < player->levelPosY2) { // 0, 0, 0, 1 up flip
		Matrix* tempMoveY = translateMatrix(player->modelMatrix, 0, -1, 0);
		Matrix* laydownMatrixX = rotateMatrixAxisX(tempMoveY, -90.0/360.0*2.0*M_PI);
		free_matrix(tempMoveY);
		translatedMatrixXY = translateMatrix(laydownMatrixX, player->levelPosX1, player->levelPosY1, 0);
		free_matrix(laydownMatrixX);
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
}

void initPlayer(RectPlayer* player, int startLocX, int startLocY) {
	player->modelMatrix = create_matrix(4, PLAYERRECTPOINTNUM);
	if (player->modelMatrix == NULL) {
		while(1);
	}
	player->levelPosX1 = 0;
	player->levelPosY1 = 0;
	// direction of the laydown
	player->levelPosX2 = 0;
	player->levelPosY2 = 0;
	
	
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