#include "game.h"
#include <stdlib.h>
#include "stm324xg_lcd_sklin.h"
#include "stdbool.h"

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

void initPlayer(RectPlayer* player, int startLocX, int startLocY) {
	player->modelMatrix = create_matrix(4, PLAYERRECTPOINTNUM);
	if (player->modelMatrix == NULL) {
		while(1);
	}
	
	
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

void freeLevel(Level* level) {
	free(level);
}

void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotY, double tZ, double fov, double near, double far) {
	// level screen coord must be freed first
	Matrix* scaledMatrix = scaleMatrix(player->modelMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(scaledMatrix, rotX/360.0*2.0*M_PI);
	free_matrix(scaledMatrix);
	Matrix* rotatedMatrixAxisY = rotateMatrixAxisY(rotatedMatrixAxisX, rotY/360.0*2.0*M_PI);
	free_matrix(rotatedMatrixAxisX);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisY, 0, 0, tZ);
	free_matrix(rotatedMatrixAxisY);
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

void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotY, double tZ, double fov, double near, double far) {
	// level screen coord must be freed first
	Matrix* scaledMatrix = scaleMatrix(level->floorMatrix, scale, scale, scale);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(scaledMatrix, rotX/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisY = rotateMatrixAxisY(rotatedMatrixAxisX, rotY/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisY, 0, 0, tZ);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, fov/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, near, far);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisY);
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

void play() {
	Buffer* buffer = createBuffer();
	Level* level = initLevel();
	RectPlayer player;
	initPlayer(&player, 0, 0);
	
	generateLevel1(level);
	
	clearBuffer(BLACK, buffer);
	projectPlayerRectToBuffer(&player, buffer, 0.5, -120, 10, 8, 50, 0.1, 100.0);
	projectLevelToBuffer(level, buffer, 0.5, -120, 10, 8, 50, 0.1, 100.0);
	drawBuffer(buffer);
	
	freeLevel(level);
	freeBuffer(buffer);

	while(1);
}
	