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

void freeLevel(Level* level) {
	free(level);
}

void projectLevelToBuffer(Level* level, Buffer* buffer) {
	// level screen coord must be freed first
	Matrix* scaledMatrix = scaleMatrix(level->floorMatrix, 0.5, 0.5, 0.5);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(scaledMatrix, 60.0/360.0*2.0*M_PI);
	Matrix* rotatedMatrixAxisY = rotateMatrixAxisY(rotatedMatrixAxisX, 10.0/360.0*2.0*M_PI);
	Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisY, 0, 0, 8);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, 50.0/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, 0.1, 100.0);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(translatedMatrix);
	free_matrix(rotatedMatrixAxisX);
	
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
	
	generateLevel1(level);
	
	clearBuffer(BLACK, buffer);
	projectLevelToBuffer(level, buffer);
	drawBuffer(buffer);

	while(1);
}
	