#include "game.h"
#include <stdlib.h>
#include "stm324xg_lcd_sklin.h"

void generateLevel1(Level* level) {
//	uint8_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
//		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
//		{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
//		{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
//		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
//	};
	
//	for (int y = 0; y < FLOOR_HEIGHT; y++) {
//		for (int x = 0; x < FLOOR_WIDTH; x++) {
//			level->floor[y][x] = temp[y][x];
//		}
//	}
	
	double temp2[LEVEL1POINTSNUM][LEVEL1POINTSNUM] = {
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

void freeScreenCoords(Level* level) {
	free(level->screenCoords);
}

void projectLevelToScreenCoords(Level* level) {
	// level screen coord must be freed first
	Matrix* scaledMatrix = scaleMatrix(level->floorMatrix, 0.5, 0.5, 0.5);
	Matrix* translatedMatrix = translateMatrix(scaledMatrix, 0, 0, 10);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, 70.0/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, 0.1, 100.0);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(translatedMatrix);
	
	ScreenCoord coords[LEVEL1POINTSNUM];
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		coords[columnIndex] = coord;
	}
	
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM-1; pointIndex++) {
		LCD_DrawLine(coords[pointIndex].x, coords[pointIndex].y, coords[pointIndex+1].x, coords[pointIndex+1].y);
	}
	
//	level->screenCoords = (ScreenCoord*) malloc(sizeof(struct ScreenCoord) * level->floorMatrix->cols);
//	if (level->screenCoords	== NULL) {
//		debugText(-1);
//		while(1);
//	}
//	
//	level->screenCoordsSize = level->floorMatrix->cols;
//	
//	debugText(level->screenCoordsSize);
//	
//	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
//		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
//		level->screenCoords[columnIndex] = coord;
//		debugText(coord.x);
//		debugText(coord.y);
//	}
}

void drawLevelToBuffer(Level* level, Buffer* buffer) {
	debugText(level->screenCoordsSize);
	debugText(666);
	for (int pointIndex = 0; pointIndex < level->screenCoordsSize-1; pointIndex++) {
		if (level->screenCoords[pointIndex].z < 0 || level->screenCoords[pointIndex].z > 1) continue;
		if (level->screenCoords[pointIndex+1].z < 0 || level->screenCoords[pointIndex+1].z > 1) continue;
		ScreenCoord* points = level->screenCoords;
		int i = pointIndex;

		debugText(points[i].x);
		debugText(points[i+1].y);
		debugText(666);
		LCD_SetTextColor(WHITE);
		LCD_DrawLine(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
	}
}

void play() {
	Buffer* buffer = createBuffer();
	Level* level = initLevel();
	generateLevel1(level);

	clearBuffer(BLACK, buffer);
	drawBuffer(buffer);
	projectLevelToScreenCoords(level);

//	drawLevelToBuffer(level, buffer);


	while(1);
}
	