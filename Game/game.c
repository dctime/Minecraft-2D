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
	
	double temp2[LEVEL1POINTSNUM][2] = {
		{1, 1},
		{-1, -1},
		{1, -1}
//		{-5, 3},
//		{-2, 3},
//		{-2, 2},
//		{1, 2},
//		{1, 1},
//		{4, 1},
//		{4, 0},
//		{5, 0},
//		{5, -2},
//		{4, -2},
//		{4, -3},
//		{1, -3},
//		{1, -2},
//		{0, -2},
//		{0, -1},
//		{-4, -1},
//		{-4, 0},
//		{-5, 0},
//		{-5, -3}
	};
	
	level->floorMatrix = create_matrix(4, LEVEL1POINTSNUM);
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM; pointIndex++) {
		level->floorMatrix->data[level->floorMatrix->cols*0+pointIndex] = (double) (temp2[pointIndex][0]);
		level->floorMatrix->data[level->floorMatrix->cols*1+pointIndex] = (double) (temp2[pointIndex][1]);
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
	Matrix* scaledMatrix = scaleMatrix(level->floorMatrix, 0.05, 0.05, 0.05);
	Matrix* translatedMatrix = translateMatrix(scaledMatrix, 0, 0, 10);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, 70.0/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, 0.1, 100.0);
	processProjectedMatrix(projectedMatrix);
	
	free_matrix(scaledMatrix);
	free_matrix(translatedMatrix);
	
	level->screenCoords = (ScreenCoord*) malloc(sizeof(struct ScreenCoord) * level->floorMatrix->cols);
	level->screenCoordsSize = level->floorMatrix->cols;
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		level->screenCoords[columnIndex] = coord;
		debugText(coord.x);
		debugText(coord.y);
	}
}

void drawLevelToBuffer(Level* level, Buffer* buffer) {
	for (int pointIndex = 0; pointIndex < level->screenCoordsSize-1; pointIndex++) {
		if (level->screenCoords[pointIndex].z < 0 || level->screenCoords[pointIndex].z > 1) continue;
		if (level->screenCoords[pointIndex+1].z < 0 || level->screenCoords[pointIndex+1].z > 1) continue;
		ScreenCoord* points = level->screenCoords;
		int i = pointIndex;

//		debugText(points[i+1].x);
//		debugText(points[i+1].y);
//		debugText(666);
		Buffer_DrawLine(points[i].x, points[i].y, points[i+1].x, points[i+1].y, buffer, WHITE);
	}
}

void play() {
	Buffer* buffer = createBuffer();
//	Level* level = initLevel();
//	generateLevel1(level);
	Matrix* matrix = create_matrix(4, 3);
	double temp2[3][2] = {
		{1, 1},
		{-1, -1},
		{1, -1}
	};
	
	for (int pointIndex = 0; pointIndex < 3; pointIndex++) {
		matrix->data[matrix->cols*0+pointIndex] = (temp2[pointIndex][0]);
		matrix->data[matrix->cols*1+pointIndex] = (temp2[pointIndex][1]);
		matrix->data[matrix->cols*2+pointIndex] = 1;
		matrix->data[matrix->cols*3+pointIndex] = 1;
	}
	
	Matrix* scaledMatrix = scaleMatrix(matrix, 0.3, 0.3, 0.3);
	Matrix* translatedMatrix = translateMatrix(scaledMatrix, 0, 0, 2);
	Matrix* projectedMatrix = projectMatrix(translatedMatrix, 70.0/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, 0.1, 100.0);
	processProjectedMatrix(projectedMatrix);
	
	struct ScreenCoord points[3];
	
	for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
		struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
		points[columnIndex] = coord;
		debugText(coord.x);
//		debugText(coord.y);
		if (coord.x < 0 || coord.x > LCD_Width) continue;
		if (coord.y < 0 || coord.y > LCD_Height) continue;
	}
//	
////	projectLevelToScreenCoords(level);
	clearBuffer(BLACK, buffer);
	drawBuffer(buffer);
////	drawLevelToBuffer(level, buffer);
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM-1; pointIndex++) {
		if (points[pointIndex].z < 0 || points[pointIndex].z > 1) continue;
		if (points[pointIndex+1].z < 0 || points[pointIndex+1].z > 1) continue;

		int i =  pointIndex;

//		debugText(points[i+1].x);
//		debugText(points[i+1].y);
//		debugText(666);
//		Buffer_DrawLine(points[i].x, points[i].y, points[i+1].x, points[i+1].y, buffer, WHITE);
		LCD_SetTextColor(WHITE);
		LCD_DrawLine(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
	}
//		
	
	while(1);
	free_matrix(scaledMatrix);
	free_matrix(translatedMatrix);
}
	