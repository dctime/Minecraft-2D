#include "level.h"
#include "stdlib.h"
#include "dctimegl.h"

void generateLevel2(Level* level) {
		uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 2, 1},
		{1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1},
		{1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1},
		{0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0}
	};
		
	for (int y = 0; y < FLOOR_HEIGHT; y++) {
		for (int x = 0; x < FLOOR_WIDTH; x++) {
			level->floor[y][x] = temp[y][x];
		}
	}
	
	float temp2[LEVEL2POINTSNUM][2] = {
		{-8, 2},
		{-5, 2},
		{-5, 1},
		{0, 1},
		{0, 4},
		{4, 4},
		{4, 2},
		{7, 2},
		{7, -2},
		{4, -2},
		{4, 1},
		{3, 1},
		{3, 2},
		{1, 2},
		{1, 0},
		{2, 0},
		{2, -2},
		{0, -2},
		{0, -4},
		{-1, -3},
		{-4, -3},
		{-4, -2},
		{-5, -2},
		{-5, 0},
		{-1, 0},
		{-1, -3},
		{0, -4},
		{-6, -4},
		{-6, -3},
		{-7, -3},
		{-7, -2},
		{-8, -2},
		{-8, 2}
	};
	
	level->floorPointsNum = LEVEL2POINTSNUM;
	
	level->floorMatrix = create_matrix(4, LEVEL2POINTSNUM);
	for (int pointIndex = 0; pointIndex < LEVEL2POINTSNUM; pointIndex++) {
		level->floorMatrix->data[level->floorMatrix->cols*0+pointIndex] = (temp2[pointIndex][0]);
		level->floorMatrix->data[level->floorMatrix->cols*1+pointIndex] = (temp2[pointIndex][1]);
		level->floorMatrix->data[level->floorMatrix->cols*2+pointIndex] = 0;
		level->floorMatrix->data[level->floorMatrix->cols*3+pointIndex] = 1;
	}
	
	float tempGoalVec[GOALPOINTSNUM][2] = {
		{5, 1},
		{6, 1},
		{6, 0},
		{5, 0}
	};
	
	level->goalMatrix = create_matrix(4, GOALPOINTSNUM);
	for (int i = 0; i < GOALPOINTSNUM; i++) {
		level->goalMatrix->data[level->goalMatrix->cols*0+i] = tempGoalVec[i][0];
		level->goalMatrix->data[level->goalMatrix->cols*1+i] = tempGoalVec[i][1];
		level->goalMatrix->data[level->goalMatrix->cols*2+i] = 0;
		level->goalMatrix->data[level->goalMatrix->cols*3+i] = 1;
	}
	
	level->buttonCount = 1;
	level->buttons = malloc(sizeof(struct LevelButton) * level->buttonCount);
	int* floorPoints = malloc(sizeof(int) * 2);
	*floorPoints = 1;
	*(floorPoints+1) = -1;
	initLevelButton(level->buttons, 1, -1, floorPoints, level->buttonCount);
	free(floorPoints);

//	debugText(level->buttons->levelButtonMatrix->data[4*0+0]);
//	debugText(level->buttons->levelButtonMatrix->data[4*1+0]);
//	debugText(666);
}

void generateLevel1(Level* level) {
	uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
	};
	
	for (int y = 0; y < FLOOR_HEIGHT; y++) {
		for (int x = 0; x < FLOOR_WIDTH; x++) {
			level->floor[y][x] = temp[y][x];
		}
	}
	
	float temp2[LEVEL1POINTSNUM][2] = {
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
	
	level->floorPointsNum = LEVEL1POINTSNUM;
	
	level->floorMatrix = create_matrix(4, LEVEL1POINTSNUM);
	for (int pointIndex = 0; pointIndex < LEVEL1POINTSNUM; pointIndex++) {
		level->floorMatrix->data[level->floorMatrix->cols*0+pointIndex] = (temp2[pointIndex][0]);
		level->floorMatrix->data[level->floorMatrix->cols*1+pointIndex] = (temp2[pointIndex][1]);
		level->floorMatrix->data[level->floorMatrix->cols*2+pointIndex] = 0;
		level->floorMatrix->data[level->floorMatrix->cols*3+pointIndex] = 1;
	}
	
	float tempGoalVec[GOALPOINTSNUM][2] = {
		{2, -1},
		{3, -1},
		{3, -2},
		{2, -2}
	};
	
	level->goalMatrix = create_matrix(4, GOALPOINTSNUM);
	for (int i = 0; i < GOALPOINTSNUM; i++) {
		level->goalMatrix->data[level->goalMatrix->cols*0+i] = tempGoalVec[i][0];
		level->goalMatrix->data[level->goalMatrix->cols*1+i] = tempGoalVec[i][1];
		level->goalMatrix->data[level->goalMatrix->cols*2+i] = 0;
		level->goalMatrix->data[level->goalMatrix->cols*3+i] = 1;
	}
	
	level->buttonCount = 0;
}

int getLevelTile(Level* level, int x, int y) {
	// debugText(level->floor[-y+3][x+8]);
	return level->floor[-y+3][x+8];
}

bool posValid(Level* level, int x1, int x2, int y1, int y2) {
	if (getLevelTile(level, x1, y1) == 0) return false;
	if (getLevelTile(level, x2, y2) == 0) return false;
	return true;
}

bool posWinning(Level* level, int x1, int x2, int y1, int y2) {
	if (getLevelTile(level, x1, y1) == 2 && getLevelTile(level, x2, y2) == 2) return true;
	// debugText(-1);
	return false;
}

Level* initLevel() {
	Level* level = (struct Level* ) malloc(sizeof(struct Level));
	return level;
}

void freeLevel(Level* level) {
	free(level->buttons);
	free_matrix(level->floorMatrix);
	free_matrix(level->goalMatrix);
	free(level);
}

void initLevelButton(LevelButton* button, int levelX, int levelY, int* floorPoints, int floorCount) {
	button->triggered = false;
	button->levelX = levelX;
	button->levelY = levelY;
	button->levelButtonMatrix = create_matrix(4, 4);
	button->levelTriggerFloorMatrix = create_matrix(4, floorCount);
	
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
	
	for (int floorPointIndex = 0; floorPointIndex < floorCount; floorPointIndex++) {
		button->levelTriggerFloorMatrix->data[4*0+floorPointIndex] = *(floorPoints++);
		button->levelTriggerFloorMatrix->data[4*1+floorPointIndex] = *(floorPoints++);
		button->levelTriggerFloorMatrix->data[4*2+floorPointIndex] = 0;
		button->levelTriggerFloorMatrix->data[4*3+floorPointIndex] = 1;
	}
}

void freeLevelButtonMatrices(LevelButton* button) {
	free_matrix(button->levelButtonMatrix);
	free_matrix(button->levelTriggerFloorMatrix);
}