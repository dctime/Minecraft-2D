#include "level.h"
#include "stdlib.h"
#include "dctimegl.h"
#include "levelButton.h"

enum TileType {
	AIR,
	FLOOR,
	GOAL,
	BTN1F,
	FRAGILE
};

void generateLevel3(Level* level) {
	level->playerStartLocX = -7;
	level->playerStartLocY = -2;
	
	uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0},
		{0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0},
		{1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
		{1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
		{1, 1, 1, 0, 0, 1, 1, 1, 1, 4, 4, 4, 4, 4, 0},
		{1, 1, 1, 0, 0, 1, 1, 1, 1, 4, 4, 4, 4, 4, 0},
		{0, 0, 0, 0, 0, 1, 2, 1, 0, 0, 4, 4, 1, 4, 0},
		{0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 4, 4, 4, 4, 0}
	};
	
	for (int y = 0; y < FLOOR_HEIGHT; y++) {
		for (int x = 0; x < FLOOR_WIDTH; x++) {
			level->floor[y][x] = temp[y][x];
		}
	}
	
	float temp2[LEVEL3POINTSNUM][2] = {
		{-4, 1},
		{-5, 1},
		{-5, -3},
		{-8, -3},
		{-8, 2},
		{-4, 2},
		{-4, 1},
		{-3, -1},
		{-3, -5},
		{0, -5},
		{0, -3},
		{1, -3},
		{1, -1},
		{2, 1},
		{1, 1},
		{1, 2},
		{4, 2},
		{4, -1},
		{5, -3},
		{4, -3},
		{4, -4},
		{5, -4},
		{5, -3},
		{4, -1},
		{2, -1},
		{2, 1},
		{1, -1},
		{-3, -1}
	};
	
	level->floorPointsNum = LEVEL3POINTSNUM;
	
	level->floorMatrix = create_matrix(4, LEVEL3POINTSNUM);
	for (int pointIndex = 0; pointIndex < LEVEL3POINTSNUM; pointIndex++) {
		level->floorMatrix->data[level->floorMatrix->cols*0+pointIndex] = (temp2[pointIndex][0]);
		level->floorMatrix->data[level->floorMatrix->cols*1+pointIndex] = (temp2[pointIndex][1]);
		level->floorMatrix->data[level->floorMatrix->cols*2+pointIndex] = 0;
		level->floorMatrix->data[level->floorMatrix->cols*3+pointIndex] = 1;
	}
	
	level->fragilePointsNum = 14;
	float tempFragile[14][2] = {
		{0, 2},
		{-5, 2},
		{-5, 4},
		{2, 4},
		{2, 2},
		{0, 2},
		{2, -1},
		{6, -1},
		{6, -5},
		{2, -5},
		{2, -3},
		{1, -3},
		{1, -1},
		{2, -1}
	};
	
	level->fragileMatrix = create_matrix(4, level->fragilePointsNum);
	for (int pointIndex = 0; pointIndex < level->fragilePointsNum; pointIndex++) {
		level->fragileMatrix->data[level->fragileMatrix->cols*0+pointIndex] = (tempFragile[pointIndex][0]);
		level->fragileMatrix->data[level->fragileMatrix->cols*1+pointIndex] = (tempFragile[pointIndex][1]);
		level->fragileMatrix->data[level->fragileMatrix->cols*2+pointIndex] = 0;
		level->fragileMatrix->data[level->fragileMatrix->cols*3+pointIndex] = 1;
	}
	
	
	float tempGoalVec[GOALPOINTSNUM][2] = {
		{-2, -3},
		{-1, -3},
		{-1, -4},
		{-2, -4}
	};
	
	level->goalMatrix = create_matrix(4, GOALPOINTSNUM);
	for (int i = 0; i < GOALPOINTSNUM; i++) {
		level->goalMatrix->data[level->goalMatrix->cols*0+i] = tempGoalVec[i][0];
		level->goalMatrix->data[level->goalMatrix->cols*1+i] = tempGoalVec[i][1];
		level->goalMatrix->data[level->goalMatrix->cols*2+i] = 0;
		level->goalMatrix->data[level->goalMatrix->cols*3+i] = 1;
	}
	
	level->buttonCount = 0;
	
	level->fixPairsCount = 8;
	level->fixMatrix = create_matrix(4, level->fixPairsCount*2);
	float fixPointsTemp[8][2] = {
		{-4, 1},
		{-3, -1},
		{1, -1},
		{2, 1},
		{4, -1},
		{5, -3},
		{0, 2},
		{2, -1}
	};
	
	for (int fixPointIndex = 0; fixPointIndex < level->fixPairsCount*2; fixPointIndex++) {
		level->fixMatrix->data[level->fixMatrix->cols*0+fixPointIndex] = fixPointsTemp[fixPointIndex][0];
		level->fixMatrix->data[level->fixMatrix->cols*1+fixPointIndex] = fixPointsTemp[fixPointIndex][1];
		level->fixMatrix->data[level->fixMatrix->cols*2+fixPointIndex] = 0;
		level->fixMatrix->data[level->fixMatrix->cols*3+fixPointIndex] = 1;
	}
}

void generateLevel2(Level* level) {
	level->playerStartLocX = -7;
	level->playerStartLocY = 0;
	
		uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 2, 1},
		{1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1},
		{1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1},
		{0, 1, 1, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
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
		{-5, -3},
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
	*floorPoints = -5;
	*(floorPoints+1) = -3;
	initLevelButton(level->buttons, 1, -1, floorPoints, level->buttonCount);
	free(floorPoints);
	
	level->fixPairsCount = 1;
	level->fixMatrix = create_matrix(4, level->fixPairsCount*2);
	float fixPointsTemp[2][2] = {
		{-1, -3},
		{0, -4}
	};
	for (int fixPointIndex = 0; fixPointIndex < level->fixPairsCount*2; fixPointIndex++) {
		level->fixMatrix->data[level->fixMatrix->cols*0+fixPointIndex] = fixPointsTemp[fixPointIndex][0];
		level->fixMatrix->data[level->fixMatrix->cols*1+fixPointIndex] = fixPointsTemp[fixPointIndex][1];
		level->fixMatrix->data[level->fixMatrix->cols*2+fixPointIndex] = 0;
		level->fixMatrix->data[level->fixMatrix->cols*3+fixPointIndex] = 1;
	}
	
	level->fragilePointsNum = 0;
}

void generateLevel1(Level* level) {
	level->playerStartLocX = -4;
	level->playerStartLocY = 1;
	
	uint16_t temp[FLOOR_HEIGHT][FLOOR_WIDTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
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
	level->fixPairsCount = 0;
	level->fragilePointsNum = 0;
}


int getLevelTile(Level* level, int x, int y) {
	// debugText(level->floor[-y+3][x+8]);
	int tempX = x+8;
	int tempY = -y+3;
	if (tempX < 0 || tempX >= FLOOR_WIDTH) return AIR;
	if (tempY < 0 || tempY >= FLOOR_HEIGHT) return AIR;
	return level->floor[-y+3][x+8];
}

bool isSolidFloor(Level* level, int x, int y) {
	if (getLevelTile(level, x, y) == FLOOR) return true;
	if (getLevelTile(level, x, y) == GOAL) return true;
	if (getLevelTile(level, x, y) == BTN1F && level->buttonCount >= 1 && (level->buttons+1-1)->isOn) return true;
	if (getLevelTile(level, x, y) == FRAGILE) return true;
	return false;
}

bool isFragileAndDangerous(Level* level, int x1, int x2, int y1, int y2) {
	if (getLevelTile(level, x1, y1) == FRAGILE && x1 == x2 && y1 == y2) return true;
	return false;
}

bool posValid(Level* level, int x1, int x2, int y1, int y2) {
	if (isFragileAndDangerous(level, x1, x2, y1, y2)) return false;
	if (!isSolidFloor(level, x1, y1)) return false;
	if (!isSolidFloor(level, x2, y2)) return false;
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
	for (int buttonIndex = 0; buttonIndex < level->buttonCount; buttonIndex++) {
		freeLevelButtonMatrices(level->buttons+buttonIndex);
	}

	free(level->buttons);
	free_matrix(level->floorMatrix);
	free_matrix(level->goalMatrix);
	
	if (level->fragilePointsNum != 0) {
		free_matrix(level->fragileMatrix);
	}
	
	if (level->fixPairsCount != 0) {
		free_matrix(level->fixMatrix);
	}
	free(level);
}



