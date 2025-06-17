#ifndef LEVEL
#define LEVEL

#include <stdbool.h>
#include <stdint.h>
#include <matrix.h>


#define FLOOR_WIDTH 15
#define FLOOR_HEIGHT 8
#define LEVEL1POINTSNUM 19
#define LEVEL2POINTSNUM 33
#define GOALPOINTSNUM 4

struct LevelButton;

typedef struct Level {
	// 15*8
	uint8_t floor[FLOOR_HEIGHT][FLOOR_WIDTH];
	uint8_t midX;
	uint8_t midY;
	Matrix* floorMatrix; // stores render 3d coords
	Matrix* goalMatrix; 
	int floorPointsNum;
	struct LevelButton* buttons;
	int buttonCount;
} Level;

void generateLevel2(Level* level);
void generateLevel1(Level* level);
int getLevelTile(Level* level, int x, int y);
bool posValid(Level* level, int x1, int x2, int y1, int y2);
bool posWinning(Level* level, int x1, int x2, int y1, int y2);
Level* initLevel();
void freeLevel(Level* level);

typedef struct LevelButton {
	int levelX;
	int levelY;
	Matrix* levelButtonMatrix;
	bool triggered;
	Matrix* levelTriggerFloorMatrix;
} LevelButton;

void initLevelButton(LevelButton* button, int levelX, int levelY, int* floorPoints, int floorCount);
void freeLevelButtonMatrices(LevelButton* button);

#endif