#ifndef LEVELBUTTON
#define LEVELBUTTON

#include "matrix.h"
#include <stdbool.h>
#include "level.h"
#include "dctimegl.h"
#include "playerRect.h"

typedef struct LevelButton {
	int levelX;
	int levelY;
	Matrix* levelButtonMatrix;
	bool isOn;
	Matrix* levelTriggerFloorMatrix;
	int floorCount;
	bool levelButtonAlreadyPressed;
} LevelButton;

void initLevelButton(LevelButton* button, int levelX, int levelY, int* floorPoints, int floorCount);
void freeLevelButtonMatrices(LevelButton* button);
void levelButtonTick(LevelButton* button, RectPlayer* player);
	
#endif