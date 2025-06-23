#ifndef GAME
#define GAME
#include <stdint.h>
#include "dctimegl.h"
#include "touch_module.h"
#include "playerRect.h"
#include "level.h"

#define BUTTON_WIDTH 60
#define BUTTON_HEIGHT 30

#define M_PI 3.14159265358979323846

struct RectPlayer;

Level* initLevel();
void freeLevel(Level* level);

bool inRect(int touchX, int touchY, int x0, int y0, int deltaX, int deltaY);
bool play(void (*genLevelFunc)(Level*));

int getTotalUsedStep();
void accTotalUsedStep(int levelUsedStep);
void resetTotalUsedStep();
bool trySetLeastTotalUsedStep(int stepsCount);
int getLeastTotalUsedStep();

#endif