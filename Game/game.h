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

typedef struct ScreenControlObject {
		// private
	bool isTouchingScreenLastTick;
	bool firstTickOnScreen;
	int lastTouchX;
	int lastTouchY;
	// public
	bool isHolding;
	bool triggered;
	int screenRotZDeg;
} ScreenControlObject;

void play();

Level* initLevel();
void freeLevel(Level* level);

void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
bool inRect(int touchX, int touchY, int x0, int y0, int deltaX, int deltaY);

#endif