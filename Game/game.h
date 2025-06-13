#ifndef GAME
#define GAME
#include <stdint.h>
#include "dctimegl.h"
#include "touch_module.h"


#define FLOOR_WIDTH 15
#define FLOOR_HEIGHT 8
#define LEVEL1POINTSNUM 19

#define BUTTON_WIDTH 60
#define BUTTON_HEIGHT 30

#define M_PI 3.14159265358979323846
	
typedef struct Level {
	// 15*8
	uint8_t floor[FLOOR_HEIGHT][FLOOR_WIDTH];
	uint8_t midX;
	uint8_t midY;
	Matrix* floorMatrix; // stores render 3d coords
	
} Level;

typedef struct WorldLoc {
	float x;
	float y;
} WorldLoc;

typedef struct Button {
	float x0, y0;
	float wX, wY;
	// private
	bool isTouchingScreenLastTick;
	bool firstTickInButton;
	int lastTouchX;
	int lastTouchY;
	// public
	bool isHolding;
	bool triggered; 
} Button;

void play();

Level* initLevel();
void freeLevel(Level* level);

void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);

#endif