#ifndef GAME
#define GAME
#include <stdint.h>
#include "dctimegl.h"


#define FLOOR_WIDTH 15
#define FLOOR_HEIGHT 8
#define LEVEL1POINTSNUM 19
#define PLAYERRECTPOINTNUM 8
#define M_PI 3.14159265358979323846
	
typedef struct Level {
	// 15*8
	uint8_t floor[FLOOR_HEIGHT][FLOOR_WIDTH];
	uint8_t midX;
	uint8_t midY;
	Matrix* floorMatrix; // stores render 3d coords
	
} Level;

typedef struct RectPlayer {
	Matrix* modelMatrix;
} RectPlayer;

void initPlayer(RectPlayer* player, int startLocX, int startLocY);
void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotY, double tZ, double fov, double near, double far);

typedef struct WorldLoc {
	float x;
	float y;
} WorldLoc;

void play();

Level* initLevel();
void freeLevel(Level* level);

void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotY, double tZ, double fov, double near, double far);

#endif