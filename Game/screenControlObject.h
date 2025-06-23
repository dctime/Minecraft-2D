#ifndef SCREEN_CONTROL_OBJECT
#define SCREEN_CONTROL_OBJECT

#include <stdbool.h>
#include "uiButton.h"

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

void initScreenControlObject(ScreenControlObject* object);
void screenControlTick(ScreenControlObject* object, TS_StateTypeDef* state, Button buttons[4]);
void screenHolding(ScreenControlObject* object, int currentX, int currentY);

#endif