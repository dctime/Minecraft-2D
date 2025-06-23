#include "screenControlObject.h"

void initScreenControlObject(ScreenControlObject* object) {
	object->firstTickOnScreen = false;
	object->isTouchingScreenLastTick = false;
	object->lastTouchX = -1;
	object->lastTouchY = -1;
	object->isHolding = false;
	object->triggered = false;
	object->screenRotZDeg = 30;
}

void screenControlTick(ScreenControlObject* object, TS_StateTypeDef* state, Button buttons[4]) {
	object->triggered = false;
	if (state->TouchDetected) {
		int x = state->x;
		int y = state->y;
		
		bool notInButton = true;
		for (int buttonID = 0; buttonID < 4; buttonID++) {
			Button* button = buttons+buttonID;
			if (inRect(x, y, button->x0, button->y0, button->wX, button->wY)) notInButton = false;
		}
		
		if (notInButton) {
			if (!object->isTouchingScreenLastTick) {
				object->firstTickOnScreen = true;
				object->lastTouchX = -1;
				object->lastTouchY = -1;
			}
		}
		if (object->firstTickOnScreen) {
			// holding func
			object->isHolding = true;
			// screen moving
			screenHolding(object, x, y);
			
		}
		object->isTouchingScreenLastTick = true;
		object->lastTouchX = x;
		object->lastTouchY = y;
	} else {
		object->isHolding = false;
		if (object->firstTickOnScreen) {
			bool notInButton = true;
			for (int buttonID = 0; buttonID < 4; buttonID++) {
				Button* button = buttons+buttonID;
				if (inRect(button->lastTouchX, button->lastTouchY, button->x0, button->y0, button->wX, button->wY)) notInButton = false;
			}
			if (notInButton) {
				object->triggered = true;
			}
			object->firstTickOnScreen = false;
			object->lastTouchX = -1;
			object->lastTouchY = -1;
		}
		object->isTouchingScreenLastTick = false;
	}
}

void screenHolding(ScreenControlObject* object, int currentX, int currentY) {
	if (object->lastTouchX == -1 || object->lastTouchY == -1) return;
	int deltaX = currentX - object->lastTouchX;
	object->screenRotZDeg += deltaX;
}