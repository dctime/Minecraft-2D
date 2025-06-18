#ifndef UIBUTTON
#define UIBUTTON

#include <stdbool.h>
#include "playerRect.h"
#include "level.h"

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
	void (*triggerFunc)(struct RectPlayer*, struct Level*);
} Button;

void initButton(Button* button, void (*triggerFunc)(RectPlayer*, Level*));
void buttonToBuffer(Button* button, Buffer* buffer);
void buttonTick(Button* button, TS_StateTypeDef* state, RectPlayer* player, Level* level);
void upButtonTrigger(RectPlayer* player, Level* level);
void downButtonTrigger(RectPlayer* player, Level* level);
void leftButtonTrigger(RectPlayer* player, Level* level);
void rightButtonTrigger(RectPlayer* player, Level* level);


#endif