#include "uiButton.h"
#include "game.h"

void initButton(Button* button, void (*triggerFunc)(RectPlayer*, Level*)) {
	button->firstTickInButton = false;
	button->isTouchingScreenLastTick = false;
	button->lastTouchX = -1;
	button->lastTouchY = -1;
	button->isHolding = false;
	button->triggered = false;
	button->triggerFunc = triggerFunc;
}

void buttonToBuffer(Button* button, Buffer* buffer) {
	if (button->triggered) {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, BLACK);
	} else if (button->isHolding) {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, DARKGRAY);
	} else {
		Buffer_FillRect(button->x0, button->y0, button->wX, button->wY, buffer, GRAY);
	}
	
}

void buttonTick(Button* button, TS_StateTypeDef* state, RectPlayer* player, Level* level) {
	button->triggered = false;
	if (state->TouchDetected) {
		int x = state->x;
		int y = state->y;
		
		if (inRect(x, y, button->x0, button->y0, button->wX, button->wY)) {
			if (!button->isTouchingScreenLastTick) {
				button->firstTickInButton = true;
			}
		}
		if (button->firstTickInButton) {
			// holding func
			button->isHolding = true;
		}
		button->isTouchingScreenLastTick = true;
		button->lastTouchX = x;
		button->lastTouchY = y;
	} else {
		button->isHolding = false;
		if (button->firstTickInButton) {
			if (inRect(button->lastTouchX, button->lastTouchY, button->x0, button->y0, button->wX, button->wY) && player->aniAngleProcess == 0) {
				button->triggerFunc(player, level);
				button->triggered = true;
			}
			button->firstTickInButton = false;
		}
		button->isTouchingScreenLastTick = false;
	}
}

void upButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;
	
	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetY1 += 1;
		targetY2 += 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		int targetY;
		if (targetY1 > targetY2) {
			targetY = targetY1+1;
		} else {
			targetY = targetY2+1;
		}
		
		targetY1 = targetY;
		targetY2 = targetY;
	} else if (targetY1 == targetY2) {
		// lay left/right
		targetY1 += 1;
		targetY2 += 1;
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = UP;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
		player->usedStep += 1;
	}
}

void downButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetY1 -= 1;
		targetY2 -= 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		if (targetY1 > targetY2) {
			targetY1 -= 2;
			targetY2 -= 1;
		} else {
			targetY1 -= 1;
			targetY2 -= 2;
		}
	} else if (targetY1 == targetY2) {
		// lay left/right
		targetY1 -= 1;
		targetY2 -= 1;
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = DOWN;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
		player->usedStep += 1;
	}
}

void leftButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetX1 -= 1;
		targetX2 -= 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		targetX1 -= 1;
		targetX2 -= 1;
	} else if (targetY1 == targetY2) {
		// lay left/right
		if (targetX1 < targetX2) {
			targetX1 -= 1;
			targetX2 -= 2;
		} else {
			targetX2 -= 1;
			targetX1 -= 2;
		}
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = LEFT;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
		player->usedStep += 1;
	}
}

void rightButtonTrigger(RectPlayer* player, Level* level) {
	int targetX1 = player->levelPosX1, targetX2 = player->levelPosX2, targetY1 = player->levelPosY1, targetY2 = player->levelPosY2;

	if (targetX1 == targetX2 && targetY1 == targetY2) {
		targetX1 += 1;
		targetX2 += 2;
	} else if (targetX1 == targetX2 && targetY1 != targetY2) {
		// lay up/down
		targetX1 += 1;
		targetX2 += 1;
	} else if (targetY1 == targetY2) {
		// lay left/right
		if (targetX1 < targetX2) {
			targetX1 += 2;
			targetX2 += 1;
		} else {
			targetX2 += 2;
			targetX1 += 1;
		}
	}
	
	if (posValid(level, targetX1, targetX2, targetY1, targetY2)) {
		player->lastControl = RIGHT;
		player->levelPosX1 = targetX1;
		player->levelPosX2 = targetX2;
		player->levelPosY1 = targetY1;
		player->levelPosY2 = targetY2;
		player->aniAngleProcess = 90;
		player->usedStep += 1;
	}
}