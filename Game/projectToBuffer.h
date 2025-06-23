#ifndef PROJECT_TO_BUFFER
#define PROJECT_TO_BUFFER

#include "level.h"
#include "levelButton.h"

void projectGoalToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectLevelToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectLevelFixToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectLevelButtonToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectButtonTriggeredFloorToBuffer(LevelButton* button, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectLevelAllButtonTriggeredFloorToBuffer(Level* level, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);
void projectPlayerRectToBuffer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);

#endif