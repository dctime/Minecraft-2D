#ifndef DCTIMEGL_H
#define DCTIMEGL_H

#include "matrix.h"
#include <stdbool.h>
#include <stdint.h>
#include "dctime_lcd.h"

Matrix* createPerspectiveProjectionMatrix(double fov_rad, double aspect, double near, double far);
Matrix* createTranslationMatrix(double tx, double ty, double tz);
Matrix* createScaleMatrix(double sx, double sy, double sz);
Matrix* createRotationXMatrix(double alpha);
Matrix* createRotationYMatrix(double beta);
Matrix* createRotationZMatrix(double gamma);
void processProjectedMatrix(Matrix* matrix);
struct ScreenCoord getCoordFromMatrix(int columnIndex, int screenWidth, int screenHeight, Matrix* matrix);

Matrix* projectMatrix(Matrix* matrix, double fov_rad, double aspect, double near, double far);
Matrix* scaleMatrix(Matrix* matrix, double sx, double sy, double sz);
Matrix* rotateMatrixAxisX(Matrix* matrix, double rad);
Matrix* rotateMatrixAxisY(Matrix* matrix, double rad);
Matrix* rotateMatrixAxisZ(Matrix* matrix, double rad);
Matrix* translateMatrix(Matrix* matrix, double tx, double ty, double tz);

void debugText(double n);

typedef struct ScreenCoord {
	// x, y 0 to LCD_Width, LCD_Height
	// z 0 to 1
	float x, y, z;
}ScreenCoord;

bool insideTriangle(int x, int y, ScreenCoord coord1, ScreenCoord coord2, ScreenCoord coord3, int v12x, int v12y, int v23x, int v23y, int v31x, int v31y);
uint16_t coordZsToRGB565(double z1, double z2);

#endif