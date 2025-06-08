#ifndef DCTIMEGL_H
#define DCTIMEGL_H

#include "matrix.h"

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
	double x, y;
}ScreenCoord;

#endif