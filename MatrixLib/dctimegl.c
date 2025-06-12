#include "matrix.h"
#include "math.h"
#include "dctimegl.h"
#include "stdio.h"
#include "stm324xg_lcd_sklin.h"
#include "dctime_lcd.h"
#include <stdbool.h>

Matrix* createPerspectiveProjectionMatrix(double fov_rad, double aspect, double near, double far) {
	Matrix* returnMatrix = create_matrix(4, 4);
	double f = 1.0 / tan(fov_rad/2.0);
	returnMatrix->data[4*0+0]=f/aspect; returnMatrix->data[4*0+1]=0;              returnMatrix->data[4*0+2]=0;              returnMatrix->data[4*0+3]=0;
	returnMatrix->data[4*1+0]=0;          returnMatrix->data[4*1+1]=f;            returnMatrix->data[4*1+2]=0;              returnMatrix->data[4*1+3]=0;
	returnMatrix->data[4*2+0]=0;          returnMatrix->data[4*2+1]=0;              returnMatrix->data[4*2+2]=far/(far-near); returnMatrix->data[4*2+3]=-near*far/(far-near);
	returnMatrix->data[4*3+0]=0;          returnMatrix->data[4*3+1]=0;              returnMatrix->data[4*3+2]=1;              returnMatrix->data[4*3+3]=0;
	
	return returnMatrix;
}

Matrix* createTranslationMatrix(double tx, double ty, double tz) {
	Matrix* returnMatrix = create_matrix(4, 4);
	returnMatrix->data[4*0+0]=1;returnMatrix->data[4*0+1]=0;returnMatrix->data[4*0+2]=0;returnMatrix->data[4*0+3]=tx;
	returnMatrix->data[4*1+0]=0;returnMatrix->data[4*1+1]=1;returnMatrix->data[4*1+2]=0;returnMatrix->data[4*1+3]=ty;
	returnMatrix->data[4*2+0]=0;returnMatrix->data[4*2+1]=0;returnMatrix->data[4*2+2]=1;returnMatrix->data[4*2+3]=tz;
	returnMatrix->data[4*3+0]=0;returnMatrix->data[4*3+1]=0;returnMatrix->data[4*3+2]=0;returnMatrix->data[4*3+3]=1;
	
	return returnMatrix;
}

Matrix* createScaleMatrix(double sx, double sy, double sz) {
	Matrix* returnMatrix = create_matrix(4, 4);
	returnMatrix->data[4*0+0]=sx;returnMatrix->data[4*0+1]=0; returnMatrix->data[4*0+2]=0; returnMatrix->data[4*0+3]=0;
	returnMatrix->data[4*1+0]=0; returnMatrix->data[4*1+1]=sy;returnMatrix->data[4*1+2]=0; returnMatrix->data[4*1+3]=0;
	returnMatrix->data[4*2+0]=0; returnMatrix->data[4*2+1]=0; returnMatrix->data[4*2+2]=sz;returnMatrix->data[4*2+3]=0;
	returnMatrix->data[4*3+0]=0; returnMatrix->data[4*3+1]=0; returnMatrix->data[4*3+2]=0; returnMatrix->data[4*3+3]=1;
	
	return returnMatrix;
}

Matrix* createRotationXMatrix(double alpha) {
	Matrix* returnMatrix = create_matrix(4, 4);
	returnMatrix->data[4*0+0]=1;returnMatrix->data[4*0+1]=0;         returnMatrix->data[4*0+2]=0;          returnMatrix->data[4*0+3]=0;
	returnMatrix->data[4*1+0]=0;returnMatrix->data[4*1+1]=cos(alpha);returnMatrix->data[4*1+2]=-sin(alpha);returnMatrix->data[4*1+3]=0;
	returnMatrix->data[4*2+0]=0;returnMatrix->data[4*2+1]=sin(alpha);returnMatrix->data[4*2+2]=cos(alpha); returnMatrix->data[4*2+3]=0;
	returnMatrix->data[4*3+0]=0;returnMatrix->data[4*3+1]=0;         returnMatrix->data[4*3+2]=0;          returnMatrix->data[4*3+3]=1;
	
	return returnMatrix;
}

Matrix* createRotationYMatrix(double beta) {
	Matrix* returnMatrix = create_matrix(4, 4);
	returnMatrix->data[4*0+0]=cos(beta); returnMatrix->data[4*0+1]=0;returnMatrix->data[4*0+2]=sin(beta); returnMatrix->data[4*0+3]=0;
	returnMatrix->data[4*1+0]=0;         returnMatrix->data[4*1+1]=1;returnMatrix->data[4*1+2]=0;         returnMatrix->data[4*1+3]=0;
	returnMatrix->data[4*2+0]=-sin(beta);returnMatrix->data[4*2+1]=0;returnMatrix->data[4*2+2]=cos(beta); returnMatrix->data[4*2+3]=0;
	returnMatrix->data[4*3+0]=0;         returnMatrix->data[4*3+1]=0;returnMatrix->data[4*3+2]=0;         returnMatrix->data[4*3+3]=1;
	
	return returnMatrix;
}

Matrix* createRotationZMatrix(double gamma) {
	Matrix* returnMatrix = create_matrix(4, 4);
	returnMatrix->data[4*0+0]=cos(gamma); returnMatrix->data[4*0+1]=-sin(gamma);returnMatrix->data[4*0+2]=0; returnMatrix->data[4*0+3]=0;
	returnMatrix->data[4*1+0]=sin(gamma); returnMatrix->data[4*1+1]=cos(gamma); returnMatrix->data[4*1+2]=0; returnMatrix->data[4*1+3]=0;
	returnMatrix->data[4*2+0]=0;          returnMatrix->data[4*2+1]=0;          returnMatrix->data[4*2+2]=1; returnMatrix->data[4*2+3]=0;
	returnMatrix->data[4*3+0]=0;          returnMatrix->data[4*3+1]=0;          returnMatrix->data[4*3+2]=0; returnMatrix->data[4*3+3]=1;
	
	return returnMatrix;
}

Matrix* projectMatrix(Matrix* matrix, double fov_rad, double aspect, double near, double far) {
	Matrix* perspect = createPerspectiveProjectionMatrix(fov_rad, aspect, near, far);
	Matrix* output = matrix_mul(perspect, matrix);
	free_matrix(perspect);
	return output;
}

Matrix* scaleMatrix(Matrix* matrix, double sx, double sy, double sz) {
	Matrix* scale = createScaleMatrix(sx, sy, sz);
	Matrix* output = matrix_mul(scale, matrix);
	free_matrix(scale);
	return output;
}

Matrix* translateMatrix(Matrix* matrix, double tx, double ty, double tz) {
	Matrix* translate = createTranslationMatrix(tx, ty, tz);
	Matrix* output = matrix_mul(translate, matrix);
	free_matrix(translate);
	return output;
}

Matrix* rotateMatrixAxisX(Matrix* matrix, double rad) {
	Matrix* rotateMatrix = createRotationXMatrix(rad);
	Matrix* output = matrix_mul(rotateMatrix, matrix);
	free_matrix(rotateMatrix);
	return output;
}

Matrix* rotateMatrixAxisY(Matrix* matrix, double rad) {
	Matrix* rotateMatrix = createRotationYMatrix(rad);
	Matrix* output = matrix_mul(rotateMatrix, matrix);
	free_matrix(rotateMatrix);
	return output;
}

Matrix* rotateMatrixAxisZ(Matrix* matrix, double rad) {
	Matrix* rotateMatrix = createRotationZMatrix(rad);
	Matrix* output = matrix_mul(rotateMatrix, matrix);
	free_matrix(rotateMatrix);
	return output;
}

void processProjectedMatrix(Matrix* matrix) {
	for (int columnIndex = 0; columnIndex < matrix->cols; columnIndex++) {
		double w = matrix->data[3*matrix->cols+columnIndex];
// 		debugText(w);
		matrix->data[0*matrix->cols+columnIndex] = matrix->data[0*matrix->cols+columnIndex] / w;
		matrix->data[1*matrix->cols+columnIndex] = matrix->data[1*matrix->cols+columnIndex] / w;
		matrix->data[2*matrix->cols+columnIndex] = matrix->data[2*matrix->cols+columnIndex] / w;
		matrix->data[3*matrix->cols+columnIndex] = 1;
	}
}

struct ScreenCoord getCoordFromMatrix(int columnIndex, int screenWidth, int screenHeight, Matrix* matrix) {
	struct ScreenCoord coord;
//	debugText(matrix->data[0*matrix->cols+columnIndex]);
	coord.x = matrix->data[0*matrix->cols+columnIndex]*(screenWidth/2.0)+(screenWidth/2.0);
//	debugText(matrix->data[1*matrix->cols+columnIndex]);
	coord.y = -matrix->data[1*matrix->cols+columnIndex]*(screenHeight/2.0)+(screenHeight/2.0);
	coord.z = matrix->data[2*matrix->cols+columnIndex];
	
	return coord;
}

void debugText(double n) {
		char c[10];
//	
		if (sprintf(c, "%f", n)) {
			LCD_SaveFont();
			LCD_SaveColors();
			LCD_SetFont(&Font16);	
			LCD_SetColors(RED, WHITE); // Text = red; back = white
			LCD_DisplayStringLineCol(12, 2, c);
			LCD_RestoreColors();
			LCD_RestoreFont();
		}

}

uint16_t max(int x, int y) {
	return (((x) > (y)) ? (x) : (y));
}

uint16_t min(int x, int y) {
	return (((x) < (y)) ? (x) : (y));
}


uint16_t coordZsToRGB565(double z1, double z2) {
	return RGB332ToRGB565(RGB332GrayScale((z1 + z2)/2));
}


bool insideTriangle(int x, int y, ScreenCoord coord1, ScreenCoord coord2, ScreenCoord coord3, int v12x, int v12y, int v23x, int v23y, int v31x, int v31y) {
	// [0, 1] a
	// [1, 0] b
	// det -1
	
	int v1px, v1py, v2px, v2py, v3px, v3py;
	v1px = x - coord1.x;
	v1py = y - coord1.y;
	v2px = x - coord2.x;
	v2py = y - coord2.y;
	v3px = x - coord3.x;
	v3py = y - coord3.y;
	
	int det1, det2, det3;
	det1 = v12x*v1py - v12y*v1px;
	det2 = v23x*v2py - v23y*v2px;
	det3 = v31x*v3py - v31y*v3px;
	
	if (det1 >= 0 && det2 >= 0 && det3 >= 0) {
		return true;
	}
	return false;
}


