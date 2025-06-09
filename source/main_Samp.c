/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "stm32f4xx.h"
#include "stm324xg_lcd_sklin.h"
#include "user_defined.h"
#include <stdio.h>	// for sprintf
#include "touch_module.h"
#include "dctime_lcd.h"

#define LCD_Width 320
#define LCD_Height 240

void stm32f4_Hardware_Init (void);
void Wait_PressPA0(uint16_t Cnum);

void Driver_GPIO(void);
void Driver_SPIpin_GPIO(void);

void Default_Calibration(void);
void Touchscreen_demo (void);
void Touchscreen_playFig(void);

void Touchscreen_Calibration (void);

void Touch_sample_FreeDraw(void);
void Touch_sample_Sine(void);
void Touch_sample_Hit(void);
void Sample_alarmA(void);

static TS_StateTypeDef  TS_State;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
#define KEY_TIME 10  
 uint16_t keyTimer = KEY_TIME;  

	stm32f4_Hardware_Init();

	/* Initialize the LCD */
	LCD_Init();
	LCD_Clear(GREEN);
	LCD_DisplayOn();

	/* Initialize the Touch module */
	Default_Calibration();
	Driver_GPIO();

	
//	Touchscreen_Calibration();

startup:
	while(GPIOA->IDR & 0x01)			// wait until release KEY1
    delay_ms(20);						// wait 20 msec for debouncing

	LCD_SetFont(&Font20);
	LCD_SetColors(RED, BLUE);
	LCD_DisplayStringAt(36, 140, (char*)" Press KEY1   ", LEFT_MODE);
	LCD_DisplayStringAt(36, 160, (char*)" then Release ", LEFT_MODE);

//=================
	Wait_PressPA0(KEY_TIME);	
	while(GPIOA->IDR & Bit(0));			// wait until release KEY1
//====================================
	
	LCD_Clear(LCD_COLOR_LIGHTBLUE);

	#define JPG_fileAddress	0x08020000
	
	TS_GetState(&TS_State);
	int x = TS_State.x;
	int y = TS_State.y;
	
	LCD_SetTextColor(LCD_COLOR_GREEN);
	LCD_FillRect(0, 0, LCD_Width-10, LCD_Height-10);
	
	uint8_t res;
	res = LCD_DrawJPG(0, 0, (uint8_t *) JPG_fileAddress, 100, 100);
	
	#include "matrix.h"
	#include "dctimegl.h"

	#define M_PI 3.14159265358979323846
	
	#define POINTS_NUM 36
	
	Matrix* cube = create_matrix(4, POINTS_NUM);
	double cubeVertices[8][3] = {
			{-1, -1, -1}, // 0
			{ 1, -1, -1}, // 1
			{ 1,  1, -1}, // 2
			{-1,  1, -1}, // 3
			{-1, -1,  1}, // 4
			{ 1, -1,  1}, // 5
			{ 1,  1,  1}, // 6
			{-1,  1,  1}  // 7
	};
	
	uint8_t cubePoints[POINTS_NUM] = {
		0, 1, 2,
		0, 2, 3,
		5, 6, 2,
		5, 2, 1,
		4, 7, 5,
		5, 7, 6,
		4, 0, 3,
		4, 3, 7,
		3, 2, 6,
		3, 6, 7,
		0, 4, 5,
		0, 5, 1
	};
	
	for (int pointIndex = 0; pointIndex < cube->cols; pointIndex++) {
		cube->data[cube->cols*0+pointIndex] = cubeVertices[cubePoints[pointIndex]][0];
		cube->data[cube->cols*1+pointIndex] = cubeVertices[cubePoints[pointIndex]][1];
		cube->data[cube->cols*2+pointIndex] = cubeVertices[cubePoints[pointIndex]][2];
		cube->data[cube->cols*3+pointIndex] = 1;
	}
	
	int rowsCount = 0;
	int columnCount = 0;

	Matrix* scaledMatrix = scaleMatrix(cube, 0.5, 0.5, 0.5);
	Matrix* rotatedMatrixAxisX = rotateMatrixAxisX(scaledMatrix, M_PI/180.0*45.0);
	Matrix* rotatedMatrixAxisY = rotateMatrixAxisY(rotatedMatrixAxisX, M_PI/180.0*45.0);
	
	double degree = 0;
	
	struct Buffer* buffer = createBuffer();
	LCD_Clear(WHITE);
	while (1) {
		clearBuffer(WHITE, buffer);
		degree += 1;
		Matrix* rotatedMatrixAxisZ = rotateMatrixAxisZ(rotatedMatrixAxisY, M_PI/180.0*degree);
		Matrix* translatedMatrix = translateMatrix(rotatedMatrixAxisZ, 0, 0, 2);
		Matrix* projectedMatrix = projectMatrix(translatedMatrix, 70.0/360.0*2.0*M_PI, LCD_Width/(double)LCD_Height, 0.1, 100.0);
		processProjectedMatrix(projectedMatrix);
		
		// LCD_DrawVLine(50, 50, 50);
		// Buffer_DrawVLine(50, 50, 50, buffer, BLACK);
//		Buffer_FillCircle(100, 100, 10, buffer, BLACK);
		struct ScreenCoord points[POINTS_NUM];
		
		for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
			struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
			points[columnIndex] = coord;
			
			if (coord.x < 0 || coord.x > LCD_Width) continue;
			if (coord.y < 0 || coord.y > LCD_Height) continue;
			Buffer_FillCircle(coord.x, coord.y, 5, buffer, BLACK);
		}
		
		
		for (int i = 0; i < POINTS_NUM; i+=3) {
			Buffer_DrawLine(points[i].x, points[i].y, points[i+1].x, points[i+1].y, buffer, BLACK);
			Buffer_DrawLine(points[i].x, points[i].y, points[i+2].x, points[i+2].y, buffer, BLACK);
			Buffer_DrawLine(points[i+1].x, points[i+1].y, points[i+2].x, points[i+2].y, buffer, BLACK);
		}
		
//		delay_ms(2);
//		LCD_SetTextColor(WHITE);
//		for (int columnIndex = 0; columnIndex < projectedMatrix->cols; columnIndex++) {
//			struct ScreenCoord coord = getCoordFromMatrix(columnIndex, LCD_Width, LCD_Height, projectedMatrix);
//			points[columnIndex] = coord;
//			
//			if (coord.x < 0 || coord.x > LCD_Width) continue;
//			if (coord.y < 0 || coord.y > LCD_Height) continue;
//			Buffer_FillCircle(coord.x, coord.y, 5, buffer);
//		}
//		
//		for (int i = 1; i < 8; i++) {
//			for (int j = 0; j < i; j++) {
//				Buffer_DrawLine(points[i].x, points[i].y, points[j].x, points[j].y, buffer);
//			}
//		}
		
		drawBuffer(buffer);
		free_matrix(rotatedMatrixAxisZ);
		free_matrix(translatedMatrix);
		free_matrix(projectedMatrix);
		delay_ms(10);
	}
	
	free_matrix(rotatedMatrixAxisX);
	free_matrix(rotatedMatrixAxisY);
	free_matrix(scaledMatrix);
	free_matrix(cube);
	
	#define JPG_OK 0
	if (res != JPG_OK)
	{
		LCD_SetTextColor(YELLOW);
		LCD_SaveFont();
		LCD_SetFont(&Font16);
		LCD_DisplayStringAt(5, 100, get_JPG_error_code(),  LEFT_MODE);
		LCD_RestoreFont();
		while(1);
	}
		
		// LCD_DrawJPG(0, 0, (uint8_t *) JPG_fileAddress, 100, 100);
	while(1);
	
//>>>>===== Scinario 1 =======================		
	LCD_Clear(LCD_COLOR_WHITE);
	Touch_sample_FreeDraw();
	
//>>>>===== Scinario 2 =======================		
  Touch_sample_Sine();
	
	//>>>>===== Scinario 3 =======================		
	LCD_Clear(GREEN);
	Touch_sample_Hit();

	//>>>>===== Scinario 3 =======================
	LCD_Clear(GREEN);
	Sample_alarmA();
	
	LCD_Clear(LCD_COLOR_GREEN);
	goto startup;
}


//===============================
void Wait_PressPA0(uint16_t Cnum)
{
	uint16_t count = Cnum;
	while(1)
	{	
    if(GPIOA->IDR & Bit(0))						// normally low
    {
			if (--(count)==0){
				return;
			}
		} else count = Cnum;

    delay_ms(10);
	}
}





