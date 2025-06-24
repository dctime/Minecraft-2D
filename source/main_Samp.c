/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm324xg_lcd_sklin.h"
#include "user_defined.h"
#include <stdio.h>	// for sprintf
#include "touch_module.h"
#include "dctime_lcd.h"
#include "matrix.h"
#include "dctimegl.h"
#include "game.h"
#include "mainMenu.h"
#include "level.h"
#include "ai_func.h"
#include "uiButton.h"

#define LCD_Width 320
#define LCD_Height 240

void stm32f4_Hardware_Init (void);
void Wait_PressPA0(uint16_t Cnum);

void Driver_GPIO(void);
void Driver_SPIpin_GPIO(void);

void Default_Calibration(void);

void Touchscreen_Calibration (void);


void resetButton1Setup() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // SYSCFG
	
	GPIOA->MODER &= ~(3 << (0 * 2));  // MODER0[1:0] = 00 = input mode
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;  // 
	
	EXTI->IMR  |= EXTI_IMR_IM0;     // 
	EXTI->FTSR |= EXTI_FTSR_TR0;    // 
	
	NVIC_EnableIRQ(EXTI0_IRQn);  //
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void generateLevel1(Level* level);
void generateLevel2(Level* level);

typedef void(*genLevelFunc)(Level*);	

genLevelFunc levelNumToFunc(int n) {
	switch(n) {
		case 1:
			return generateLevel1;
		case 2:
			return generateLevel2;
		case 3:
			return generateLevel3;
		default:
			return generateLevel1;
	}
}

void youWinScreen();



extern volatile bool key1Triggered;
float input_image[28][28];
void drawShapeScreen();

int main(void)
{
	stm32f4_Hardware_Init();

	/* Initialize the LCD */
	LCD_Init();
	LCD_Clear(BLACK);
	LCD_DisplayOn();

	/* Initialize the Touch module */
	Default_Calibration();
	Driver_GPIO();
	
	resetButton1Setup();
	
//	Touchscreen_Calibration();
	while(1) {
		resetTotalUsedStep();
		showMainMenu();
		drawShapeScreen();
		while(!play(levelNumToFunc(3)));
		while(!play(levelNumToFunc(1)));
		while(!play(levelNumToFunc(2)));
		youWinScreen();
	}
}

static TS_StateTypeDef tsState;

bool inDrawingPanel(int x, int y, int x0, int y0, int width, int height) {
	if (x < x0 || x > x0 + width) return false;
	if (y < y0 || y > y0 + height) return false;
	return true;
}

void checkShapeButtonTrigger(RectPlayer* player, Level* level) {
	LCD_SaveColors();
	LCD_SetTextColor(BLACK);
	LCD_FillRect(LCD_Width/2-168/2+1, 1, 168, 168);
	LCD_RestoreColors();
}

void aistuff() {
	// player and level is null
		int width = 168;
		int height = 168;
		float inputImage[28][28];
	
		uint8_t* drawImage = malloc(2 * width * height);
		LCD_ReadRGBImage(LCD_Width/2-168/2+1, 1, width, height, drawImage);
		//	LCD_Clear(BLACK);
		//	LCD_DrawRGBImage(0, 0, width, height, drawImage);
	
		for (int smallImageX = 0; smallImageX < 28; smallImageX++) {
			for (int smallImageY = 0; smallImageY < 28; smallImageY++) {
				int sum = 0;
				for (int largePartialImageX = 0; largePartialImageX < 6; largePartialImageX++) {
					for (int largePartialImageY = 0; largePartialImageY < 6; largePartialImageY++) {
						int largeImageX = 6*smallImageX+largePartialImageX;
						int largeImageY = 6*smallImageY+largePartialImageY;
						sum += 1&*((drawImage+(168*largeImageY+largeImageX)*2)); // check if white
		//					LCD_DrawPixel(largeImageX, largeImageY, *((drawImage+(168*largeImageY+largeImageX)*2)) + 0x10* *((drawImage+(168*largeImageY+largeImageX)*2+1)));
						
					}
				}
		//			LCD_DrawRect(LCD_Width/2-168/2+6*smallImageX, 6*smallImageY, 6, 6);
				if (sum > 1) {
					inputImage[smallImageY][smallImageX] = 1;
				} else {
					inputImage[smallImageY][smallImageX] = 0;
				}
					
//				inputImage[smallImageY][smallImageX] = sum/36.0;
			}
		}

		free(drawImage);

		for (int smallImageX = 0; smallImageX < 28; smallImageX++) {
			for (int smallImageY = 0; smallImageY < 28; smallImageY++) {
				LCD_DrawPixel(smallImageX, smallImageY, inputImage[smallImageY][smallImageX] * 255); // blue
			}
		}

		float probCircle, probSquare, probTriangle;


		run_cnn((float*)inputImage, &probCircle, &probSquare, &probTriangle);

		char c[50];
		sprintf(c, "C:%f, R:%f, T:%f", probCircle, probSquare, probTriangle);
		LCD_SetFont(&Font12);
		LCD_DisplayStringAt(0, LCD_Height-60, c, LEFT_MODE);
}

void drawShapeScreen() {
	init_cnn();
	
	Button button = {.x0=LCD_Width/2-BUTTON_WIDTH/2, .y0=LCD_Height-BUTTON_HEIGHT, .wX=BUTTON_WIDTH, .wY=BUTTON_HEIGHT};
	initButton(&button, checkShapeButtonTrigger);
	LCD_Clear(BLACK);
	LCD_SetTextColor(WHITE);
	LCD_DrawRect(LCD_Width/2-168/2, 0, 170, 170);

	TS_StateTypeDef tsState;
	uint16_t lastX = 0xFFFF, lastY = 0xFFFF;
	const uint8_t sampleCount = 5;
	uint16_t xs[sampleCount], ys[sampleCount];
	uint8_t idx = 0;
	uint8_t collected = 0;

	while (1) {
		TS_GetState(&tsState);
		buttonTick(&button, &tsState, NULL, NULL);
		renderButtonImmediatelyLCD(&button);
		
		if (tsState.TouchDetected && inDrawingPanel(tsState.x, tsState.y, LCD_Width/2-168/2+1, 1, 168, 168)) {
			// Collect new sample
			xs[idx] = tsState.x;
			ys[idx] = tsState.y;
			idx = (idx + 1) % sampleCount;
			if (collected < sampleCount) collected++;

			// Compute average only if enough samples
			if (collected >= sampleCount) {
				uint32_t sumX = 0, sumY = 0;
				for (int i = 0; i < sampleCount; i++) {
					sumX += xs[i];
					sumY += ys[i];
				}
				uint16_t avgX = sumX / sampleCount;
				uint16_t avgY = sumY / sampleCount;

				// there is difference (no redraw)
				if (abs(avgX - lastX) > 2 || abs(avgY - lastY) > 2) {
					LCD_FillCircle(avgX, avgY, 2);
					lastX = avgX;
					lastY = avgY;
				}
			}
		} else {
			// Reset state if touch released
			collected = 0;
			idx = 0;
			lastX = 0xFFFF;
			lastY = 0xFFFF;
			aistuff();
		}
		/* ------------------------------------
		// AI stuff
		----------------------------------*/
		

			
			
	}
	
	free_cnn();
}

void youWinScreen() {
	LCD_Clear(BLACK);
	LCD_SaveFont();
	LCD_SaveColors();
	LCD_SetFont(&Font16);
	
	
	if (trySetLeastTotalUsedStep(getTotalUsedStep())) {
		LCD_SetColors(YELLOW, BLACK);
		char c[12] = "NEW RECORD!";
		LCD_DisplayStringAt(0, 100, c, CENTER_MODE);
	} else {
		LCD_SetColors(WHITE, BLACK);
		char c[9] = "YOU WIN!";
		LCD_DisplayStringAt(0, 100, c, CENTER_MODE);
	}
	
	LCD_SetColors(WHITE, BLACK);
	
	char c1[23];
	char c2[20] = "Press KEY1 to reset";
	sprintf(c1, "Total Used Step: %d", getTotalUsedStep());
	
	LCD_DisplayStringAt(0, 150, c1, CENTER_MODE);
	
	delay_ms(1000);
	LCD_DisplayStringAt(0, 200, c2, CENTER_MODE);
	
	LCD_RestoreColors();
	LCD_RestoreFont();
	key1Triggered = 0;
	while(!key1Triggered);
	key1Triggered = 0;
}







