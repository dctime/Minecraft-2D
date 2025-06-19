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
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;  // ? PA0
	
	EXTI->IMR  |= EXTI_IMR_IM0;     // ?? EXTI0 ?????
	EXTI->FTSR |= EXTI_FTSR_TR0;    // ?????????(? 1 ? 0,??????)
	
	NVIC_EnableIRQ(EXTI0_IRQn);  // ?? EXTI0 ????
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
		default:
			return generateLevel1;
	}
}

void youWinScreen();

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
		showMainMenu();
		play(levelNumToFunc(1));
		play(levelNumToFunc(2));
		youWinScreen();
	}
}

extern volatile bool key1Triggered;

void youWinScreen() {
	LCD_Clear(BLACK);
	char c[10] = "YOU WIN!";
	char c2[20] = "Press KEY1 to reset";
	LCD_SaveFont();
	LCD_SaveColors();
	LCD_SetFont(&Font16);	
	LCD_SetColors(WHITE, BLACK); // Text = red; back = white
	LCD_DisplayStringLineCol(6, 10, c);
	LCD_DisplayStringLineCol(10, 5, c2);
	LCD_RestoreColors();
	LCD_RestoreFont();
	
	while(!key1Triggered);
	key1Triggered = 0;
}







