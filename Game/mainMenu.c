#include "stm324xg_lcd_sklin.h"
#include "mainMenu.h"
#include "game.h"
#include "user_defined.h"	// for delay_ms(wait_ms)
#include <limits.h>
#include <stdio.h>
#include "projectToBuffer.h"

void renderingSpinningPlayer(RectPlayer* player, Buffer* buffer, double scale, double rotX, double rotZ, double tZ, double fov, double near, double far);

static uint16_t width = 200;
static uint16_t height = 38;
static uint16_t tempX = 60;
static uint16_t tempY = 101;

extern volatile bool key1Triggered;

void showMainMenu() {
	delay_ms(1000);
	
	uint8_t res;
	
	res = LCD_DrawJPG(tempX, tempY, (uint8_t *) JPG_fileAddress, width, height);
	
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
	
	uint8_t* mainTitleImage = malloc(2 * width * height);
	
	LCD_ReadRGBImage(tempX, tempY, width, height, mainTitleImage);	
	
	RectPlayer player;
	initPlayer(&player, 0, 1);
	Buffer* buffer = createBuffer();

	int xRot = 0;
	int zRot = 0;

	key1Triggered = 0;
	while(!key1Triggered) {
		clearBuffer(BLACK, buffer, NULL);
		xRot += 5;
		zRot += 1;
		projectPlayerRectToBuffer(&player, buffer, 0.5, xRot, zRot, 4, 50, 0.1, 100.0);
		Buffer_DrawRGBImage(tempX, tempY, width, height, mainTitleImage, buffer);
		Buffer_DisplayStringAt(0, 180, (char*)" Press KEY1 To Continue", CENTER_MODE, buffer, &Font12, WHITE, BLACK);
		
		if (getLeastTotalUsedStep() != INT_MAX) {
			char c[24];
			sprintf(c, "Least Used Steps: %d", getLeastTotalUsedStep());
			Buffer_DisplayStringAt(0, 200, c, CENTER_MODE, buffer, &Font12, WHITE, BLACK);
		}
		
		drawBuffer(buffer);
	}
	key1Triggered = 0;
	freePlayerModel(&player);
	freeBuffer(buffer);
	free(mainTitleImage);
}



