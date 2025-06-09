#ifndef DCTIME_LCD
#define DCTIME_LCD

#include <stdint.h>
#include <stdlib.h>

#define LCD_Width 320
#define LCD_Height 240

#define  RS_bit							 10
#define	 LCD_BASE_sel        0x6C000000
#define	 LCD_BASE        (LCD_BASE_sel | (0x01UL<<(RS_bit+1))-2)

#define  LCD             ((LCD_TypeDef *) LCD_BASE)
#define LCD_WriteData(x)	(LCD->LCD_RAM = (uint16_t) x)	//lcd write data

typedef struct
{
  volatile uint16_t LCD_REG;			// RS = 0
  volatile uint16_t LCD_RAM;      // RS = 1
} LCD_TypeDef;

typedef struct Buffer {
	uint8_t data[LCD_Height][LCD_Width];
} Buffer;

Buffer* createBuffer();
void clearBuffer(uint16_t n, Buffer* buffer);


uint16_t bufferGetColor(int x, int y, struct Buffer* buffer);
void writeBuffer(int x, int y, struct Buffer* buffer, uint16_t color);

void drawBuffer(struct Buffer* buffer);
void Buffer_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, struct Buffer* buffer, uint16_t color);
void Buffer_FillCircle(uint16_t Xcen, uint16_t Ycen, uint16_t Radius, struct Buffer* buffer, uint16_t color);
void Buffer_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, struct Buffer* buffer, uint16_t color);
void Buffer_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, struct Buffer* buffer, uint16_t color);

uint8_t RGB565ToRGB332(uint16_t n);
uint16_t RGB332ToRGB565(uint8_t n);
uint8_t RGB332GrayScale(double z);

#endif