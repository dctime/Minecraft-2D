#include "stm324xg_lcd_sklin.h"
#include "dctime_lcd.h"
#include "dctimegl.h"
#include <math.h>

void LCD_OpenWin(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

void freeBuffer(Buffer* buffer) {
	free(buffer);
}

void drawBuffer(struct Buffer* buffer) {
	LCD_OpenWin(0, 0, LCD_Width-1, LCD_Height-1);
	for (int y = 0; y < LCD_Height; y++) {
		for (int x = 0; x < LCD_Width; x++) {
			uint16_t color = bufferGetColor(x, y, buffer);
			LCD_WriteData(color);
		}
	}
}

Buffer* createBuffer() {
	Buffer* buffer = (Buffer*) malloc(sizeof(Buffer));
	for (int i = 0; i < LCD_Width; i++) {
		for (int j = 0; j < LCD_Height; j++) {
			buffer->data[j][i] = 0xFF;
		}
	}
	return buffer;
}

void clearBuffer(uint16_t n, Buffer* buffer) {
		for (int i = 0; i < LCD_Width; i++) {
			for (int j = 0; j < LCD_Height; j++) {
				buffer->data[j][i] = RGB565ToRGB332(n);
			}
		}
}

uint16_t bufferGetColor(int x, int y, struct Buffer* buffer) {
	return RGB332ToRGB565(buffer->data[y][x]);
}

void writeBuffer(int x, int y, struct Buffer* buffer, uint16_t color) {
	buffer->data[y][x] = RGB565ToRGB332(color);
}

//=================================================
#define Default_TextColor		0xFFFF
#define Default_BackColor		0x0000
#define LCD_DEFAULT_FONT    Font20
#define  LCD_PIXEL_WIDTH   (LCD_COL_NUM-1)
#define  LCD_PIXEL_HEIGHT  (LCD_ROW_NUM-1)
#define  LCD_COL_NUM    320                //columns in ILI9341
#define  LCD_ROW_NUM    240                //pages in ILI9341
#define ABS(X)  ((X) > 0 ? (X) : -(X))
//=============================================

/* Global variables to set the written text color */
typedef struct 
{ 
  uint16_t TextColor;
  uint16_t BackColor;
  sFONT    *pFont; 
}LCD_DrawPropTypeDef;

static LCD_DrawPropTypeDef DrawProp={ \
			Default_TextColor, Default_BackColor, &LCD_DEFAULT_FONT};

void Buffer_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, struct Buffer* buffer, uint16_t color)
{
	uint16_t j;

	if (Xpos > LCD_PIXEL_WIDTH) return;
		for(j = 0; j < Length; j++)
				writeBuffer(Xpos, Ypos+j, buffer, color);
}

void Buffer_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, struct Buffer* buffer, uint16_t color)
{
	uint16_t i;

	if (Ypos > LCD_PIXEL_HEIGHT) return;
		for(i = 0; i < Length; i++)
			writeBuffer(Xpos+i, Ypos, buffer, color);
}

void Buffer_FillCircle(uint16_t Xcen, uint16_t Ycen, uint16_t Radius, struct Buffer* buffer, uint16_t color)
{
//	uint16_t color=DrawProp.TextColor;
  int32_t  next;/* Decision Variable */
  uint16_t  Ri;/* increasing from 0 to Radius */
  uint16_t  Rd;/* decreasing from Radius to 0 */

  next = 3 - (Radius << 1);
  Ri = 0;
  Rd = Radius;

  while (Ri <= Rd)
  {
		int32_t X0, Y0;

		X0 = Xcen - Ri;
		Y0 = Ycen -	Rd;
		if(X0<0) X0=0;
		if(Y0<0) Y0=0;
		Buffer_DrawVLine((uint16_t)X0, (uint16_t)Y0, (uint16_t)(Ycen + Rd - Y0), buffer, color);				// vertical line in landscape  view
		X0 = Xcen + Ri;
		Buffer_DrawVLine((uint16_t)X0, (uint16_t)Y0, (uint16_t)(Ycen + Rd - Y0), buffer, color);				// vertical line in landscape  view
		X0 = Xcen - Rd;
		Y0 = Ycen -	Ri;
		if(X0<0) X0=0;
		if(Y0<0) Y0=0;
		Buffer_DrawHLine((uint16_t)X0, (uint16_t)Y0, (uint16_t)(Xcen + Rd - X0), buffer, color);				// horizontal line in landscape  view
		Y0 = Ycen +	Ri;
		Buffer_DrawHLine((uint16_t)X0, (uint16_t)Y0, (uint16_t)(Xcen + Rd - X0), buffer, color);				// horizontal line in landscape  view
    if (next < 0)
    {
      next += (Ri << 2) + 6;
    }
    else
    {
      next += ((Ri - Rd) << 2) + 10;
      Rd--;
    }
    Ri++;
  }
}


void Buffer_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, struct Buffer* buffer, uint16_t color)
{
  int16_t deltax, deltay, x, y, xinc1, xinc2,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = (int16_t)ABS(x2 - x1);        /* The difference between the x's */
  deltay = (int16_t)ABS(y2 - y1);        /* The difference between the y's */
  x = (int16_t)x1;                       /* Start x off at the first pixel */
  y = (int16_t)y1;                       /* Start y off at the first pixel */
	
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
		if(x < 0 || x > LCD_PIXEL_WIDTH || y < 0 || y > LCD_PIXEL_HEIGHT)
		{
			goto escape;
		}

			writeBuffer((uint16_t)x, (uint16_t)y, buffer, color);

escape:
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

uint8_t RGB565ToRGB332(uint16_t n) {
	uint8_t r5 = (n >> 11) & 0x1F;
	uint8_t g6 = (n >> 5) & 0x3F;
	uint8_t b5 = n & 0x1F;
	
	uint8_t r3 = r5 >> 2;
	uint8_t g3 = g6 >> 3;
	uint8_t b2 = b5 >> 3;
	
	uint8_t ans = r3 << 5 | g3 << 2 | b2;
	return ans;
}

uint16_t RGB332ToRGB565(uint8_t n) {
	uint16_t r3 = n >> 5 & 0x0007;
	uint16_t g3 = n >> 2 & 0x0007;
	uint16_t b2 = n & 0x0003;
	
	uint16_t r5 = r3 << 2;
	uint16_t g6 = g3 << 3;
	uint16_t b5 = b2 << 3;
	
	uint16_t ans = r5 << 11 | g6 << 5 | b5;
	return ans;
}

// 0 - 1 to 0 - 8
uint8_t RGB332GrayScale(double z) {
	uint8_t scale = 8 * z*z*z*z*z;
	uint8_t fullR = scale;
	uint8_t fullG = scale;
	uint8_t fullB = scale/2;
	return fullR << 5 | fullG << 2 | fullB;
}