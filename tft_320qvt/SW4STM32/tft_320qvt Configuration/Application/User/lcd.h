/*
 * lcd.h
 */

#ifndef APPLICATION_USER_LCD_H_
#define APPLICATION_USER_LCD_H_

#include "stm32f1xx_hal.h"

// FSMC
extern SRAM_HandleTypeDef hsram1;

#define FSMC_LCD_INSTANCE &hsram1
#define FSMC_LCD_DATA (uint32_t*)0x60020000
#define FSMC_LCD_COMMAND (uint32_t*)0x60000000

// reset PE1
#define LCD_RESET_GPIO_PORT GPIOE
#define LCD_RESET_GPIO_PIN GPIO_PIN_1

// back light switch PB10
#define LCD_BACKLIGHT_PORT GPIOB
#define LCD_BACKLIGHT_PIN GPIO_PIN_10

// SSD1289 Register R01h
#define LCD_R01_RL 0
#define LCD_R01_REV 1
#define LCD_R01_CAD 0
#define LCD_R01_BGR 1
#define LCD_R01_SM 0
#define LCD_R01_TB 1
#define LCD_R01_MUX 319

// orientation
#define LCD_ORIENTATION_PORTRAIT 0
#define LCD_ORIENTATION_LANDSCAPE 1

// lcd size definition
#define LCD_WIDTH 240
#define LCD_HEIGHT 320

// font config
struct _current_font
{
  uint8_t* font;
  uint8_t x_size;
  uint8_t y_size;
  uint8_t offset;
  uint8_t numchars;
};

// interface
void LCD_Init(uint8_t orientation);
void LCD_setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_clrXY();
void LCD_drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_fillRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_drawCircle(uint16_t x, uint16_t y, uint16_t radius);
void LCD_fillCircle(uint16_t x, uint16_t y, uint16_t radius);
void LCD_clrScr();
void LCD_fillScr(uint16_t color);
void LCD_setColor(uint16_t color);
uint16_t LCD_getColor();
void LCD_setBackColor(uint32_t color);
uint16_t LCD_getBackColor();
void LCD_setPixel(uint16_t color);
void LCD_drawPixel(uint16_t x, uint16_t y);
void LCD_drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_drawHLine(uint16_t x, uint16_t y, uint16_t l);
void LCD_drawVLine(uint16_t x, uint16_t y, uint16_t l);
void LCD_printChar(uint16_t c, uint16_t x, uint16_t y);
void LCD_rotateChar(uint8_t c, uint16_t x, uint16_t y, uint16_t pos, uint16_t deg);
void LCD_print(char* st, uint16_t x, uint16_t y, uint16_t deg);
void LCD_printNumI(int32_t num, uint16_t x, uint16_t y, uint16_t length, uint8_t filler);
void LCD_printNumF(double num, uint8_t dec, uint16_t x, uint16_t y, uint8_t divider, uint16_t length, uint8_t filler);
void LCD_setFont(uint8_t* font);
uint8_t* LCD_getFont();
uint8_t LCD_getFontXsize();
uint8_t LCD_getFontYsize();
void LCD_drawBitmap(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint8_t* data, uint16_t deg, uint16_t rox, uint16_t roy);
uint16_t LCD_getDisplayXSize();
uint16_t LCD_getDisplayYSize();


#define LCD_TEXT_ALING_LEFT 0
#define LCD_TEXT_ALING_RIGHT 9999
#define LCD_TEXT_ALING_CENTER 9998

// VGA color code
#define LCD_RGB_TO_COLOR(r,g,b) (uint16_t)((r&0b11111) << 11) | ((g&0b111111) << 5) | ((b&0b11111))
#define VGA_BLACK   LCD_RGB_TO_COLOR(0,0,0)
#define VGA_WHITE   LCD_RGB_TO_COLOR(0xff,0xff,0xff)
#define VGA_RED     LCD_RGB_TO_COLOR(0xff,0,0)
#define VGA_GREEN   LCD_RGB_TO_COLOR(0,0xff,0)
#define VGA_BLUE    LCD_RGB_TO_COLOR(0,0,0xff)
/*#define VGA_SILVER    0xC618
#define VGA_GRAY    0x8410
#define VGA_MAROON    0x8000
#define VGA_YELLOW    0xFFE0
#define VGA_OLIVE   0x8400
#define VGA_LIME    0x07E0
#define VGA_AQUA    0x07FF
#define VGA_TEAL    0x0410
#define VGA_NAVY    0x0010
#define VGA_FUCHSIA   0xF81F
#define VGA_PURPLE    0x8010*/
#define VGA_TRANSPARENT 0xFFFFFFFF


#endif /* APPLICATION_USER_LCD_H_ */
