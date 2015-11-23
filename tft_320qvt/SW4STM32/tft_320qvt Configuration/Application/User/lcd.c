/*
 * lcd.c
 */

// LCD Graphic Interface

// BASE: FSMC

#include <string.h>
#include <math.h>
#include "lcd.h"

void FSMC_LCD_Write_Command(uint16_t command) {
  HAL_SRAM_Write_16b(FSMC_LCD_INSTANCE, FSMC_LCD_COMMAND, &command, 1);
}

void FSMC_LCD_Write_Data(uint16_t data) {
  HAL_SRAM_Write_16b(FSMC_LCD_INSTANCE, FSMC_LCD_DATA, &data, 1);
}

#define FSMC_LCD_Write_Register(reg, val)\
{\
  FSMC_LCD_Write_Command(reg);\
  FSMC_LCD_Write_Data(val);\
}

// MAIN
// Aliases
#define LCD_Write_Command FSMC_LCD_Write_Command
#define LCD_Write_Data FSMC_LCD_Write_Data
#define LCD_Write_Register FSMC_LCD_Write_Register

// GPIO macro
#define LCD_RESET() {\
  LCD_RESET_GPIO_PORT->BRR = LCD_RESET_GPIO_PIN;\
  HAL_Delay(100);\
  LCD_RESET_GPIO_PORT->BSRR = LCD_RESET_GPIO_PIN;\
  HAL_Delay(100);\
}
#define LCD_BACKLIGHT_ON() { LCD_BACKLIGHT_PORT->BSRR = LCD_BACKLIGHT_PIN; }
#define LCD_BACKLIGHT_OFF() { LCD_BACKLIGHT_PORT->BRR = LCD_BACKLIGHT_PIN; }

// initialize SSD1289
volatile uint8_t LCD_Orient = LCD_ORIENTATION_PORTRAIT;

void LCD_Init(uint8_t orientation)
{
  LCD_Orient = orientation;

  LCD_BACKLIGHT_ON();
  LCD_RESET();

  // Display ON Sequence
  // Power supply setting

  // Set R07h at 0021h
  LCD_Write_Register(0x07, 0x0021);
  HAL_Delay(1);  // s
  LCD_Write_Register(0x00, 0x0001);
  HAL_Delay(1);  // s

  // Set R07h at 0023h
  LCD_Write_Register(0x07, 0x0023);
  HAL_Delay(1);  // s

  // Set R10h at 0000h
  LCD_Write_Register(0x10, 0x0000);
  HAL_Delay(1);  // s

  // Wait 30ms
  HAL_Delay(30);  // s

  // Set R07h at 0033h
  LCD_Write_Register(0x07, 0x0033);
  HAL_Delay(1);  // s

  // Entry Mode setting (R11h)
  LCD_Write_Register(0x0011, 0x6070);
  HAL_Delay(1);  // s

  // LCD driver AC setting (R02h)
  LCD_Write_Register(0x0002, 0x0600);
  HAL_Delay(1);  // s

  // Driver Output Control (R01h)
  LCD_Write_Register(0x0001,
      (LCD_R01_RL << 14) | (LCD_R01_REV << 13) | (LCD_R01_CAD << 12) | (LCD_R01_BGR << 11) | (LCD_R01_SM << 10) | (LCD_R01_TB << 9) | LCD_R01_MUX);
  HAL_Delay(1);

  // RAM data write (R22h)
  LCD_Write_Command(0x0022);  // s

  // Display ON and start to write RAM
  HAL_Delay(100);
}

// graphics
// import from UTFT

// swap macro
#define swap(type,a,b) { type temp = a; a = b; b = temp; }

volatile uint16_t disp_x_size = LCD_WIDTH - 1;
volatile uint16_t disp_y_size = LCD_HEIGHT - 1;
volatile uint8_t _transparent = 0;
volatile uint16_t fColor = 0;
volatile uint16_t bColor = 0;
volatile struct _current_font cfont;

void LCD_setXY(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (LCD_Orient == LCD_ORIENTATION_LANDSCAPE)
  {
    swap(uint16_t, x1, y1);
    swap(uint16_t, x2, y2);
    y1 = disp_y_size - y1;
    y2 = disp_y_size - y2;
    swap(uint16_t, y1, y2);
  }

  LCD_Write_Register(0x44, (x2<<8)+x1);
  LCD_Write_Register(0x45, y1);
  LCD_Write_Register(0x46, y2);
  LCD_Write_Register(0x4e, x1);
  LCD_Write_Register(0x4f, y1);
  LCD_Write_Command(0x22);
}

void LCD_clrXY()
{
  if (LCD_Orient == LCD_ORIENTATION_PORTRAIT)
  {
    LCD_setXY(0, 0, disp_x_size, disp_y_size);
  }
  else
  {
    LCD_setXY(0, 0, disp_y_size, disp_x_size);
  }
}

void LCD_drawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (x1 > x2)
  {
    swap(uint16_t, x1, x2);
  }
  if (y1 > y2)
  {
    swap(uint16_t, y1, y2);
  }

  LCD_drawHLine(x1, y1, x2-x1);
  LCD_drawHLine(x1, y2, x2-x1);
  LCD_drawVLine(x1, y1, y2-y1);
  LCD_drawVLine(x2, y1, y2-y1);
}

void LCD_drawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (x1>x2)
  {
    swap(uint16_t, x1, x2);
  }
  if (y1>y2)
  {
    swap(uint16_t, y1, y2);
  }
  if ((x2-x1)>4 && (y2-y1)>4)
  {
    LCD_drawPixel(x1+1,y1+1);
    LCD_drawPixel(x2-1,y1+1);
    LCD_drawPixel(x1+1,y2-1);
    LCD_drawPixel(x2-1,y2-1);
    LCD_drawHLine(x1+2, y1, x2-x1-4);
    LCD_drawHLine(x1+2, y2, x2-x1-4);
    LCD_drawVLine(x1, y1+2, y2-y1-4);
    LCD_drawVLine(x2, y1+2, y2-y1-4);
  }
}

void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (x1>x2)
  {
    swap(uint16_t, x1, x2);
  }
  if (y1>y2)
  {
    swap(uint16_t, y1, y2);
  }

  int i;
  if (LCD_Orient == LCD_ORIENTATION_PORTRAIT)
  {
    for (i = 0; i < ((y2-y1)/2)+1; i++)
    {
      LCD_drawHLine(x1, y1+i, x2-x1);
      LCD_drawHLine(x1, y2-i, x2-x1);
    }
  }
  else
  {
    for (i = 0; i < ((x2-x1)/2)+1; i++)
    {
      LCD_drawVLine(x1+i, y1, y2-y1);
      LCD_drawVLine(x2-i, y1, y2-y1);
    }
  }
}

void LCD_fillRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (x1>x2)
  {
    swap(uint16_t, x1, x2);
  }
  if (y1>y2)
  {
    swap(uint16_t, y1, y2);
  }

  if ((x2-x1)>4 && (y2-y1)>4)
  {
    int i;
    for (i = 0; i < ((y2-y1)/2)+1; i++)
    {
      switch(i)
      {
      case 0:
        LCD_drawHLine(x1+2, y1+i, x2-x1-4);
        LCD_drawHLine(x1+2, y2-i, x2-x1-4);
        break;
      case 1:
        LCD_drawHLine(x1+1, y1+i, x2-x1-2);
        LCD_drawHLine(x1+1, y2-i, x2-x1-2);
        break;
      default:
        LCD_drawHLine(x1, y1+i, x2-x1);
        LCD_drawHLine(x1, y2-i, x2-x1);
      }
    }
  }
}

void LCD_drawCircle(uint16_t x, uint16_t y, uint16_t radius)
{
  int f = 1 - radius;
  int ddF_x = 1;
  int ddF_y = -2 * radius;
  int x1 = 0;
  int y1 = radius;

  LCD_setXY(x, y + radius, x, y + radius);
  LCD_Write_Data(fColor);
  LCD_setXY(x, y - radius, x, y - radius);
  LCD_Write_Data(fColor);
  LCD_setXY(x + radius, y, x + radius, y);
  LCD_Write_Data(fColor);
  LCD_setXY(x - radius, y, x - radius, y);
  LCD_Write_Data(fColor);

  while(x1 < y1)
  {
    if(f >= 0)
    {
      y1--;
      ddF_y += 2;
      f += ddF_y;
    }
    x1++;
    ddF_x += 2;
    f += ddF_x;
    LCD_setXY(x + x1, y + y1, x + x1, y + y1);
    LCD_Write_Data(fColor);
    LCD_setXY(x - x1, y + y1, x - x1, y + y1);
    LCD_Write_Data(fColor);
    LCD_setXY(x + x1, y - y1, x + x1, y - y1);
    LCD_Write_Data(fColor);
    LCD_setXY(x - x1, y - y1, x - x1, y - y1);
    LCD_Write_Data(fColor);
    LCD_setXY(x + y1, y + x1, x + y1, y + x1);
    LCD_Write_Data(fColor);
    LCD_setXY(x - y1, y + x1, x - y1, y + x1);
    LCD_Write_Data(fColor);
    LCD_setXY(x + y1, y - x1, x + y1, y - x1);
    LCD_Write_Data(fColor);
    LCD_setXY(x - y1, y - x1, x - y1, y - x1);
    LCD_Write_Data(fColor);
  }
  LCD_clrXY();
}

void LCD_fillCircle(uint16_t x, uint16_t y, uint16_t radius)
{
  int y1, x1;
  for(y1=-radius; y1<=0; y1++)
    for(x1=-radius; x1<=0; x1++)
      if(x1*x1+y1*y1 <= radius*radius)
      {
        LCD_drawHLine(x+x1, y+y1, 2*(-x1));
        LCD_drawHLine(x+x1, y-y1, 2*(-x1));
        break;
      }
}

void LCD_clrScr()
{
  LCD_fillScr(0);
}

void LCD_fillScr(uint16_t color)
{
  long i;
  LCD_clrXY();

  for (i=0; i<((disp_x_size+1)*(disp_y_size+1)); i++)
  {
    LCD_Write_Data(color);
  }
}

void LCD_setColor(uint16_t color)
{
  fColor = color;
}

uint16_t LCD_getColor()
{
  return fColor;
}

void LCD_setBackColor(uint32_t color)
{
  if (color == VGA_TRANSPARENT)
    _transparent=1;
  else
  {
    bColor = color;
    _transparent=0;
  }
}

uint16_t LCD_getBackColor()
{
  return bColor;
}

void LCD_setPixel(uint16_t color)
{
  LCD_Write_Data(color);
}

void LCD_drawPixel(uint16_t x, uint16_t y)
{
  LCD_setXY(x, y, x, y);
  LCD_setPixel(fColor);
  LCD_clrXY();
}

void LCD_drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if (y1==y2)
    LCD_drawHLine(x1, y1, x2-x1);
  else if (x1==x2)
    LCD_drawVLine(x1, y1, y2-y1);
  else
  {
    unsigned int  dx = (x2 > x1 ? x2 - x1 : x1 - x2);
    short     xstep =  x2 > x1 ? 1 : -1;
    unsigned int  dy = (y2 > y1 ? y2 - y1 : y1 - y2);
    short     ystep =  y2 > y1 ? 1 : -1;
    int       col = x1, row = y1;

    if (dx < dy)
    {
      int t = - (dy >> 1);
      while (1)
      {
        LCD_setXY (col, row, col, row);
        LCD_Write_Data(fColor);
        if (row == y2)
          return;
        row += ystep;
        t += dx;
        if (t >= 0)
        {
          col += xstep;
          t   -= dy;
        }
      }
    }
    else
    {
      int t = - (dx >> 1);
      while (1)
      {
        LCD_setXY (col, row, col, row);
        LCD_Write_Data(fColor);
        if (col == x2)
          return;
        col += xstep;
        t += dy;
        if (t >= 0)
        {
          row += ystep;
          t   -= dx;
        }
      }
    }
  }
  LCD_clrXY();
}

void LCD_drawHLine(uint16_t x, uint16_t y, uint16_t l)
{
  if (l<0)
  {
    l = -l;
    x -= l;
  }
  LCD_setXY(x, y, x+l, y);
  uint32_t i;
  for (i=0; i<l+1; i++)
  {
    LCD_Write_Data(fColor);
  }
  LCD_clrXY();
}

void LCD_drawVLine(uint16_t x, uint16_t y, uint16_t l)
{
  if (l<0)
  {
    l = -l;
    y -= l;
  }
  LCD_setXY(x, y, x, y+l);
  uint32_t i;
  for (i=0; i<l+1; i++)
  {
    LCD_Write_Data(fColor);
  }
  LCD_clrXY();
}

void LCD_printChar(uint16_t c, uint16_t x, uint16_t y)
{
  uint8_t i,ch;
  uint16_t j;
  uint16_t temp;
  uint32_t zz;

  if (!_transparent)
  {
    if (LCD_Orient == LCD_ORIENTATION_PORTRAIT)
    {
      LCD_setXY(x,y,x+cfont.x_size-1,y+cfont.y_size-1);

      temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
      for(j=0;j<((cfont.x_size/8)*cfont.y_size);j++)
      {
        ch=cfont.font[temp];
        for(i=0;i<8;i++)
        {
          if((ch&(1<<(7-i)))!=0)
          {
            LCD_setPixel(fColor);
          }
          else
          {
            LCD_setPixel(bColor);
          }
        }
        temp++;
      }
    }
    else
    {
      temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;

      for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
      {
        LCD_setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
        for (zz=(cfont.x_size/8)-1; zz>=0; zz--)
        {
          ch=cfont.font[temp+zz];
          for(i=0;i<8;i++)
          {
            if((ch&(1<<i))!=0)
            {
              LCD_setPixel(fColor);
            }
            else
            {
              LCD_setPixel(bColor);
            }
          }
        }
        temp+=(cfont.x_size/8);
      }
    }
  }
  else
  {
    temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
    for(j=0;j<cfont.y_size;j++)
    {
      for (zz=0; zz<(cfont.x_size/8); zz++)
      {
        ch=cfont.font[temp+zz];
        for(i=0;i<8;i++)
        {

          if((ch&(1<<(7-i)))!=0)
          {
            LCD_setXY(x+i+(zz*8),y+j,x+i+(zz*8)+1,y+j+1);
            LCD_setPixel(fColor);
          }
        }
      }
      temp+=(cfont.x_size/8);
    }
  }
  LCD_clrXY();
}

void LCD_rotateChar(uint8_t c, uint16_t x, uint16_t y, uint16_t pos, uint16_t deg)
{
  uint8_t i,j,ch;
  uint16_t temp;
  int newx,newy;
  double radian;
  radian=deg*0.0175;
  uint32_t zz;

  temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
  for(j=0;j<cfont.y_size;j++)
  {
    for (zz=0; zz<(cfont.x_size/8); zz++)
    {
      ch=cfont.font[temp+zz];
      for(i=0;i<8;i++)
      {
        newx=x+(((i+(zz*8)+(pos*cfont.x_size))*cos(radian))-((j)*sin(radian)));
        newy=y+(((j)*cos(radian))+((i+(zz*8)+(pos*cfont.x_size))*sin(radian)));

        LCD_setXY(newx,newy,newx+1,newy+1);

        if((ch&(1<<(7-i)))!=0)
        {
          LCD_setPixel(fColor);
        }
        else
        {
          if (!_transparent)
            LCD_setPixel(bColor);
        }
      }
    }
    temp+=(cfont.x_size/8);
  }
  LCD_clrXY();
}

void LCD_print(char *st, uint16_t x, uint16_t y, uint16_t deg)
{
  int stl, i;

  stl = strlen(st);

  if (LCD_Orient == LCD_ORIENTATION_PORTRAIT)
  {
  if (x == LCD_TEXT_ALING_RIGHT)
    x=(disp_x_size+1)-(stl*cfont.x_size);
  if (x == LCD_TEXT_ALING_CENTER)
    x=((disp_x_size+1)-(stl*cfont.x_size))/2;
  }
  else
  {
  if (x == LCD_TEXT_ALING_RIGHT)
    x=(disp_y_size+1)-(stl*cfont.x_size);
  if (x == LCD_TEXT_ALING_CENTER)
    x=((disp_y_size+1)-(stl*cfont.x_size))/2;
  }

  for (i=0; i<stl; i++)
    if (deg==0)
      LCD_printChar(*st++, x + (i*(cfont.x_size)), y);
    else
      LCD_rotateChar(*st++, x, y, i, deg);
}

void LCD_printNumI(int32_t num, uint16_t x, uint16_t y, uint16_t length, uint8_t filler)
{
  char buf[25];
  char st[27];
  uint8_t neg = 0;
  int c=0, f=0;
  uint32_t i;

  if (num==0)
  {
    if (length!=0)
    {
      for (c=0; c<(length-1); c++)
        st[c]=filler;
      st[c]=48;
      st[c+1]=0;
    }
    else
    {
      st[0]=48;
      st[1]=0;
    }
  }
  else
  {
    if (num<0)
    {
      neg=1;
      num=-num;
    }

    while (num>0)
    {
      buf[c]=48+(num % 10);
      c++;
      num=(num-(num % 10))/10;
    }
    buf[c]=0;

    if (neg)
    {
      st[0]=45;
    }

    if (length>(c+neg))
    {
      for (i=0; i<(length-c-neg); i++)
      {
        st[i+neg]=filler;
        f++;
      }
    }

    for (i=0; i<c; i++)
    {
      st[i+neg+f]=buf[c-i-1];
    }
    st[c+neg+f]=0;

  }

  LCD_print(st,x,y,0);
}

void LCD_printNumF(
    double num, uint8_t dec,
    uint16_t x, uint16_t y,
    uint8_t divider,
    uint16_t length,
    uint8_t filler)
{
  char st[27];
  uint8_t neg=0;
  uint32_t i;

  if (dec<1)
    dec=1;
  else if (dec>5)
    dec=5;

  if (num<0)
    neg = 1;

  char format[10];
  sprintf(format, "%%%i.%if", length, dec);
  sprintf(st, format, num);

  /*if (divider != '.')
  {
    for (i=0; i<sizeof(st); i++)
      if (st[i]=='.')
        st[i]=divider;
  }

  if (filler != ' ')
  {
    if (neg)
    {
      st[0]='-';
      for (i=1; i<sizeof(st); i++)
        if ((st[i]==' ') || (st[i]=='-'))
          st[i]=filler;
    }
    else
    {
      for (i=0; i<sizeof(st); i++)
        if (st[i]==' ')
          st[i]=filler;
    }
  }*/

  LCD_print(st,x,y,0);
}

void LCD_setFont(uint8_t* font)
{
  cfont.font=font;
  cfont.x_size=font[0];
  cfont.y_size=font[1];
  cfont.offset=font[2];
  cfont.numchars=font[3];
}

uint8_t* LCD_getFont()
{
  return cfont.font;
}

uint8_t LCD_getFontXsize()
{
  return cfont.x_size;
}

uint8_t LCD_getFontYsize()
{
  return cfont.y_size;
}

void LCD_drawBitmap(
    uint16_t x, uint16_t y,
    uint16_t sx, uint16_t sy,
    uint8_t* data,
    uint16_t deg, uint16_t rox, uint16_t roy)
{
  unsigned int col;
  int tx, ty, newx, newy;
  double radian;
  radian=deg*0.0175;
  for (ty=0; ty<sy; ty++)
    for (tx=0; tx<sx; tx++)
    {
      col = data[(ty*sx)+tx];

      newx=x+rox+(((tx-rox)*cos(radian))-((ty-roy)*sin(radian)));
      newy=y+roy+(((ty-roy)*cos(radian))+((tx-rox)*sin(radian)));

      LCD_setXY(newx, newy, newx, newy);
      LCD_Write_Data(col);
    }
  LCD_clrXY();
}

uint16_t LCD_getDisplayXSize()
{
  return (LCD_Orient == LCD_ORIENTATION_PORTRAIT ? disp_x_size : disp_y_size) + 1;
}

uint16_t LCD_getDisplayYSize()
{
  return (LCD_Orient == LCD_ORIENTATION_PORTRAIT ? disp_y_size : disp_x_size) + 1;
}

