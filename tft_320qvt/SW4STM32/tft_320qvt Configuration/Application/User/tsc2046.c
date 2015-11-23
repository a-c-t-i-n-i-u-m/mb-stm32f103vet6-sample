/*
 * touch.c
 */

#include <math.h>
#include "tsc2046.h"

// TSC2046 driver
// calibration values
volatile int16_t offsetX = 0;
volatile int16_t offsetY = 0;
volatile double scaleX = TSC2046_WIDTH/4096;
volatile double scaleY = TSC2046_HEIGHT/4096;

void TSC2046_Init()
{
  // TSC2046_Restore_Calibration();
}

// read value
uint16_t TSC2046_Read(uint8_t command)
{
  // select chip
  HAL_GPIO_WritePin(TSC2046_CS_PORT, TSC2046_CS_PIN, GPIO_PIN_RESET);

  // send control byte
  HAL_SPI_Transmit(TSC2046_SPI_INSTANCE, &command, 1, 0xff);

  // wait
  HAL_Delay(2);

  // receive
  uint16_t value = 0;
  uint8_t buf[2] = {0, 0};

  // if 8bit mode
  if (command & TSC2046_CB_8BIT)
  {
    HAL_SPI_Receive(TSC2046_SPI_INSTANCE, buf, 1, 0xff);
    value = buf[0];
  }
  else
  {
    HAL_SPI_Receive(TSC2046_SPI_INSTANCE, buf, 2, 0xff);
    // [0] = {x 11 10 9 8 7 6 5}
    // [1] = {4  3  2 1 0 x x x} x = dummy fill
    value = (buf[0] << 5 | buf[1] >> 3);
  }

  // unselect
  HAL_GPIO_WritePin(TSC2046_CS_PORT, TSC2046_CS_PIN, GPIO_PIN_SET);

  return value;
}

void TSC2046_Calibration()
{
  // start
  TOUCH_CAL_LCD_CLEAR();
  TOUCH_CAL_LCD_DRAW_STRING("Start Calibration", 20, 0);
  TSC2046_TOUCH_CONFIRM();
  HAL_Delay(100);

  // points
  uint16_t points[9][2] = {
      {40, 40},
      {120, 40},
      {200, 40},
      {40, 160},
      {120, 160},
      {200, 160},
      {40, 280},
      {120, 280},
      {200, 280}
  };
  uint16_t values[9][2];

  uint8_t i,j,skip = 3;
  uint32_t tmpx,tmpy,ta,tb;

  for (i = 0; i < 9; i++)
  {
    // draw marker
    TOUCH_CAL_LCD_DRAW_CIRCLE(points[i][0], points[i][1], 5, 0);

    // read
    j = 0;
    tmpx = 0;
    tmpy = 0;
    while (!Is_Touhcing());
    while (j < 100 && Is_Touhcing())
    {
      ta = TSC2046_Read(TSC2046_COMMAND_READ_X);
      tb = TSC2046_Read(TSC2046_COMMAND_READ_Y);
      if (j>skip)
      {
        tmpx += ta;
        tmpy += tb;
      }
      j++;
    }
    while(Is_Touhcing());

    values[i][0] = (uint16_t)(tmpx / (j - skip));
    values[i][1] = (uint16_t)(tmpy / (j - skip));
  }

  // scale
  double sx = 0, sy = 0;
  double dpx,dpy,dvx,dvy, sxdiv = 0, sydiv = 0;

  for (i = 1; i < 9; i++)
  {
    dpx = points[i][0] - points[0][0];
    dvx = values[i][0] - values[0][0];
    dpy = points[0][1] - points[i][1];
    dvy = values[0][1] - values[i][1];

    if (dpx != 0 && dvx != 0)
    {
      sx += fabs(dpx / dvx);
      sxdiv++;
    }
    if (dpy != 0 && dvy != 0)
    {
      sy += fabs(dpy / dvy);
      sydiv++;
    }
  }
  sx /= sxdiv;
  sy /= sydiv;

  // offset
  uint16_t minx = 4096, miny = 4096;
  for (i = 0; i < 9; i++)
  {
    if (minx > values[i][0])
    {
      minx = values[i][0];
    }
    if (miny > values[i][1])
    {
      miny = values[i][1];
    }
  }
  double ox = (double)(minx * sx) - points[0][0];
  double oy = (double)(miny * sy) - points[0][1];

  // result
  //char buf[100];
  //sprintf(buf, "scale (%.04f, %.04f)", sx, sy);
  //TOUCH_CAL_LCD_DRAW_STRING(buf, 20, 10);
  //sprintf(buf, "offset(%d, %d)", (int32_t)ox, (int32_t)oy);
  //TOUCH_CAL_LCD_DRAW_STRING(buf, 20, 22);

  // save
  scaleX = sx;
  scaleY = sy;
  offsetX = (int16_t)ox;
  offsetY = (int16_t)oy;
  // TSC2046_Save_Calibration();
}

// PENIRQ
#ifdef TSC2046_IRQ_EXTI_ENABLE
volatile uint8_t TSC2046_Touch_Status = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  // nPENIRQ - PC5
  case TSC2046_IRQ_PIN:
    TSC2046_Touch_Status = HAL_GPIO_ReadPin(TSC2046_IRQ_PORT, TSC2046_IRQ_PIN) == GPIO_PIN_RESET ?
        TSC2046_STATUS_TOUCHING : TSC2046_STATUS_RELEASE;
    break;
  }
}
#endif

// touch status
uint8_t Is_Touhcing()
{
#ifdef TSC2046_IRQ_PIN
  return TSC2046_Touch_Status;
#else
  return HAL_GPIO_ReadPin(TSC2046_IRQ_PORT, TSC2046_IRQ_PIN) == GPIO_PIN_RESET ?
      TSC2046_STATUS_TOUCHING : TSC2046_STATUS_RELEASE;
#endif
}

// to position
uint16_t Touch_Get_LCD_X()
{
  uint8_t i;
  int32_t x = 0;
  for (i = 0; i < TSC2046_MAX_SAMPLE && Is_Touhcing(); i++)
  {
    x += TSC2046_Read(TSC2046_COMMAND_READ_X);
  }
  x = (x / i) * scaleX - offsetX;
  return (x > TSC2046_WIDTH - 1 ? TSC2046_WIDTH - 1 : (x < 0 ? 0 : x));
}

uint16_t Touch_Get_LCD_Y()
{
  uint8_t i;
  int32_t y = 0;
  for (i = 0; i < TSC2046_MAX_SAMPLE && Is_Touhcing(); i++)
  {
    y += TSC2046_Read(TSC2046_COMMAND_READ_Y);
  }
  y = LCD_HEIGHT - ((y / i) * scaleY - offsetY);
  return (y > TSC2046_HEIGHT - 1 ? TSC2046_HEIGHT - 1 : (y < 0 ? 0 : y));
}

