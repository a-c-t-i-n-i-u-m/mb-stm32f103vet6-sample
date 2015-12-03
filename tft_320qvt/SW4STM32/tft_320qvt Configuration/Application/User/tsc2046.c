/*
 * touch.c
 */

#include <stdlib.h>
#include <math.h>
#include "tsc2046.h"

/**
 * TSC2046 Library for STM32
 *  datasheet: http://www.ti.com/lit/ds/symlink/tsc2046.pdf
 */

/**
 * Initialize
 */
void _TSC2046_Load_Cal_Data();
void TSC2046_Init()
{
  _TSC2046_Load_Cal_Data();
}

/**
 * communicate via SPI interface
 *  args
 *    uint8_t command:  Control byte; send to TSC2046
 *  return
 *    uint16_t:         raw value from TSC2046
 */
// read value
uint16_t TSC2046_Read(uint8_t command)
{
  // select chip
  HAL_GPIO_WritePin(TSC2046_CS_PORT, TSC2046_CS_PIN, GPIO_PIN_RESET);

  // send control byte
  HAL_SPI_Transmit(TSC2046_SPI_INSTANCE, &command, 1, 0xff);

  // wait
  HAL_Delay(TSC2046_SPI_BUSY_WAIT_DURATION);

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

#ifdef TSC2046_DEBUG
  // debug
  char debugbuf[15];
  sprintf(debugbuf, "read %04d\r\n", value);
  TSC2046_DEBUG_PRINT(debugbuf);
#endif

  return value;
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


/**
 * Reducing Analog Input Noise in Touch Screen Systems
 * http://www.ti.com/lit/an/sbaa155a/sbaa155a.pdf
 *  args
 *   uint8_t command: Control byte
 *  return
 *   int16_t: if -1, error, or, 8bit 0-(2^8-1) / 12bit 0-(2^12-1)
 */
int16_t TSC2046_Read_Filter(uint8_t command)
{
  int16_t result = 0, i = 0;
  uint16_t buf[TSC2046_MAX_SAMPLE];

#if TSC2046_SAMPLING_MODE == 0
  // simple average
  uint32_t value = 0;
  while (++i <= TSC2046_MAX_SAMPLE && Is_Touhcing())
  {
    value += TSC2046_Read(command);
  }
  result = value / i;
#elif TSC2046_SAMPLING_MODE == 1
  // weighted average
  for (i = 0; i < TSC2046_MAX_SAMPLE && Is_Touhcing(); i++)
  {
    buf[i] = TSC2046_Read(command);
  }
  if (i < 2)
  {
    result = -1;
  }
  else
  {
    uint8_t j;
    uint32_t t = 0;
    for (j = 1; j < i - 1; j++)
    {
      t += TSC2046_Read(command);
    }
    result = t / (i - 2);
  }
#elif TSC2046_SAMPLING_MODE == 2
  // median
  for (i = 0; i < TSC2046_MAX_SAMPLE && Is_Touhcing(); i++)
  {
    buf[i] = TSC2046_Read(command);
  }
  // sort
  int16_t temp, n = i, j;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (buf[j] < buf[i])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  // get median
  i = n / 2;
  if (n % 2 == 0)
  {
    result = (buf[i] + buf[i + 1]) / 2;
  }
  else
  {
    result = buf[i];
  }
#elif TSC2046_SAMPLING_MODE == 3
  // averaging the closest data
  for (i = 0; i < TSC2046_MAX_SAMPLE && Is_Touhcing(); i++)
  {
    buf[i] = TSC2046_Read(command);
  }
  // diff
  int n = i - 1, diff[n];
  for (i = 0; i < n; i++)
  {
    diff[i] = abs(buf[i + 1] - buf[i]);
  }
  // search minimum diff
  int di = -1, dm = 4096;
  for (i = 0; i < n; i++)
  {
    if (dm > diff[i])
    {
      dm = diff[i];
      di = i;
    }
  }
  // set average
  result = (buf[di] + buf[di + 1]) / 2;

#elif TSC2046_SAMPLING_MODE == 4
  // pressure
  uint16_t z1 = TSC2046_Read(TSC2046_CB_Z1_POSITION),
           z2 = TSC2046_Read(TSC2046_CB_Z2_POSITION),
           x = TSC2046_Read(TSC2046_CB_X_POSITION),
           q = command & TSC2046_CB_8BIT ? 256 : 4096,
           rx = 1000;// Rx X-plate resistance, temporary
  // when z1 == 0, released
  int16_t pressure = z1 > 0 ? rx * x / q * (z2 / z1 - 1) : -1;
  // if p<th, error
  result = pressure > TSC2046_PRESSURE_THRESHOLD ? TSC2046_Read(command) : -1;
#endif

  return result;
}


/**
 * Calibration in touch-screen systems
 */

#if TSC2046_CALIBRATION_MODE == 0
typedef struct {
  int16_t sx;
  int16_t sy;
  int16_t vx;
  int16_t vy;
} _TSC2046_Cal_Points;

typedef struct {
  double ax;
  double bx;
  double dx;
  double ay;
  double by;
  double dy;
} _TSC2046_Cal_Matrix;

volatile _TSC2046_Cal_Matrix _mcal = {1, 0, 0, 0, 1, 0};

void _TSC2046_Load_Cal_Data()
{
  // _mcal = read_from_flash(addr);
}

void _TSC2046_Save_Cal_Data()
{
  // write_to_flash(addr, &_mcal, sizeof(_mcal));
}

// three point calibration
void TSC2046_Calibration()
{
  // message
  TOUCH_CAL_LCD_CLEAR();
  TOUCH_CAL_LCD_DRAW_STRING("Start Calibration...", 10, 10);
  while (!Is_Touhcing());
  while (Is_Touhcing());

  // message
  TOUCH_CAL_LCD_CLEAR();

  // generate marker points
  uint16_t i = 0,
    gx = TSC2046_WIDTH / 4,
    gy = TSC2046_HEIGHT / 4;

  _TSC2046_Cal_Points p[3] = {
      {gx, gy},
      {gx *3, gy * 2},
      {gx, gy * 3}
  };
  while (i < 3)
  {
    // show marker
    TOUCH_CAL_LCD_DRAW_STRING("Please touch marker...", 10, 10);
    TOUCH_CAL_LCD_DRAW_CIRCLE(p[i].sx, p[i].sy, 5);

    // read
    while (!Is_Touhcing());
    p[i].vx = TSC2046_Read_Filter(TSC2046_COMMAND_READ_X);
    p[i].vy = TSC2046_Read_Filter(TSC2046_COMMAND_READ_Y);

    // wait release
    while (Is_Touhcing());

    // check value
    if (p[i].vx > 0 && p[i].vy > 0)
    {
      TOUCH_CAL_LCD_CLEAR();
      i++;
    }
    else
    {
      TOUCH_CAL_LCD_DRAW_STRING("Touch error, please retry.", 10, 20);
    }
  }

  // calculate matrix determinant
  double detA = (p[0].vx - p[2].vx) * (p[1].vy - p[2].vy)
      - (p[1].vx - p[2].vx) * (p[0].vy - p[2].vy);
  double detAx1 = (p[0].sx - p[2].sx) * (p[1].vy - p[2].vy)
      - (p[1].sx - p[2].sx) * (p[0].vy - p[2].vy);
  double detAx2 = (p[0].vx - p[2].vx) * (p[1].sx - p[2].sx)
      - (p[1].vx - p[2].vx) * (p[0].sx - p[2].sx);
  double detAx3 = p[0].sx *(p[1].vx * p[2].vy - p[2].vx * p[1].vy)
      - p[1].sx *(p[0].vx * p[2].vy - p[2].vx * p[0].vy)
      + p[2].sx *(p[0].vx * p[1].vy - p[1].vx * p[0].vy);
  double detAy1 = (p[0].sy - p[2].sy) * (p[1].vy - p[2].vy)
      - (p[1].sy - p[2].sy) * (p[0].vy - p[2].vy);
  double detAy2 = (p[0].vx - p[2].vx) * (p[1].sy - p[2].sy)
      - (p[1].vx - p[2].vx) * (p[0].sy - p[2].sy);
  double detAy3 = p[0].sy *(p[1].vx * p[2].vy - p[2].vx * p[1].vy)
        - p[1].sy *(p[0].vx * p[2].vy - p[2].vx * p[0].vy)
        + p[2].sy *(p[0].vx * p[1].vy - p[1].vx * p[0].vy);

  // save
  _mcal.ax = detAx1 / detA;
  _mcal.bx = detAx2 / detA;
  _mcal.dx = detAx3 / detA;
  _mcal.ay = detAy1 / detA;
  _mcal.by = detAy2 / detA;
  _mcal.dy = detAy3 / detA;

   _TSC2046_Save_Cal_Data();
}

// get calibrated values
int16_t TSC2046_Get_Position_X()
{
  // X = alphaX * rawX + betaX * rawY + deltaX;
  int16_t rawX = TSC2046_Read_Filter(TSC2046_COMMAND_READ_X),
          rawY = TSC2046_Read_Filter(TSC2046_COMMAND_READ_Y);
  if (rawX < 0 || rawY < 0)
  {
    return -1;
  }
  int16_t x = _mcal.ax * rawX + _mcal.bx * rawY + _mcal.dx;

  // clip to LCD resolution
  return x < 0 ? 0 : (x < TSC2046_WIDTH ? x : TSC2046_WIDTH - 1);
}

int16_t TSC2046_Get_Position_Y()
{
  // Y = alphaY * rawX + betaY * rawY + deltaY;
  uint16_t rawX = TSC2046_Read_Filter(TSC2046_COMMAND_READ_X),
           rawY = TSC2046_Read_Filter(TSC2046_COMMAND_READ_Y);
  if (rawX < 0 || rawY < 0)
  {
    return -1;
  }
  int16_t y = _mcal.ay * rawX + _mcal.by * rawY + _mcal.dy;

  // clip to LCD resolution
  return y < 0 ? 0 : (y < TSC2046_HEIGHT ? y : TSC2046_HEIGHT - 1);
}

#elif TSC2046_CALIBRATION_MODE == 1

#define _TSC2046_N TSC2046_CAL_DEG+1
#define _TSC2046_S TSC2046_CAL_POINTS

void _TSC2046_Solve(int16_t* x, int16_t* y, double* a)
{
  int i,j,k;
  double m[_TSC2046_N][_TSC2046_N+1];

  for (i = 0; i < _TSC2046_N; i++)
  {
    for (j = 0; j < _TSC2046_N + 1; j++)
    {
      m[i][j] = 0;
    }
  }

  for (i = 0; i < _TSC2046_N; i++)
  {
    for (j = 0; j < _TSC2046_N; j++)
    {
      for (k = 0; k < _TSC2046_S; k++)
      {
        m[i][j] += pow(x[k], i + j);
      }
    }
  }

  for (i = 0; i < _TSC2046_N; i++)
  {
    for (k = 0; k < _TSC2046_S; k++)
    {
      m[i][_TSC2046_N] += pow(x[k], i) * y[k];
    }
  }

  int l,pivot;
  double p,q,r,b[1][_TSC2046_N+1];

  for (i = 0; i < _TSC2046_N; i++)
  {
    r = 0;
    pivot = i;
    for (l = i; l < _TSC2046_N; l++)
    {
      if (fabs(m[l][i]) > r)
      {
        r = fabs(m[l][i]);
        pivot = l;
      }
    }
    if (pivot != i)
    {
      for (j = 0; j < _TSC2046_N + 1; j++)
      {
        b[0][j] = m[i][j];
        m[i][j] = m[pivot][j];
        m[pivot][j] = b[0][j];
      }
    }
  }

  for (k = 0; k < _TSC2046_N; k++)
  {
    p = m[k][k];
    m[k][k] = 1;

    for (j = k + 1; j < _TSC2046_N + 1; j++)
    {
      m[k][j] /= p;
    }

    for (i = k + 1; i < _TSC2046_N; i++)
    {
      q = m[i][k];
      for (j = k + 1; j < _TSC2046_N + 1; j++)
      {
        m[i][j] -= (double)(q * m[k][j]);
      }
      m[i][k] = 0;
    }
  }

  for (i = _TSC2046_N - 1; i >= 0; i--)
  {
    a[i] = m[i][_TSC2046_N];
    for (j = _TSC2046_N - 1; j > i; j--)
    {
      a[i] -= (double)(m[i][j] * a[j]);
    }
  }
}

double _TSC2046_Cal_x[_TSC2046_N];
double _TSC2046_Cal_y[_TSC2046_N];

void _TSC2046_Load_Cal_Data()
{
  // _TSC2046_Cal_x = read_from_flash(addr1);
  // _TSC2046_Cal_y = read_from_flash(addr2);
}

void _TSC2046_Save_Cal_Data()
{
  // write_to_flash(addr1, _TSC2046_Cal_x, _TSC2046_N);
  // write_to_flash(addr2, _TSC2046_Cal_y, _TSC2046_N);
}

void TSC2046_Calibration()
{
  // message
  TOUCH_CAL_LCD_CLEAR();
  TOUCH_CAL_LCD_DRAW_STRING("Start Calibration...", 10, 10);
  while (!Is_Touhcing());
  while (Is_Touhcing());
  TOUCH_CAL_LCD_CLEAR();

  // get values
  int16_t x[_TSC2046_S], sx[_TSC2046_S],
          y[_TSC2046_S], sy[_TSC2046_S];

  uint16_t i = 0,
      dx = TSC2046_WIDTH / (_TSC2046_S + 1),
      dy = TSC2046_HEIGHT / (_TSC2046_S + 1);

  while (i < _TSC2046_S)
  {
    // show marker
    sx[i] = dx * (i + 1);
    sy[i] = dy * (i + 1);
    TOUCH_CAL_LCD_DRAW_STRING("Please touch marker...", 10, 10);
    TOUCH_CAL_LCD_DRAW_CIRCLE(sx[i], sy[i], 5);

    // read
    while (!Is_Touhcing());
    x[i] = TSC2046_Read_Filter(TSC2046_COMMAND_READ_X);
    y[i] = TSC2046_Read_Filter(TSC2046_COMMAND_READ_Y);

    // wait release
    while (Is_Touhcing());

    // check value
    if (x[i] > 0 && y[i] > 0)
    {
      TOUCH_CAL_LCD_CLEAR();
      i++;
    }
    else
    {
      TOUCH_CAL_LCD_DRAW_STRING("Touch error, please retry.", 10, 20);
    }
  }

  // calcurate X
  _TSC2046_Solve(x, sx, _TSC2046_Cal_x);

  // calcurate Y
  _TSC2046_Solve(y, sy, _TSC2046_Cal_y);

  // save
  _TSC2046_Save_Cal_Data();

  TOUCH_CAL_LCD_CLEAR();
}

// get calibrated values
int16_t TSC2046_Get_Position_X()
{
  // interpolated value
  int16_t rawX = TSC2046_Read_Filter(TSC2046_COMMAND_READ_X), i;
  if (rawX < 0)
  {
    return -1;
  }
  double x = 0;
  for (i = 0; i < _TSC2046_N; i++) {
    x += (double)(_TSC2046_Cal_x[i] * pow(rawX, i));
  }

  // clip to LCD resolution
  return x < 0 ? 0 : (x < TSC2046_WIDTH ? x : TSC2046_WIDTH - 1);
}

int16_t TSC2046_Get_Position_Y()
{
  // interpolated value
  uint16_t rawY = TSC2046_Read_Filter(TSC2046_COMMAND_READ_Y), i;
  if (rawY < 0)
  {
    return -1;
  }
  double y = 0;
  for (i = 0; i < _TSC2046_N; i++)
  {
    y += (double)(_TSC2046_Cal_y[i] * pow(rawY, i));
  }

  // clip to LCD resolution
  return y < 0 ? 0 : (y < TSC2046_HEIGHT ? y : TSC2046_HEIGHT - 1);
}
#endif
