/*
 * touch.h
 */

#ifndef APPLICATION_USER_TSC2046_H_
#define APPLICATION_USER_TSC2046_H_

#include "stm32f1xx_hal.h"

/**
 * Constant values for TSC2046_Read
 */

// pointer of SPI instance
extern SPI_HandleTypeDef hspi1;
#define TSC2046_SPI_INSTANCE &hspi1

// wait busy state(A/D conversion)
#define TSC2046_SPI_BUSY_WAIT_DURATION 3

// Chip select GPIO pin
#define TSC2046_CS_PORT GPIOC
#define TSC2046_CS_PIN GPIO_PIN_4

/**
 * PENIRQ Configuration
 */

// Handle EXTI interrupt
#define TSC2046_IRQ_EXTI_ENABLE

// EXTI port
#define TSC2046_IRQ_PORT GPIOC
#define TSC2046_IRQ_PIN GPIO_PIN_5

/**
 * sampling configuration
 *  mode
 *   0: Average; result = sum(N samples)/N
 *   1: Weighted Averaging; skip first and tail sample, result = sum(samples[1 to N-1])/(N-2)
 *   2: Middle Value; result = median(samples)
 *   3: Averaging the Closest Data; result = sum(closest_two_data(samples))/2
 *   4: Touch Screen Panel Pressure; if (pressure > threshold) { result = sample; }
 */
#define TSC2046_SAMPLING_MODE 2

// maximum number of sampling, for average method
#define TSC2046_MAX_SAMPLE 5

// pressure threshold for mode.4
#define TSC2046_PRESSURE_THRESHOLD 0

/**
 * Calibration
 */

// calibration mode
// 0. http://www.ti.com/lit/an/slyt277/slyt277.pdf
// 1. single value N degree least squares method
// 2. Linear interpolation
#define TSC2046_CALIBRATION_MODE 2

// maximum number of sampling, while calibration
#define TSC2046_CAL_MAX_SAMPLE 25

// parameter for mode.1
#define TSC2046_CAL_DEG 3     // degree of approximation curve
#define TSC2046_CAL_POINTS 8  // number of samples

// display resolution
#define TSC2046_WIDTH  240
#define TSC2046_HEIGHT 320

// LCD draw interface wrapper for calibration
#include "lcd.h"
#define TOUCH_CAL_LCD_CLEAR() {\
    LCD_fillScr(0xffff);\
}
#define TOUCH_CAL_LCD_DRAW_CIRCLE(x,y,r) {\
    LCD_setColor(VGA_BLUE);\
    LCD_fillCircle(x,y,r);\
}
#define TOUCH_CAL_LCD_DRAW_STRING(str,x,y) {\
    LCD_setColor(VGA_BLACK);\
    LCD_setBackColor(VGA_WHITE);\
    LCD_print(str,x,y,0);\
}

/**
 * definition of Control Byte parameters
 */

#define TSC2046_CB_START_BIT    (1<<7)

// input channel, for SER/DFR mode
#define TSC2046_CB_X_POSITION   (0b101<<4)
#define TSC2046_CB_Y_POSITION   (0b001<<4)
#define TSC2046_CB_Z1_POSITION  (0b011<<4)
#define TSC2046_CB_Z2_POSITION  (0b100<<4)

// input channel, for SER mode only
#define TSC2046_CB_TEMP0 (0b000<<4)
#define TSC2046_CB_TEMP1 (0b111<<4)
#define TSC2046_CB_VBAT  (0b010<<4)
#define TSC2046_CB_AUXIN (0b110<<4)

// ADC mode
#define TSC2046_CB_12BIT (0<<3)
#define TSC2046_CB_8BIT  (1<<3)
#define TSC2046_CB_SER   (1<<2)
#define TSC2046_CB_DFR   (0<<2)

// power down configuration                 // PENIRQ
#define TSC2046_CB_PD_AFTER_CONV     (0b00) // Enabled
#define TSC2046_CB_PD_REF_OFF_ADC_ON (0b01) // Disabled
#define TSC2046_CB_PD_REF_ON_ADC_OFF (0b10) // Enabled
#define TSC2046_CB_PD_ALWAYS_POWERED (0b11) // Disabled

// command presets
#define TSC2046_COMMAND_BASE (\
    TSC2046_CB_START_BIT|\
    TSC2046_CB_12BIT|\
    TSC2046_CB_DFR|\
    TSC2046_CB_PD_AFTER_CONV\
)
#define TSC2046_COMMAND_READ_X  (TSC2046_COMMAND_BASE|TSC2046_CB_X_POSITION)
#define TSC2046_COMMAND_READ_Y  (TSC2046_COMMAND_BASE|TSC2046_CB_Y_POSITION)
#define TSC2046_COMMAND_READ_Z1 (TSC2046_COMMAND_BASE|TSC2046_CB_Z1_POSITION)
#define TSC2046_COMMAND_READ_Z2 (TSC2046_COMMAND_BASE|TSC2046_CB_Z2_POSITION)

/**
 * Debug
 */
//#define TSC2046_DEBUG
//extern UART_HandleTypeDef huart1;
//#define TSC2046_DEBUG_PRINT(str) {\
//  HAL_UART_Transmit(&huart1, str, strlen(str), 0xff);\
//}

/**
 * Exports
 */

void TSC2046_Init();

#define TSC2046_STATUS_TOUCHING 1
#define TSC2046_STATUS_RELEASE 0
uint8_t Is_Touhcing();

uint16_t TSC2046_Read(uint8_t command);
int16_t TSC2046_Read_Filter(uint8_t command, uint8_t maxSample);
void TSC2046_Calibration();

int16_t TSC2046_Get_Position_X();
int16_t TSC2046_Get_Position_Y();

#endif /* APPLICATION_USER_TSC2046_H_ */
