/*
 * touch.h
 */

#ifndef APPLICATION_USER_TSC2046_H_
#define APPLICATION_USER_TSC2046_H_

#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1;
#define TSC2046_SPI_INSTANCE &hspi1

// Chip select
#define TSC2046_CS_PORT GPIOC
#define TSC2046_CS_PIN GPIO_PIN_4

// config
#define TSC2046_WIDTH 240
#define TSC2046_HEIGHT 320
#define TSC2046_MAX_SAMPLE 5

// Control Byte
#define TSC2046_CB_START_BIT    (1<<7)
#define TSC2046_CB_X_POSITION   (0b101<<4)
#define TSC2046_CB_Y_POSITION   (0b001<<4)
#define TSC2046_CB_Z1_POSITION  (0b011<<4)
#define TSC2046_CB_Z2_POSITION  (0b100<<4)
#define TSC2046_CB_12BIT        (0<<3)
#define TSC2046_CB_8BIT         (1<<3)
#define TSC2046_CB_SER          (1<<2)
#define TSC2046_CB_DFR          (0<<2)
#define TSC2046_CB_POWERDOWN_DEFAULT (0b00)

#define TSC2046_COMMAND_BASE (\
    TSC2046_CB_START_BIT|\
    TSC2046_CB_12BIT|\
    TSC2046_CB_DFR|\
    TSC2046_CB_POWERDOWN_DEFAULT\
)
#define TSC2046_COMMAND_READ_X  (TSC2046_COMMAND_BASE|TSC2046_CB_X_POSITION)
#define TSC2046_COMMAND_READ_Y  (TSC2046_COMMAND_BASE|TSC2046_CB_Y_POSITION)
#define TSC2046_COMMAND_READ_Z1 (TSC2046_COMMAND_BASE|TSC2046_CB_Z1_POSITION)
#define TSC2046_COMMAND_READ_Z2 (TSC2046_COMMAND_BASE|TSC2046_CB_Z2_POSITION)

// export
void TSC2046_Init();
uint16_t TSC2046_Read(uint8_t command);
void TSC2046_Calibration();

#define TSC2046_STATUS_TOUCHING 1
#define TSC2046_STATUS_RELEASE 0
uint8_t Is_Touhcing();
uint16_t Touch_Get_LCD_X();
uint16_t Touch_Get_LCD_Y();

// LCD interface for calibration
#include "lcd.h"
#define TOUCH_CAL_LCD_CLEAR() {\
    LCD_fillScr(0xffff);\
}
#define TOUCH_CAL_LCD_DRAW_CIRCLE(x,y,r,s) {\
    LCD_setColor(s ? VGA_RED : VGA_BLUE);\
    LCD_fillCircle(x,y,r);\
}
#define TOUCH_CAL_LCD_DRAW_STRING(str,x,y) {\
    LCD_setColor(VGA_BLACK);\
    LCD_setBackColor(VGA_WHITE);\
    LCD_print(str,x,y,0);\
}

// irq/status
#define TSC2046_IRQ_PORT GPIOC
#define TSC2046_IRQ_PIN GPIO_PIN_5
#define TSC2046_IRQ_EXTI_ENABLE// handle external interrupt

#define TSC2046_TOUCH_CONFIRM() {\
  while(!Is_Touhcing());\
  while(Is_Touhcing());\
}
#define TSC2046_WAIT_PENIRQ() {\
    while(!Is_Touhcing());\
}

#endif /* APPLICATION_USER_TSC2046_H_ */
