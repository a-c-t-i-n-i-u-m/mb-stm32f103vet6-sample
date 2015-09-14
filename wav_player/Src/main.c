/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS fs;
DIR dir;
FIL fp;
FILINFO fi;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/****************************************************
 * Debug Utilities
 *  ConsoleLog: send string via DEBUG_UART_HANDLE
 ****************************************************/

#define DEBUG_UART_HANDLE huart1

void ConsoleLog(const char* str) {
  HAL_UART_Transmit(&DEBUG_UART_HANDLE, str, strlen(str), 1);
}

/********************************************************************************
 * FIFO Audio buffer
 *  AudioBufferIsFull:  return 1 when buffer is full
 *  AudioBufferIsEmpty: return 1 when buffer is empty
 *  AudioBufferPush:    push data into buffer (busy wait while buffer is full)
 *  AudioBufferPop:     pop data from buffer (busy wait while buffer is empty)
 ********************************************************************************/

#define AUIDO_BUF_TYPE uint8_t
#define AUIDO_BUF_SIZE 512  // 2n
#define AUIDO_BUF_SIZE_MAX_INDEX (AUIDO_BUF_SIZE - 1)
volatile uint32_t AudioBufferWritePointer = 0;
volatile uint32_t AudioBufferReadPointer = 0;

uint8_t AudioPlayCancel = 0;
AUIDO_BUF_TYPE AudioBuffer[AUIDO_BUF_SIZE];

uint8_t AudioBufferIsFull(void) {
  return
      ((AudioBufferWritePointer + 1) & AUIDO_BUF_SIZE_MAX_INDEX)
          == AudioBufferReadPointer ? 1 : 0;
}

uint8_t AudioBufferIsEmpty(void) {
  return AudioBufferReadPointer == AudioBufferWritePointer ? 1 : 0;
}

void AudioBufferPush(AUIDO_BUF_TYPE val) {
  // wait
  while (AudioBufferIsFull() && AudioPlayCancel == 0);

  // set data
  AudioBuffer[AudioBufferWritePointer] = val;

  // increment
  AudioBufferWritePointer = (AudioBufferWritePointer + 1)
      & AUIDO_BUF_SIZE_MAX_INDEX;
}

AUIDO_BUF_TYPE AudioBufferPop(void) {
  // wait
  while (AudioBufferIsEmpty() && AudioPlayCancel == 0);

  // get data
  AUIDO_BUF_TYPE data = AudioBuffer[AudioBufferReadPointer];

  // increment
  AudioBufferReadPointer = (AudioBufferReadPointer + 1) & AUIDO_BUF_SIZE_MAX_INDEX;

  return data;
}

/********************************************************
 * Wave header parser
 *  return: data chunk length [bytes]; to read and play
 *  set: AudioChannels:     1 or 2     [channels]
 *       audioResolution:   8 or 16    [bits]
 *       AudioSamplingRate  <=AUDIO_SAMPLINGRATE_MAX [Hz]
 ********************************************************/

#define LD_DWORD(ptr) (DWORD)(((DWORD)*((BYTE*)(ptr)+3)<<24)|((DWORD)*((BYTE*)(ptr)+2)<<16)|((WORD)*((BYTE*)(ptr)+1)<<8)|*(BYTE*)(ptr))
#define FCC(c1,c2,c3,c4)    (((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)
#define wave_header FCC('W','A','V','E')
#define fmt_chunk   FCC('f','m','t',' ')
#define data_chunk  FCC('d','a','t','a')
#define LIST_chunk  FCC('L','I','S','T')
#define fact_chunk  FCC('f','a','c','t')

#define AUDIO_SAMPLINGRATE_MAX 48000

uint8_t AudioChannels = 0;
uint8_t AudioResolution = 0;
uint32_t AudioSamplingRate = 0;

uint32_t parse_wav_header(void) {
  uint32_t ChunkID, ChunkSize, readBytes;
  uint8_t buf[AUIDO_BUF_SIZE];

  if (f_read(&fp, buf, 12, &readBytes) != FR_OK)
    return 0;
  if (readBytes != 12 || LD_DWORD(buf+8) != wave_header)
    return 0;

  while (1) {
    f_read(&fp, buf, 8, &readBytes);
    if (readBytes != 8)
      return 0;
    ChunkID = LD_DWORD(&buf[0]);
    ChunkSize = LD_DWORD(&buf[4]);

    switch (ChunkID) {
    // get format information
    case fmt_chunk:
      // read chunk content
      if (ChunkSize > 100 || ChunkSize < 16)
        return 0;
      f_read(&fp, buf, ChunkSize, &readBytes);
      if (readBytes != ChunkSize)
        return 0;
      // Check coding type (PCM=1)
      if (buf[0] != 1)
        return 0;
      // channels
      AudioChannels = buf[2];
      if (AudioChannels != 1 && AudioChannels != 2)
        return 0;
      // resolution
      AudioResolution = buf[14];
      if (AudioResolution != 8 && AudioResolution != 16)
        return 0;
      // sampling rate
      AudioSamplingRate = LD_DWORD(buf + 4);
      if (AudioSamplingRate
          < HAL_RCC_GetHCLKFreq()
              / 65535|| AudioSamplingRate > AUDIO_SAMPLINGRATE_MAX)
        return 0;
      break;
      // get data length
    case data_chunk:
      return ChunkSize; // data length in bytes
      // skip
    default:
      f_lseek(&fp, fp.fptr + ChunkSize);
    }
  }
  return 0;
}

/********************************************************
 * Wave player
 *  filename: wave file to play
 ********************************************************/

#define CHANNEL_LEFT TIM_CHANNEL_1// or TIM_CHANNEL_2

uint32_t AudioLastOutputChannel = CHANNEL_LEFT;// Left
uint8_t AudioPause = 0;

void play(char* filename) {
  // open file
  if (f_open(&fp, filename, FA_READ) != FR_OK) {
    return;
  }

  // parse header
  uint32_t len = parse_wav_header();
  if (len == 0) {
    return;
  }

  // print play info
  char text[50];
  uint8_t slen = len / AudioSamplingRate / AudioChannels / AudioResolution * 8;
  sprintf(text, ">%s:  %d:%d  %luHz %ubit %uch\r\n", filename,
      slen / 60, slen % 60,
      AudioSamplingRate,
      AudioResolution, AudioChannels);
  ConsoleLog(text);

  // enable sampling clock
  htim1.Instance->ARR = HAL_RCC_GetHCLKFreq() / AudioSamplingRate;
  HAL_TIM_Base_Start_IT(&htim1);

  // enable PWM generator
  htim2.Instance->ARR = 255;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // read data
  uint8_t buf[AUIDO_BUF_SIZE];
  uint32_t rb, i;

  AudioPause = 0;
  AudioPlayCancel = 0;
  AudioBufferWritePointer = 0;
  AudioBufferReadPointer = 0;
  AudioLastOutputChannel = CHANNEL_LEFT;

  // unsigned 8bit integer L,R
  if (AudioResolution == 8) {
    while (AudioPlayCancel == 0)
    {
      if (f_read(&fp, buf, AUIDO_BUF_SIZE, &rb) != FR_OK)
      {
        break;
      }
      if (rb != AUIDO_BUF_SIZE)
      {
        break;
      }
      // push data
      for (i = 0; i < AUIDO_BUF_SIZE; i++)
        AudioBufferPush(buf[i]);
    }
  }
  // little endian signed 16bit integer, L,R
  else {
    int16_t sval;
    uint16_t seval;
    while (AudioPlayCancel == 0)
    {
      if (f_read(&fp, buf, AUIDO_BUF_SIZE, &rb) != FR_OK)
      {
        break;
      }
      if (rb != AUIDO_BUF_SIZE)
      {
        break;
      }
      // push data
      for (i = 0; i < AUIDO_BUF_SIZE; i += 2)
      {
        sval = buf[i] | (buf[i + 1] << 8);// get int16
        seval = sval < 0 ? (uint16_t)sval - 32768 : (uint16_t)sval + 32768;// to uint16
        AudioBufferPush(seval / 0x101);// to uint8
      }
    }
  }

  f_close(&fp);

  // wait play tail
  HAL_Delay(10);

  // break
  AudioPlayCancel = 1;

  // stop timers
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
}

uint32_t get_next_channel() {
  uint32_t next =
      AudioLastOutputChannel == TIM_CHANNEL_1 ? TIM_CHANNEL_2 : TIM_CHANNEL_1;
  AudioLastOutputChannel = next;
  return next;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  // no data to play
  if (AudioBufferIsEmpty() || AudioPause == 1) {
    return;
  }

  // set data
  AUIDO_BUF_TYPE data = AudioBufferPop();
  __HAL_TIM_SetCompare(&htim2, get_next_channel(), data);
  __HAL_TIM_SetCompare(&htim2, get_next_channel(), AudioChannels == 2 ? AudioBufferPop() : data);
}

// hardware switch handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case GPIO_PIN_5:
    // skip
    AudioPlayCancel = 1;
    break;
  case GPIO_PIN_4:
    // pause
    AudioPause ^= 1;
    break;
  case GPIO_PIN_3:
    break;
  case GPIO_PIN_2:
    break;
  }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  ConsoleLog("*** STM32F103VET6T SDIO+FATFS+PWM WAV PLAYER ***\r\n");

  // open disk
  if (f_mount(&fs, SD_Path, 0) != FR_OK) {
    ConsoleLog("Error: f_mount\r\n");
    return 1;
  }

  // open root directory
  if (f_opendir(&dir, "/") != FR_OK) {
    ConsoleLog("Error: f_opendir\r\n");
    return 1;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if (f_readdir(&dir, &fi) != FR_OK || !fi.fname[0]) {
      f_readdir(&dir, NULL);
      continue;
    }
    play(fi.fname);
  }

  f_closedir(&dir);
  f_mount(NULL, SD_Path, 0);

  return 0;
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
