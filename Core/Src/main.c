/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EPD.h"
#include "EPD_GUI.h"
#include "../Fonts/fonts.h"
#include "bitmaps.h"
#include "stdlib.h"
#include "stdbool.h"
#include "ds18b20.h"
#include "queue.h"
#include "buttons.h"
#include "time.h"
#include "usbd_cdc_if.h"
#include "USB_storage.h"
// #include "USB_storage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define VREFINT_CAL_ADDR    ((uint32_t*)0x1FFF75AA)
#define MEASURMENTS_DELTA_SEC 10
#define SAMPLES_PER_GRAPH 48
#define IMG_SIZE  ((EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1)) * EPD_HEIGHT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define roundUpToDiv(x, div) (((x) + ((x) < 0 ? -(div - 1) : (div - 1))) / (div) * (div))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// int16_t data[60];
// uint8_t data_write_ptr = 0;
// bool is_first_writing = true;

float mcuVoltage = 0;
uint16_t adcData = 0;

DS18B20 temperatureSensor;

float temp;

//uint8_t *BlackImage;
// uint16_t Imagesize = ((EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1)) * EPD_HEIGHT;
uint8_t BlackImage[IMG_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
long map(long x, long in_min, long in_max, long out_min, long out_max);
void plot();
void shutdown(bool skipRTC);
void power_on();
float DS18_GET();
void DS18_INIT();
void set_time();
void get_data(bool goto_sleep);
void timestamp_to_time(uint32_t timestamp, uint8_t* hours, uint8_t* minutes);
bool USB_DEVICE_IsConnected();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool send_usb;
bool measure;
bool initialise_usb_connection;
uint32_t usb_conn_tmr;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData, 1);
  DS18_INIT();
  EPD_Init();

  q_init();

 bool is_RTC_retained = q_load();

 bool old_usb_state = false;

  if (is_RTC_retained) {
    shutdown(true);
    power_on();
  }else{
    set_time();
  }

  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  q_save();
  plot();
  if(!USB_DEVICE_IsConnected()){
    shutdown(false);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    if(measure){
      measure = false;
      get_data(!send_usb);
      send_usb = false;
    }
    
    if(initialise_usb_connection){
      initialise_usb_connection = false;
      power_on();
      usb_conn_tmr = HAL_GetTick();

      // uint32_t usbinit_tmr = HAL_GetTick();
      // while(HAL_GetTick()-usbinit_tmr < 5000){
      //   if(measure){
      //     measure = false;
      //     get_data(!send_usb);
      //     send_usb = false;
      //   }
      //   if(USB_DEVICE_IsConnected()){break;}
      // }

      // uint32_t sleep_time = HAL_RTCEx_GetWakeUpTimer(&hrtc);
      // HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleep_time, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
    }

    if(!USB_DEVICE_IsConnected() && old_usb_state && HAL_GetTick()-usb_conn_tmr>150){
      shutdown(true);
    }
    old_usb_state = USB_DEVICE_IsConnected();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  uint32_t RTC_key =  0xAAFF55FF;
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) == RTC_key){

  }
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x25;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|CS_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OK_Pin */
  GPIO_InitStruct.Pin = OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin DC_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUSY_Pin DOWN_Pin UP_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin|DOWN_Pin|UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB4 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_Pin */
  GPIO_InitStruct.Pin = PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_WKUP_Pin */
  GPIO_InitStruct.Pin = USB_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void plot(){
  EPD_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, 90, WHITE);  	
  EPD_SelectImage(BlackImage);
  EPD_Clear(WHITE);
  int16_t min = q_get(0);
  int16_t max = q_get(0);

  for (uint8_t i = 0; i < q_len(); i++){
    if(q_get(i) <= min){
      min = q_get(i);
    }
    if(q_get(i) >= max){
      max = q_get(i);
    }
  }

  if(min == max){
    min-=10;
    max+=10;
  }

  if(min < 0 && max > 0){
    EPD_DrawLine(10, map(0, min, max, 110, 22), 250, map(0, min, max, 110, 22), BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  }

  uint16_t dt;
  switch (max - min)
  {
  case 2000 ... 4000:
    dt = 1000;
    break;
  case 800 ... 1999:
    dt = 500;
    break;
  case 400 ... 799:
    dt = 200;
    break;
  case 250 ... 399:
    dt = 100;
    break;
  case 100 ... 249:
    dt = 50;
    break;
  case 50 ... 99:
    dt = 20;
    break;
  case 25 ... 49:
    dt = 10;
    break;
  case 0 ... 24:
    dt = 5;
    break;
  default:
    dt = 2000;
    break;
  }

  for (int16_t x = roundUpToDiv(min, dt); x < max+dt; x += dt){
    bool st = LINE_STYLE_DOTTED;
    if(x == 0){st = LINE_STYLE_SOLID;}
    if(x > max && map(x, min, max, 110, 22) < 22){break;}
    EPD_DrawLine(10, map(x, min, max, 110, 22), 250, map(x, min, max, 110, 22), BLACK, DOT_PIXEL_1X1, st);
    char buf[15];
    uint8_t ln;
    if(x%100 == 0){
      ln = sprintf(buf, "%d", x/100);
    }
    else if(x%10 == 0 && max-min < 100){
      uint16_t rounded = (x + 5) / 10;
      uint8_t int_part = rounded / 10;
      uint8_t frac_part = rounded % 10;
      ln = sprintf(buf, "%d.%d", int_part, frac_part);
    }else if(x%5 == 0 && max-min < 10){
      uint8_t int_part = x / 100;
      uint8_t frac_part = x % 100;
      ln = sprintf(buf, "%d.%d", int_part, frac_part);
    }
    else{continue;}
    
    EPD_SetRotate(0);
    EPD_DrawString_EN(map(x, min, max, 2, 88)+8-(2*ln), 0,  buf, &Font8, WHITE, BLACK);
    EPD_SetRotate(90);
  }

  // Plot graf
  uint32_t timestamp = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);
  for (uint8_t i = 0; i < q_len(); i++){
    EPD_DrawLine(10+(i*5), map(q_get(i), min, max, 110, 22), 10+((i+1)*5), map(q_get(i+1), min, max, 110, 22), BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  }
  
  for (int8_t i = q_len(); i > 0; i-=8){
    if(i == q_len()){continue;}
    EPD_DrawLine(10+(i*5), 20, 10+(i*5), 112, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    uint32_t this_time = timestamp - 1800*(q_len()-i-1);
    // struct tm  ts;
    // ts = *localtime((time_t*) &this_time);
    // char t_buf[15];
    // sprintf(t_buf, "%.2d:%.2d", ts.tm_hour, ts.tm_min); 
    // EPD_DrawString_EN(10+(i*5), 114, t_buf, &Font8, WHITE, BLACK);

    char timest_buf[15];
    // sprintf(timest_buf, "%d", this_time-1740000000);
    // EPD_DrawString_EN(10+(i*5), 114, timest_buf, &Font8, WHITE, BLACK);

    uint8_t hours, minutes;
    timestamp_to_time(this_time, &hours, &minutes);
    sprintf(timest_buf, "%d:%d", hours, minutes);
    EPD_DrawString_EN(10+(i*5), 114, timest_buf, &Font8, WHITE, BLACK);
  }

  char buf[12];
  if(dt < 100 && dt >= 10){
    sprintf(buf, "%d.%d*C/div", dt/100, dt-(dt/100));
  }else if(dt < 10){
    sprintf(buf, "%d.0%d*C/div", dt/100, dt-(dt/100));
  }else{
    sprintf(buf, "%d*C/div", dt/100);
  }
  EPD_DrawString_EN(0, 0, buf, &Font12, WHITE, BLACK);

  mcuVoltage = 0xFFF * 1.18 / adcData;

  // char v_buf[20];
  // sprintf(v_buf, "%d %d", (uint32_t)(mcuVoltage*100), adcData);
  // EPD_DrawString_EN(80, 0, v_buf, &Font12, WHITE, BLACK);
  if(mcuVoltage < 2.25){
    EPD_SetRotate(0);
    EPD_DrawBMP(107, 95, 16, 26, dead_batt_26x16);
    EPD_SetRotate(90);
  }


  EPD_DrawLine(10, 112, 250, 112, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  EPD_DrawLine(10, 20, 10, 112, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  EPD_TIME time;
  time.Day = sDate.Date;
  time.Month = sDate.Month;
  time.Year = sDate.Year;
  EPD_DrawDate(175, 0, &time, &Font16, WHITE, BLACK);

  //
  // char timest_buf[15];
  // sprintf(timest_buf, "%d", timestamp);
  // EPD_DrawString_EN(85, 35, timest_buf, &Font12, WHITE, BLACK);
  //
  char temp_buf[10];
  sprintf(temp_buf, "%d.%dC", q_get(q_len()-1)/100, q_get(q_len()-1)%100);
  EPD_DrawString_EN(85, 0, temp_buf, &Font16, WHITE, BLACK);
  
  EPD_Display_Base(BlackImage);
  HAL_Delay(100);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if(in_min == in_max){
    return (out_max+out_min)/2;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void shutdown(bool skipRTC){
  EPD_Sleep();
  EPD_Exit();

  __HAL_RCC_SPI1_CLK_DISABLE();
  HAL_SPI_DeInit(&hspi1);
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_SuspendTick();
  HAL_NVIC_DisableIRQ(SysTick_IRQn);
  // uint32_t sleep_time = MEASURMENTS_DELTA_SEC;
  // if(skipRTC && ((RTC->CR & RTC_CR_WUTE) != 0)){
  //   sleep_time = HAL_RTCEx_GetWakeUpTimer(&hrtc);
  // }
  // HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleep_time, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
  HAL_PWREx_EnableLowPowerRunMode();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

void power_on(void){
  // HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_ResumeTick();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  EPD_Init();
}

void DS18_INIT(){
  DS18B20_Init(&temperatureSensor, &huart2);
  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_ReadRom(&temperatureSensor);
  DS18B20_ReadScratchpad(&temperatureSensor);
  uint8_t settings[3];
  settings[0] = temperatureSensor.temperatureLimitHigh;
  settings[1] = temperatureSensor.temperatureLimitLow;
  settings[2] = DS18B20_12_BITS_CONFIG;
  DS18B20_InitializationCommand(&temperatureSensor);
  DS18B20_SkipRom(&temperatureSensor);
  DS18B20_WriteScratchpad(&temperatureSensor, settings);
}

float DS18_GET(){
	DS18B20_InitializationCommand(&temperatureSensor);
	DS18B20_SkipRom(&temperatureSensor);
	DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);
	DS18B20_InitializationCommand(&temperatureSensor);
	DS18B20_SkipRom(&temperatureSensor);
	DS18B20_ReadScratchpad(&temperatureSensor);
	return temperatureSensor.temperature;
}

void set_time(){
  bool flag = true; // true ONLY FOR DEBUG!!!
  Button bt_ok = {GPIOA, GPIO_PIN_0, TYPE_LOW_PULL};
  Button bt_down = {GPIOA, GPIO_PIN_8, TYPE_LOW_PULL};
  Button bt_up = {GPIOA, GPIO_PIN_9, TYPE_LOW_PULL};

  int8_t d_time[5] = {25, 03, 24, 12, 00};
  uint8_t up_lim[5] = {99, 12, 31, 23, 59};
  uint8_t down_lim[5] = {0, 1, 1, 0, 0};
  uint8_t edit_ptr = 0;

  uint32_t upd_tmr = HAL_GetTick();
  bool edited;
  // FIX THIS
  tick(&bt_ok);
  tick(&bt_up);
  tick(&bt_down);
  isClicked(&bt_ok);
  isClicked(&bt_down);
  isClicked(&bt_up);
  /*                 */
  while(!flag){
    EPD_NewImage(BlackImage, EPD_WIDTH, EPD_HEIGHT, 90, WHITE);  	
    EPD_SelectImage(BlackImage);
    EPD_Clear(WHITE);

    EPD_TIME datetime;
    datetime.Day = d_time[2];
    datetime.Month = d_time[1];
    datetime.Year = d_time[0];
    datetime.Hour = d_time[3];
    datetime.Min = d_time[4];

    EPD_DrawDate(64, 32, &datetime, &Font24, WHITE, BLACK);
    EPD_DrawTime(86, 64, &datetime, &Font24, WHITE, BLACK);
    
    if(edit_ptr < 3){
      EPD_DrawLine(150-(edit_ptr*42), 52, 150-(edit_ptr*42)+30, 52, BLACK, 2, LINE_STYLE_SOLID);
    }else{
      EPD_DrawLine(-38+(edit_ptr*42), 84, -38+(edit_ptr*42)+30, 84, BLACK, 2, LINE_STYLE_SOLID);
    } 
    EPD_Display_Base(BlackImage);

    edited = false;
    for(;;){
      tick(&bt_ok);
      tick(&bt_up);
      tick(&bt_down);
      if(isClicked(&bt_ok)){
        edit_ptr++;
        if(edit_ptr == 5){
          flag = true;
        }
        break;
      }
      if(isClicked(&bt_up)){
        d_time[edit_ptr]++;
        if(d_time[edit_ptr]> up_lim[edit_ptr]){
          d_time[edit_ptr] = down_lim[edit_ptr];
        }
        edited = true;
        upd_tmr = HAL_GetTick();
      }
      if(isClicked(&bt_down)){
        d_time[edit_ptr]--;
        if(d_time[edit_ptr]< down_lim[edit_ptr]){
          d_time[edit_ptr] = up_lim[edit_ptr];
        }
        edited = true;
        upd_tmr = HAL_GetTick();
      }

      if(HAL_GetTick() - upd_tmr > 1000 && edited){
        break;
      }
    }
  }
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  sTime.Hours = d_time[3];
  sTime.Minutes = d_time[4];
  sDate.Year = d_time[0];
  sDate.Month = d_time[1];
  sDate.Date = d_time[2];
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_ADD1H;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

void get_data(bool goto_sleep){
  if(goto_sleep){
    power_on();
  }else{
    CDC_Transmit_FS("Hello\r\n", 8);
  }

  q_load();
  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  q_push(DS18_GET()*100);
  plot();
  q_save();

  if(goto_sleep){
    shutdown(false);
  }
}

void timestamp_to_time(uint32_t timestamp, uint8_t* hours, uint8_t* minutes) {
  uint32_t seconds_in_day = 86400;
  uint32_t seconds_in_hour = 3600;
  uint32_t seconds_in_minute = 60;

  // Get seconds since midnight (UTC)
  uint32_t time_of_day = timestamp % seconds_in_day;

  *hours = time_of_day / seconds_in_hour;
  *minutes = (time_of_day % seconds_in_hour) / seconds_in_minute;
}


void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
  CDC_Transmit_FS(Buf, Len);
  // HAL_Delay(1);
  // char buf[15];
  // if(Buf[0] == 'G'){
  //   uint16_t len = UDISK_len();
  //   for (uint8_t i = 0; i < len; i++){
  //     memset(buf, 0x00, 15);
  //     uint8_t ln = sprintf(buf, "%d\n\r", UDISK_get(i));
  //     CDC_Transmit_FS(buf, ln);
  //     HAL_Delay(1);
  //   }
  // }
  // memset(buf, 0x00, 15);
  // uint8_t ln = sprintf(buf, "T%d\n\r", UDISK_tst());
  // CDC_Transmit_FS(buf, ln);
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
  measure = true;
  send_usb = USB_DEVICE_IsConnected();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_5) {
    initialise_usb_connection = true;
  } else {
      __NOP();
  }
}

bool USB_DEVICE_IsConnected(){
  return HAL_GPIO_ReadPin(USB_WKUP_GPIO_Port, USB_WKUP_Pin);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
