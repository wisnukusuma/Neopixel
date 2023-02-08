/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

#define NEOPIXEL_ZERO 30
#define NEOPIXEL_ONE 58
#define SK6812_DATA_SIZE 24
#define NUM_PIXELS 35
#define DMA_BUFF_SIZE (NUM_PIXELS *32)+1
#define MAX_FRAME_BUFFER 85
union PixelRGB_t
{
  struct
  {
	uint8_t g;
	uint8_t r;
	uint8_t b;
  } color;
  uint32_t data;
};

typedef union
{
  struct
  {
	uint8_t w;
	uint8_t b;
	uint8_t r;
	uint8_t g;
  } color;
  uint32_t data;
} PixelRGBW_t;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
//uint16_t color_to_rgb565( uint8_t r,uint8_t g,uint8_t b){
//
//
//}
void wheel (uint32_t pos, union PixelRGB_t *p){

	union PixelRGB_t pixel;
	if(pos < 85 ){
		p->color.r = pos * 3;
		p->color.g = 255 - pos *3;
		p->color.b = 0;
		return 0;
	}
	if(pos < 170 ){
		pos -= 85;
		p->color.r = 255 - pos *3;
		p->color.g = 0;
		p->color.b = pos * 3;
		return 0;
	}
	pos -= 170;
	p->color.r = 0;
	p->color.g = pos * 3;
	p->color.b = 255 - pos * 3;
	return 0;
}
unsigned char reverse(unsigned char b)
{
	const uint8_t reverseLookup[256] = { 0,128,64,192,32,160,96,224,16,144,80,208,48,176,112,240,8,136,72,200,40,168,104,232,24,152,88,216,56,184,120,248,4,132,68,196,36,164,100,228,20,148,84,212,52,180,116,244,12,140,76,204,44,172,108,236,28,156,92,220,60,188,124,252,2,130,66,194,34,162,98,226,18,146,82,210,50,178,114,242,10,138,74,202,42,170,106,234,26,154,90,218,58,186,122,250,6,134,70,198,38,166,102,230,22,150,86,214,54,182,118,246,14,142,78,206,46,174,110,238,30,158,94,222,62,190,126,254,1,129,65,193,33,161,97,225,17,145,81,209,49,177,113,241,9,137,73,201,41,169,105,233,25,153,89,217,57,185,121,249,5,133,69,197,37,165,101,229,21,149,85,213,53,181,117,245,13,141,77,205,45,173,109,237,29,157,93,221,61,189,125,253,3,131,67,195,35,163,99,227,19,147,83,211,51,179,115,243,11,139,75,203,43,171,107,235,27,155,91,219,59,187,123,251,7,135,71,199,39,167,103,231,23,151,87,215,55,183,119,247,15,143,79,207,47,175,111,239,31,159,95,223,63,191,127,255};
	return reverseLookup[b];
}
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim3_ch2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1 )
  {
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  }
}
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  uint16_t timer_val;
  /* USER CODE BEGIN Init */
  union PixelRGB_t pixel[NUM_PIXELS];
  uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
  uint32_t *pBuff;
  int i, j, k;
  i=0;
  j=0;
  uint16_t stepSize;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  k = 0;
  stepSize = 4;
  HAL_TIM_Base_Start_IT(&htim1);
  uint32_t counter;
  uint32_t pos_buff=0;

  while (1){
	  for(i = MAX_FRAME_BUFFER ; i>0; i--)
	  {
		  pBuff = dmaBuffer;
		  for(j = 0 ; j < NUM_PIXELS; j++){
		  pos_buff = j * 256 / NUM_PIXELS + i * 3;
		  wheel(pos_buff & 255, &pixel[j]);
		  pixel[j].color.r = reverse(pixel[j].color.r & 0xf8);
 		  pixel[j].color.g = reverse(pixel[j].color.g & 0xfc);
		  pixel[j].color.b = reverse(pixel[j].color.b & 0xf8);
		  for ( k = 0; k < SK6812_DATA_SIZE; k++ )
			  {
			   if( ( pixel[j].data >> k ) & 1 ) *pBuff = NEOPIXEL_ONE;
			   else *pBuff = NEOPIXEL_ZERO;
			   pBuff++;
			  }
		  }
		  dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0! (uint32_t*)
		  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
		  HAL_Delay(33);
	  }
    /* USER CODE END WHILE */
//	  for (i = (NUM_PIXELS - 1); i > 0; i--)
//	  {
//		pixel[i].data = pixel[i-1].data;
//	  }
//
//	  if (k < 255)
//	  {
//		pixel[0].color.g = 254 - k; //[254, 0]
//		pixel[0].color.r =  k + 1;  //[1, 255]
//		pixel[0].color.b = 0;
//	  }
//	  else if (k < 510)
//	  {
//		pixel[0].color.g = 0;
//		pixel[0].color.r = 509 - k; //[254, 0]
//		pixel[0].color.b = k - 254; //[1, 255]
//		j++;
//	  }
//	  else if (k < 765)
//	  {
//		pixel[0].color.g = k - 509; //[1, 255];
//		pixel[0].color.r = 0;
//		pixel[0].color.b = 764 - k; //[254, 0]
//	  }
//	  k = (k + stepSize) % 765;
//
//	  // not so bright
//	  pixel[0].color.g >>= 2;
//	  pixel[0].color.r >>= 2;
//	  pixel[0].color.b >>= 2;
//
//	  pBuff = dmaBuffer;
//	  for (i = 0; i < NUM_PIXELS; i++)
//	  {
//		 for (j = 23; j >= 0; j--)
//		 {
//		   if ((pixel[i].data >> j) & 0x01)
//		   {
//			 *pBuff = NEOPIXEL_ONE;
//		   }
//		   else
//		   {
//			 *pBuff = NEOPIXEL_ZERO;
//		   }
//		   pBuff++;
//	   }
//	  }
//	dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0! (uint32_t*)
//	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, dmaBuffer, DMA_BUFF_SIZE);
//    timer_val = __HAL_TIM_GET_COUNTER(&htim1);
//    if(timer_val>90000){
//    	counter++;
//    	if(counter > 1000){
//    		counter =0;
//    		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//    	}
//    }
//    timer_val = __HAL_TIM_GET_COUNTER(&htim1);
//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
//	HAL_Delay(100);
//	timer_val = __HAL_TIM_GET_COUNTER(&htim1)-timer_val;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CANTX_GPIO_Port, CANTX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CANTX_Pin */
  GPIO_InitStruct.Pin = CANTX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CANTX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
