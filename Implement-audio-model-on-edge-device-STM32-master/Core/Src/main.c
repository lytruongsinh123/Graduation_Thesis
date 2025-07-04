/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "feature_extraction.h"
#include "string.h"
#include <stdlib.h>
#include "math_helper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//-----MelSpectrogram---//
#define HOPLENGTH 		 	512
#define FILL_BUFFER_SIZE 	1024
#define NFFT             	1024
#define NMELS            	30
#define NAUDIO 		   		16896
#define ADC_BUF_LEN 		16896
 // #define ADCbit		     	12
#define SPECTROGRAM_ROWS 	NMELS
#define SPECTROGRAM_COLS 	32
#define n_label 			10


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static float32_t aSpectrogram[SPECTROGRAM_ROWS * SPECTROGRAM_COLS];
static float32_t aColBuffer[SPECTROGRAM_ROWS];
static uint32_t SpectrColIndex;
float32_t aWorkingBuffer1[NFFT];

static arm_rfft_fast_instance_f32 S_Rfft;
static MelFilterTypeDef           S_MelFilter;
static SpectrogramTypeDef         S_Spectr;
static MelSpectrogramTypeDef      S_MelSpectr;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float32_t data_out[10];
uint16_t ADC_buffer[ADC_BUF_LEN];
uint16_t ADC_buffer_sort[ADC_BUF_LEN];
uint32_t ind=0,write=0;
uint32_t i=0;
uint32_t ColIndex=0;
float32_t pBuffer[FILL_BUFFER_SIZE];
uint32_t t1 = 0,t2 ,t, wake = 0, action = 0;
float32_t sample, sample2;
float max_output;
int max_output_ind;
int confirm2;
int confirm1=0;
int confirm3=0;


//--------IR Var---------------//

uint32_t input_capture=0;
uint8_t start=0;
uint32_t array[70]={};
uint8_t array_index=0;
uint16_t time_array[69]={};
//uint16_t us;
uint8_t unstacking=0;
uint8_t send=0;
uint32_t data;
uint8_t thoat_capture=0;
uint32_t us=1000000;
uint8_t count=0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;
CRC_HandleTypeDef hcrc;
TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);


/* USER CODE BEGIN PFP */
static void PowerTodB(float32_t *pSpectrogram);
void Sort_ADC_buffer(void);
void Mel_array(void);
void Find_max_output_ind();
void Wakeup();
void buzzer_beep();
void buzzer_beep2();
void delay_us(uint16_t us);
void mark(uint16_t time);
void space(uint16_t time);
void sendRaw(uint16_t buf[], uint8_t len, uint8_t khz);
int32_t receive_data(void);

uint16_t  maychieu[67]={9070,4400,680,1570,630,1610,660,470,660,470,660,470,650,470
					,660,470,660,1590,660,1590,660,460,660,1590,690,440,660,1590,
					660,490,660,1590,630,490,640,470,680,450,650,470,660,470,660,1590,660,460,
					660,490,640,1590,660,1590,660,1590,630,1620,680,1570,650,
					470,690,1560,660,1590,660,490,640};   // chuoi bit cua remote may ON/OFF may chieu

uint16_t den[] = {9000,4460,660,500,660,500,670,500,600,500,660,500,660,
					500,670,500,660,500,660,1590,660,1580,660,1590,660,1580,660,
					1580,670,1580,660,1580,670,1580,660,670,670,500,660,1580,660,
					500,670,500,660,500,660,1590,660,500,660,500,670,1580,660,500,660,
					1590,660,1580,670,1580,660,500,660,1590,630};  // so 1 cua remote

uint16_t tivi[] = {9160,4460,630,560,610,550,610,560,600,560,610,560
					,600,560,600,570,600,560,610,1640,600,1640,610,1640,640,
					1640,600,1640,610,1640,610,1640,600,1640,610,560,
					600,1640,610,1640,610,560,600,560,610,560,600,1640
					,610,560,600,1670,580,580,580,590,580,1670,580,
					1660,580,1670,580,580,590,1660,580};   // so 2 cua remote


uint16_t dieuhoa[253]={530,1460,550,440,560,430,570,430,560,430,560,1420,570,1420,570,420,570,1420,560,1430,560,1430,560,1450,530,1450,530,1460,560,1420,
                       570,1420,560,430,570,430,560,430,560,1430,560,1420,570,1420,560,440,550,460,540,460,530,460,480,510,530,1460,560,430,560,430,570,1420,560,
               1430,560,430,560,430,570,1420,560,1430,560,430,560,440,560,450,540,450,510,490,480,510,530,460,560,1430,560,1430,560,1420,570,1420,560,460,
          //96
               560,440,560,430,560,430,570,430,560,430,560,430,570,1420,560,1430,560,1420,560,1430,560,2930,3030,8900,550,1450,510,
               490,530,460,560,430,570,430,560,430,560,430,570,430,560,430,560,1430,560,430,560,430,570,1420,560,440,560,1430,550,1450,500,1490,530,1450,
               570,1420,560,1430,560,430,560,430,570,430,560,430,560,430,570,430,560,430,560,440,560,450,540,450,540,460,480,510,510,490,530,460,560,
               430,560,430,570,430,560,430,560,430,570,430,560,430,560,430,570,430,560,430,560,440,560,430,560,460,530,460,540,450,480,520,530,
               460,530,460,560,440,560,430,560,430,570,430,560,2920,3040,8860,580,1430,560,430,560,440,560,430,560,430,560,460,540,450,540,460,500};




//-----Mel Spectrogram ----//
static void Preprocessing_Init(void);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC3_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
  Preprocessing_Init();
  SpectrColIndex = 0;
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ADC_buffer, ADC_BUF_LEN);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ind=hdma_adc3.Instance->NDTR;
	  write=ADC_BUF_LEN-ind;
	  Sort_ADC_buffer();
	  Mel_array();
	  PowerTodB(aSpectrogram);
	  ai_run(aSpectrogram, data_out);
	  Find_max_output_ind();
	  Wakeup();

    /* USER CODE END WHILE */

  //MX_X_CUBE_AI_Process();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|COI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, IR_LED_Pin|LED_USER_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED3_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin COI_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|COI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_LED_Pin LED_USER_Pin LED4_Pin */
  GPIO_InitStruct.Pin = IR_LED_Pin|LED_USER_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-----Sắp xếp lại dữ liệu----//
void Sort_ADC_buffer()
{
	for(int i=0;i<ind;i++)
		   ADC_buffer_sort[i]=ADC_buffer[i+write];
	for(int i=0;i<write;i++)
		   ADC_buffer_sort[i+ind]=ADC_buffer[i];
}

//-----------Mảng MelSpectrogram 30x32-----//
void Mel_array()
{
	for (ColIndex=0; ColIndex<SPECTROGRAM_COLS; ColIndex++)
	{
		for(int i=0; i<1024; i++)
		  	pBuffer[i]=(ADC_buffer_sort[ColIndex*HOPLENGTH+i]-2047.0)/2047.0;
		MelSpectrogramColumn(&S_MelSpectr, pBuffer, aColBuffer);
		for (uint32_t j = 0; j < NMELS; j++)
		{
		  	aSpectrogram[j * SPECTROGRAM_COLS + ColIndex] = aColBuffer[j];
		}
	}
}

//-----Tìm vị trí và giá trị lớn nhất của output---//
void Find_max_output_ind()
{
	confirm2=max_output_ind;
	max_output_ind=0;
	max_output=data_out[0];
	for (int i=1; i<n_label;i++)
	{
		if(data_out[i]>max_output)
		{
			max_output=data_out[i];
			max_output_ind=i;
		}
	}
//	if ((max_output>0.8)&(max_output_ind==1||max_output_ind==3||max_output_ind==5||max_output_ind==7))
//		{
//			confirm1 = max_output_ind*11111;
//		}
//	else
		if ((max_output>0.9)&(max_output_ind!=0))
		{
			confirm1 = max_output_ind*111111;
		}
}

void Wakeup()
{
	//----wakeup bang cam bien dien dung-----//
	if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET)
	{
		wake = 9;
		HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
	}

	//----wakeup bang keyword "tro ly" ------//
	if((wake == 0)&&(max_output_ind==9)&&(max_output>=0.98))  // nếu trước đó chưa tìm thấy key đánh thức và đột nhiên timf được
		  {
			  wake = 9;  // xét key wake lên 1
			  t1 = HAL_GetTick();  // bắt đầu đếm time 5s
			  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
//			  if (wake == 9)
//			  	  buzzer_beep();
		  }

	//------ Neu dang duoc danh thuc-------//
	if (wake == 9)
		  {
			  Find_max_output_ind();    // tim key action
			  if((max_output_ind!=9)&&(max_output_ind!=0)&&(max_output>0.85))  // tìm xem có key action được phát ra hay không
			  {
				  action = max_output_ind;  // nếu có xét keyaction lên 1 rồi thực hiện action
				  switch (action)
				  {
				  case 1:
					  if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == GPIO_PIN_SET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  	  HAL_Delay(500);
						   	   wake = 0;
						   	   break;
						  }
					  else
					  {
						  //buzzer_beep2();
						  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
					  }

				  case 2:
					  if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) == GPIO_PIN_SET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
					  {
						  //buzzer_beep2();
						  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  sendRaw(dieuhoa,253,38);
						  HAL_Delay(500);
						  wake = 0;
						  break;
					  }

				  case 3:
					  if (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == GPIO_PIN_SET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
					 	  {
						  	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
						  	  sendRaw(maychieu,67,38);
							//  buzzer_beep2();
						  	HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
							  HAL_Delay(500);
							  wake = 0;
							  break;
					 	  }

				  case 4:
					  if (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin) == GPIO_PIN_SET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
					  	{
						  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
						  //buzzer_beep2();
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
					  	}
				  case 5:
					  if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == GPIO_PIN_RESET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
						  {
						  	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
							  //buzzer_beep2();
						  	HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
							  HAL_Delay(500);
							  wake = 0;
							  break;
						  }

				  case 6:
					  if (HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin) == GPIO_PIN_RESET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);

						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
					 	 {
						  	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
							  //buzzer_beep2();
						  	HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						    sendRaw(dieuhoa,253,38);
							  HAL_Delay(500);
							  wake = 0;
							  break;
					 	 }

				  case 7:
					  if (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == GPIO_PIN_RESET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
						  {
						  	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
						  	  sendRaw(maychieu,67,38);
						  	  HAL_Delay(1000);
						  	  sendRaw(maychieu,67,38);
							  //buzzer_beep2();
							  HAL_Delay(500);
							  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
							  wake = 0;
							  break;
						  }

				  case 8:
					  if (HAL_GPIO_ReadPin(LED4_GPIO_Port, LED4_Pin) == GPIO_PIN_RESET)
						  {
						  HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  HAL_Delay(500);
						  wake = 0;
						  break;
						  }
					  else
					 	{
						  	  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
							  //buzzer_beep2();
						  	  wake = 0;
						  	HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
						  	  HAL_Delay(500);
							  break;
					 	}
				  }
			  }
			if ((HAL_GetTick()-t1 > 5000)&&(HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET))
			{
				HAL_Delay(500);
				wake = 0;
				HAL_GPIO_WritePin(LED_USER_GPIO_Port, LED_USER_Pin, GPIO_PIN_RESET);
				//buzzer_beep();
			}
			t = 0;
		  }
}


//----------Power to dB----------//
static void PowerTodB(float32_t *pSpectrogram)
{
  float32_t max_mel_energy = 0.0f;
  uint32_t i;

  /* Find MelEnergy Scaling factor */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    max_mel_energy = (max_mel_energy > pSpectrogram[i]) ? max_mel_energy : pSpectrogram[i];
  }

  /* Scale Mel Energies */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] /= max_mel_energy;
  }

  /* Convert power spectrogram to decibel */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] = 10.0f * log10f(pSpectrogram[i]);
  }

  /* Threshold output to -80.0 dB */
  for (i = 0; i < NMELS * SPECTROGRAM_COLS; i++) {
    pSpectrogram[i] = (pSpectrogram[i] < -80.0f) ? (-80.0f) : (pSpectrogram[i]);
  }
}

//------BUZZER speaker beep-------//
void buzzer_beep()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}


//-------------BUZZer beep beep-------------//
void buzzer_beep2()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}
//----------------------------IIRR--------------------//
//-----------delay us---------//
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2,0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

//------------Mark Function--------/
void mark(uint16_t time)
{
  //uint16_t count=(uint16_t)time/(us/38000);
  //uint16_t count=(uint16_t)((uint32_t)(time*38000))/us;
  uint16_t idx=0;
  uint32_t a=time*38000;
  uint32_t count=a/us;
  uint32_t b=us/76000;
  while(idx < count)
  {
    //led on
    HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin,GPIO_PIN_SET );
    delay_us(b);
    HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin,GPIO_PIN_RESET );
    delay_us(b);
    idx=idx+1;
  }
}


//--------------Space function------------//
void space(uint16_t time)
{
  HAL_GPIO_WritePin(IR_LED_GPIO_Port, IR_LED_Pin,GPIO_PIN_RESET );
    if(time==0) return;
    delay_us(time);
}

//--------Send data------//
void sendRaw(uint16_t buf[], uint8_t len, uint8_t khz)
{
  //enableIROut(khz);

  for(uint16_t i = 0; i < len; i++)
  {
    if(i & 1) space(buf[i]);
    else mark(buf[i]);
  }

  space(0);
}

//---------------Giai ma IR ----------------------//
int32_t receive_data(void)
{
  uint32_t code=0;
  while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2)));  // wait for the pin to go high.. 9ms LOW
  while (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2));

  for (int i=0; i<32; i++)
  {
    count=0;
    while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2))); // wait for pin to go high.. this is 562.5us LOW

    while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2)))  // count the space length while the pin is high
   {
    count++;
    delay_us(7200);
   }
   if (count > 12) // if the space is more than 1.2 ms
   {
    code |= (1UL << (31-i));   // write 1
   }

   else code &= ~(1UL << (31-i));  // write 0
   array[i]=count;
  }
  return code;
}

//------Preprocessing Init-------//
static void Preprocessing_Init(void)
{
  /* Init RFFT */
  arm_rfft_fast_init_f32(&S_Rfft, 1024);

  /* Init Spectrogram */
  S_Spectr.pRfft    = &S_Rfft;
  S_Spectr.Type     = SPECTRUM_TYPE_POWER;
  S_Spectr.pWindow  = (float32_t *) hannWin_1024;
  S_Spectr.SampRate = 16000;
  S_Spectr.FrameLen = 1024;
  S_Spectr.FFTLen   = 1024;
  S_Spectr.pScratch = aWorkingBuffer1;

  /* Init Mel filter */
  S_MelFilter.pStartIndices = (uint32_t *) melFiltersStartIndices_1024_30;
  S_MelFilter.pStopIndices  = (uint32_t *) melFiltersStopIndices_1024_30;
  S_MelFilter.pCoefficients = (float32_t *) melFilterLut_1024_30;
  S_MelFilter.NumMels       = 30;

  /* Init MelSpectrogram */
  S_MelSpectr.SpectrogramConf = &S_Spectr;
  S_MelSpectr.MelFilter       = &S_MelFilter;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
