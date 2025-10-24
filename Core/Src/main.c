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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "NRF24L01P.h"
#include "DHT11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 		ADD 		0xD0
#define     RTD     57.2957
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int16_t ax = 0, ay = 0 , az = 0, gx = 0 ,gy = 0 , gz = 0;
float AX,AY,AZ,GX,GY,GZ;
float pitch = 0;
float roll = 0;
/*
--------2x18650--------
float Kp = 300;          // (P)roportional Tuning Parameter
float Ki = 2000;          // (I)ntegral Tuning Parameter        
float Kd =5;          // (D)erivative Tuning Parameter
-----------------------
*/

float Kp_a =300;          // (P)roportional Tuning Parameter
float Ki_a = 0;          // (I)ntegral Tuning Parameter        
float Kd_a = 0.4;          // (D)erivative Tuning Parameter  

float Kp_p =0.04;//0.05;          // (P)roportional Tuning Parameter
float Ki_p = 0;          // (I)ntegral Tuning Parameter        
float Kd_p = 0.0001;//0.001;          // (D)erivative Tuning Parameter
float iTerm_p = 0;       // Used to accumulate error (integral)
float lastTime_p = 0;    // Records the time the function was last called
float maxPID_p = 30;    // The maximum value that can be output
float oldValue_p = 0;    // The last sensor value
float targetValue_p = 0;
float pwmDead_p = 0;
//---------------------------------------
float iTerm_a = 0;       // Used to accumulate error (integral)
float lastTime_a = 0;    // Records the time the function was last called
float maxPID_a = 999;    // The maximum value that can be output
float oldValue_a = 0;    // The last sensor value
float targetValue_a = 0;
float pwmDead_a = 0;

short encoder = 0;

int count_dht = 0;
int count_dht_current = 0;
int count_dht_last = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU6050Init(void);
void MPU6050ReadG(void);
void MPU6050ReadA(void);
void filter(float AX, float AY, float AZ, float GX, float GY, float GZ,uint16_t dt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t MPU6050Init(void){
	uint8_t check;
	uint8_t mData;
	HAL_I2C_Mem_Read(&hi2c1,ADD, 0x75,1, &check,1,1000);
	if(check == 0x68){
		mData = 0x01;
		HAL_I2C_Mem_Write(&hi2c1, ADD,0x6B,1,&mData,1,1000);
		mData = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, ADD,0x19,1,&mData,1,1000);
		mData = 0x00; 
		HAL_I2C_Mem_Write(&hi2c1, ADD,0x1B,1,&mData,1,1000);
		mData = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, ADD,0x1C,1,&mData,1,1000);
		return 0;
	}
	return 1;
}
void MPU6050ReadG(void){
	uint8_t dataG[6];
	
	HAL_I2C_Mem_Read(&hi2c1,ADD, 0x43,1, dataG,6,1000);
	gx = (int16_t)(dataG[0] << 8 | dataG[1]);
	gy = (int16_t)(dataG[2] << 8 | dataG[3]);
	gz = (int16_t)(dataG[4] << 8 | dataG[5]);
	GX = (float)gx/131.0;
	GY = (float)gy/131.0;
	GZ = (float)gz/131.0;
}
void MPU6050ReadA(void){
	uint8_t dataA[6];
	HAL_I2C_Mem_Read(&hi2c1,ADD, 0x3B,1, dataA,6,1000);
	ax = (int16_t)(dataA[0] << 8 | dataA[1]);
	ay = (int16_t)(dataA[2] << 8 | dataA[3]);
	az = (int16_t)(dataA[4] << 8 | dataA[5]);
	AX = (float)ax/16384.0;
	AY = (float)ay/16384.0;
	AZ = (float)az/16384.0;
}
void filter(float AX, float AY, float AZ, float GX, float GY, float GZ,uint16_t dt){
	
	float pitchG = pitch + GX*(dt/64000.0f);
	float rollG = roll + GY*(dt/64000.0f);
	
	float pitchA = atan2(AY, sqrt(AX*AX + AZ * AZ))*RTD;
	float rollA = atan2(AX, sqrt(AY*AY + AZ*AZ))*RTD;
	
	pitch = 0.99*pitchG + 0.01*pitchA;
	roll = 0.99*rollG + 0.01*rollA;

}
/*
IN1: PA0
IN2: PA1
IN3: PA2
IN4: PA3
roll < 0 -> Up
roll > 0 -> Down
*/
void PWM_Start(void){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
void Update_PWM(int16_t pwm){
	if(pwm > 0){
		htim2.Instance->CCR1 = pwm;
		htim2.Instance->CCR2 = 0;
		htim2.Instance->CCR3 = pwm;
		htim2.Instance->CCR4 = 0;
		return;
	}
	htim2.Instance->CCR1 = 0;
	htim2.Instance->CCR2 = -pwm;
	htim2.Instance->CCR3 = 0;
	htim2.Instance->CCR4 = -pwm;
}
int16_t pid_angle(float target, float current,int64_t dt) {
	// Calculate the time since function was last called
	float dT = (float)dt/64000.0f;

	// Calculate error between target and current values
	float error = target - current;

	// Calculate the integral term
	iTerm_a += error * dT;
	if(iTerm_a > 0.7) iTerm_a = 0.7;
	if(iTerm_a < -0.7) iTerm_a = -0.7;
	// Calculate the derivative term (using the simplification)
	float dTerm = (oldValue_a - current) / dT;

	// Set old variable to equal new ones
	oldValue_a = current;

	// Multiply each term by its constant, and add it all up
	float result = (error * Kp_a) + (iTerm_a * Ki_a) + (dTerm * Kd_a);

	// Limit PID value to maximum values
	if (result > maxPID_a) result = maxPID_a;
	else if (result < -maxPID_a) result = -maxPID_a;

	return (int16_t)result;
}
int16_t pid_position(float target, float current,int64_t dt) {
	// Calculate the time since function was last called
	float dT = (float)dt/64000.0f;

	// Calculate error between target and current values
	float error = target - current;

	// Calculate the integral term
	iTerm_p += error * dT;
	if(iTerm_p > 1) iTerm_p = 1;
	if(iTerm_p < -1) iTerm_p = -1;
	// Calculate the derivative term (using the simplification)
	float dTerm = (oldValue_p - current) / dT;

	// Set old variable to equal new ones
	oldValue_p = current;

	// Multiply each term by its constant, and add it all up
	float result = -((error * Kp_p) + (iTerm_p * Ki_p) + (dTerm * Kd_p));

	// Limit PID value to maximum values
	if (result > maxPID_p) result = maxPID_p;
	else if (result < -maxPID_p) result = -maxPID_p;

	return (int16_t)result;
}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MPU6050Init();
	PWM_Start();
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
	HAL_Delay(500);
	DHT11_Read();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
	uint16_t x = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
uint32_t lastLoop = 0;
const uint32_t loopInterval = 10;

uint32_t lastDHT = 0;
const uint32_t DHTInterval = 4000; 
	while (1){
		uint32_t now = HAL_GetTick();
		if (now - lastLoop >= loopInterval){
			lastLoop = now;

			encoder = __HAL_TIM_GET_COUNTER(&htim3);
			x = htim1.Instance->CNT;
			htim1.Instance->CNT = 0;

			MPU6050ReadG();
			MPU6050ReadA();
			filter(AX, AY, AZ, GX, GY, GZ, x);

			targetValue_a = pid_position(targetValue_p, encoder, x) + 4.4;
			int16_t pwmvalue = pid_angle(targetValue_a, pitch, x);
			Update_PWM(pwmvalue);

			if (pitch > 60 || pitch < -60){
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				while (1);
			}
		}
		if (now - lastDHT >= DHTInterval){
			lastDHT = now;
			DHT11_Read();
		}		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 1124;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 35;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
