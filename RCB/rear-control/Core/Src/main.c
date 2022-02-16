/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This is the code for the rear control board. This reads all of the sensor data, 
  * sends and reads CAN messages, and also manages faults from around the car.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// inputs
#define PIN_WATER_TEMP_IN                 17		// water temperature sensor in
#define PIN_WATER_TEMP_OUT                29		// water temperature sensor out
#define PIN_REAR_RIGHT_WHEEL              11		// rear right wheel speed sensor
#define PIN_REAR_LEFT_WHEEL				        10		// rear left wheel speed sensor
#define PIN_REAR_RIGHT_SUSPENSION		      13		// rear right suspension sensor
#define PIN_REAR_LEFT_SUSPENSION		      12		// rear left suspension sensor
#define PIN_IMD_FAULT					            18		// IMD fault line
#define PIN_BMS_FAULT					            19		// BMS fault line

// outputs
#define PIN_BRAKE_LIGHT					          14		// brake light
#define PIN_FAN_CONTROL					          15		// fan speed/on/off control
#define PIN_PUMP_CONTROL				          16		// pump speed/on/off control

// CAN
#define PIN_CAN_PLUS					            31		// positive CAN wire
#define PIN_CAN_MINUS					            33		// negative CAN wire
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

// CAN
CAN_RxHeaderTypeDef rxHeader; 					        // CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader0; 					        // CAN Bus Transmit Header BASE
CAN_TxHeaderTypeDef txHeader1; 					        // CAN Bus Transmit Header Torque Setting
CAN_TxHeaderTypeDef txHeader2; 					        // CAN Bus Transmit Header DAQ Data
CAN_TxHeaderTypeDef txHeader3; 					        // CAN Bus Transmit Header Control Data
uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 	  // CAN Bus Receive Buffer
CAN_FilterTypeDef canFilter; 					          // CAN Bus Filter
uint32_t canMailbox; 							              // CAN Bus Mail box variable

// inputs
float waterTempIn = 0;
float waterTempOut = 0;
float wheelSpeedBR = 0;
float wheelSpeedBL = 0;
float rideHeightBR = 0;
float rideHeightBL = 0;
int IMDFaultState = 0;                          // 0 is maybe not faulting 
int BMSFaultState = 0;                          // 0 is maybe not faulting

// outputs
int brakeLightState = 0;                        // 0 is off maybe
int pumpState = 1;                              // 1 is on maybe
int fanState = 1;                               // 1 is on maybe
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void pollSensorData();
void ADC_Select_CH_WSBR();          // wheel speed
void ADC_Select_CH_WSBL();          // wheel speed
void ADC_Select_CH_RHBR();          // ride height
void ADC_Select_CH_RHBL();          // ride height
void ADC_Select_CH_WTIN();          // water temperature in
void ADC_Select_CH_WTOUT();         // water temperature out
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

  // init the CAN filter
	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterIdHigh = 0;
	canFilter.FilterIdLow = 0;
	canFilter.FilterMaskIdHigh = 0;
	canFilter.FilterMaskIdLow = 0;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterActivation = ENABLE;
	canFilter.SlaveStartFilterBank = 14;

	// init the CAN mailbox for BASE
	txHeader0.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader0.IDE = CAN_ID_STD;
	txHeader0.RTR = CAN_RTR_DATA;
	txHeader0.StdId = 0x80;
	txHeader0.ExtId = 0x02;
	txHeader0.TransmitGlobalTime = DISABLE;

	// init the CAN mailbox for DAQ DATA
	txHeader1.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader1.IDE = CAN_ID_STD;
	txHeader1.RTR = CAN_RTR_DATA;
	txHeader1.StdId = 0x81;
	txHeader1.ExtId = 0x03;
	txHeader1.TransmitGlobalTime = DISABLE;

	// init the CAN mailbox for CONTROL DATA
	txHeader2.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader2.IDE = CAN_ID_STD;
	txHeader2.RTR = CAN_RTR_DATA;
	txHeader2.StdId = 0x82;
	txHeader2.ExtId = 0x04;
	txHeader2.TransmitGlobalTime = DISABLE;

	HAL_CAN_ConfigFilter(&hcan1, &canFilter); // Initialize CAN Filter
	HAL_CAN_Start(&hcan1); // Initialize CAN Bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt
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
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // poll sensor data
    pollSensorData();

    // read CAN messages


    // send can messages
    // BASE
		uint8_t csend0[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
		HAL_CAN_AddTxMessage(&hcan1, &txHeader0, csend0, &canMailbox); // Send Message

    // DAQ DATA
		uint8_t csend1[] = {wheelSpeedBL, wheelSpeedBR, rideHeightBL, rideHeightBR, waterTempIn, waterTempOut, 0x06, 0x07};
		HAL_CAN_AddTxMessage(&hcan1, &txHeader1, csend1, &canMailbox); // Send Message

    // CONTROL SETTINGS
		uint8_t csend2[] = {brakeLightState, pumpState, fanState, 0x03, 0x04, 0x05, 0x06, 0x07};
		HAL_CAN_AddTxMessage(&hcan1, &txHeader2, csend2, &canMailbox); // Send Message
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    Error_Handler();
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

  // ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();
  /* Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  */
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
    Error_Handler();
    
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// *** functions *** //

void pollSensorData()
{
  // get back right wheel speed
	ADC_Select_CH_WSBR();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	wheelSpeedBR = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// get back left wheel speed
	ADC_Select_CH_WSBL();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	wheelSpeedBL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// get back right ride height
	ADC_Select_CH_RHBR();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	rideHeightBR = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// get back left ride height
	ADC_Select_CH_RHBL();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	rideHeightBL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// get water temp in 
	ADC_Select_CH_WTIN();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	waterTempIn = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	// get water temp out
	ADC_Select_CH_WTOUT();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	waterTempOut = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}

void ADC_Select_CH_WSBL()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

void ADC_Select_CH_WSBR()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

void ADC_Select_CH_RHBL()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

void ADC_Select_CH_RHBR()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

void ADC_Select_CH_WTIN()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

void ADC_Select_CH_WTOUT()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
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

