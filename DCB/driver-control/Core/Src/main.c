/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dash Control Board Firmware
  ******************************************************************************
  * @attention
  * 
  * This is the code for the dash board. This reads all of the sensor data, reads and 
  * sends CAN messages, and drives the LCD screen on the dashboard.
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// inputs
#define ADC_BUF_LEN 4				      // length of dma adc buffer
#define MAX_PEDAL_SKEW 36         // this is the square of the difference allowed
#define MAX_COMMAND_TORQUE 100    // Nm of torque allowed. Pedal scales this value
#define ADC_BIT_SIZE 1024         // 10 bit sample, so it's 1024 values
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

// pedal conversion ratio
float torque_conversion_ratio = (float)MAX_COMMAND_TORQUE / ADC_BIT_SIZE;

// CAN
CAN_RxHeaderTypeDef rxHeader; 					      // CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader1; 					      // CAN Bus Transmit Header Torque Setting
CAN_TxHeaderTypeDef txHeader2; 					      // CAN Bus Transmit Header DAQ Data
CAN_TxHeaderTypeDef txHeader3; 					      // CAN Bus Transmit Header Control Data
uint8_t rxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Bus Receive Buffer
uint8_t txData[8];
CAN_FilterTypeDef filter_rcb; 					        // CAN Bus Filter
uint32_t txMailbox; 							            // CAN Bus Mail box variable

// rinehart & emus  
float rinehartVoltage = 0;				  		      // voltage in rinehart
float emusVoltage = 0;				  			        // emus bus voltage
int readyToDrive = 0;							            // ready to drive (0 = no, 1 = yes)

// inputs 
float coastRegen, brakeRegen;			    	      // coast and brake regen values 
float pedal0, pedal1;                 			  // pedal values
float pedal_average;
float brake0, brake1;                		  	  // brake values
float brake_average;
uint8_t coastMap, brakeMap;						        // maps for coast and brake regen
float wheelSpeedFR = 0;               			  // read from sensor input
float wheelSpeedFL = 0;               			  // read from sensor input
float wheelSpeedBR = 0;               			  // this needs to be retrieved from CAN
float wheelSpeedBL = 0;               			  // this needs to be retrieved from CAN
float rideHeightFR = 0;               			  // read from sensor input
float rideHeightFL = 0;               			  // read from sensor input
float rideHeightBR = 0;               			  // this needs to be retrieved from CAN
float rideHeightBL = 0;               			  // this needs to be retrieved from CAN
uint8_t startButton = 0;             				  // start button state (0 is not active)
uint32_t adc_buf[ADC_BUF_LEN];                // adc buffer for dma
uint8_t faultAMS = 0;                         // updated from RCB CAN
uint8_t faultIMD = 0;                         // updated from RCB CAN
uint16_t commandedTorque = 0;                  // amount of torque we're requesting from Rinehart

// outputs
uint8_t startButtonState = 0;              			  // RTD button LED toggle (0 is off)
uint8_t coolingState = 0;                     		// cooling toggle (0 is off)
uint8_t direction = 0;		                		    // drive direction (0 is forwards)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// compares accelerator pedal, 0 if above skew value
uint8_t accel_pedal_compare(uint8_t pedal0, uint8_t pedal1); 

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

	// init the CAN mailbox for Rinehart Command Torque Parameters
	txHeader1.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader1.IDE = CAN_ID_STD;
	txHeader1.RTR = CAN_RTR_DATA;
	txHeader1.StdId = 0xC0;
	txHeader1.ExtId = 0;
	txHeader1.TransmitGlobalTime = DISABLE;

	// init the CAN mailbox for DAQ Data
	txHeader2.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader2.IDE = CAN_ID_STD;
	txHeader2.RTR = CAN_RTR_DATA;
	txHeader2.StdId = 0x92;
	txHeader2.ExtId = 0;
	txHeader2.TransmitGlobalTime = DISABLE;

	// init the CAN mailbox for Control Data
	txHeader3.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader3.IDE = CAN_ID_STD;
	txHeader3.RTR = CAN_RTR_DATA;
	txHeader3.StdId = 0x93;
	txHeader3.ExtId = 0;
	txHeader3.TransmitGlobalTime = DISABLE;

	HAL_CAN_ConfigFilter(&hcan1, &filter_rcb); // Initialize CAN Filter
	HAL_CAN_Start(&hcan1); // Initialize CAN Bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt

	// start the dma adc conversion
	HAL_ADC_Start_DMA(&hadc1, adc_buf, ADC_BUF_LEN);
  HAL_TIM_Base_Start_IT(&htim13); // start the timer interupt
  HAL_TIM_Base_Start_IT(&htim14); // start the timer interupt


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// all of the main loop code is in the defaultTask function as the infinite loop is in there
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
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

    // init the CAN filter
	filter_rcb.FilterBank = 0;
	filter_rcb.FilterMode = CAN_FILTERMODE_IDMASK;
	filter_rcb.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter_rcb.FilterIdHigh = 0x0FF << 5;
	filter_rcb.FilterIdLow = 0x082 << 5;
	filter_rcb.FilterMaskIdHigh = 0x082 << 5;
	filter_rcb.FilterMaskIdLow = 0xFFF << 5;
	filter_rcb.FilterScale = CAN_FILTERSCALE_32BIT;
	filter_rcb.FilterActivation = ENABLE;
	filter_rcb.SlaveStartFilterBank = 0;
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 9000-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 500-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB10 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// *** functions *** //

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
    Error_Handler();

  // get sensor data from rcb
  if (rxHeader.StdId == 0x81)
  {
	  wheelSpeedBL = rxData[0];
	  wheelSpeedBR = rxData[1];
	  rideHeightBL = rxData[2];
	  rideHeightBR = rxData[3];
  }

  // get ready to drive from high voltage for precharge complete
  if (rxHeader.StdId == 0x87){
	  readyToDrive = rxData[0]; // 0 is NO, 1 is YES
  }

  // data from RCB for fault lights
  if (rxHeader.StdId == 0x82){
    faultAMS = rxData[1];
    faultIMD = rxData[0];
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

// 10Hz timer
// update indicator lights and  send switch values to RCB
if (htim == &htim13){

  // read switches
  coolingState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
  direction = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);


  // write fault lights
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, faultAMS);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, faultIMD);
  
  txData[0] = 0; // regen pot
  txData[1] = 0; // brake pot
  txData[2] = coolingState << 1; // cooling
  txData[3] = direction; // direction (1 is OFF, pulled up)
  txData[4] = 0; // brake light


  HAL_CAN_AddTxMessage(&hcan1, &txHeader3, txData, &txMailbox);
}

// 20Hz timer
// send Rinehart Parameter Command Torque Things
if (htim == &htim14){

  // call functions to average pedal and brake
  // brake sampling
  brake_average = (brake0 + brake1) / 2;

  pedal_average = accel_pedal_compare(pedal0, pedal1);
  commandedTorque = (int)(pedal_average * torque_conversion_ratio); // commanded torque


  // define variables
  txData[0] = commandedTorque >> 8;     // MSB, 2 byte
  txData[1] = commandedTorque && 0xFF;  // LSB, 2 byte
  txData[2] = 0;
  txData[3] = 0;
  txData[4] = 0;
  txData[5] = 0;
  txData[6] = 0;
  txData[7] = 0;


  HAL_CAN_AddTxMessage(&hcan1, &txHeader1, txData, &txMailbox);

}

}

// interrupt for the DMA
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

  brake0 = adc_buf[0];
  brake1 = adc_buf[1];
  pedal0 = adc_buf[2];
  pedal1 = adc_buf[3];

}

uint8_t accel_pedal_compare(uint8_t pedal0, uint8_t pedal1){

  uint8_t pedal_average = (pedal0 + pedal1) / 2;

    if (pow(pedal0 - pedal_average, 2) > MAX_PEDAL_SKEW | 
        pow(pedal1 - pedal_average, 2) > MAX_PEDAL_SKEW ){
        pedal_average = 0;
    }


    return pedal_average;

}

// /**
//  * @brief welcome & boot screen
//  * 
//  */
// void welcomeScreen()
// {
// 	lcdInit(&hi2c1, 0x27, 2, 16);       			// init lcd (i2c reference, LCD address, lines, rows)
// 	lcdAutoscrollOff();								// turn off autoscroll
// 	lcdBacklightOn();								// turn on backlight
// 	lcdDisplayClear();                  			// clear the screen
// 	lcdSetCursorPosition(2, 0);         			// set the cursor
// 	lcdPrintStr((uint8_t*)"welcome AERO!", 13);   	// print
// 	lcdSetCursorPosition(2, 1);         			// next line
// 	lcdPrintStr((uint8_t*)"booting up...", 13);   	// print
// 	HAL_Delay(3000);								// delay 3 seconds so the screen can be read
// 	lcdDisplayClear();								// clear the display so the other screens can be printed
// }


// /**
//  * @brief racing hud: mph(est), battery%, drive direction, coast regen, brake regen
//  *
//  */
// void racingHUD()
// {
// 	// get wheel speed
// 	float averageWheelSpeed = (wheelSpeedFR + wheelSpeedFL) / 2;

// 	// get current mph from wheel speed
// 	float currentMPH = ((averageWheelSpeed * WHEEL_DIAMETER) * (3.14159 * 60)) / 63360;

// 	// get battery percentage
// 	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

// 	// init some char buffs for variables
// 	char battStr[10];
// 	char speedStr[10];
// 	char coastStr[10];
// 	char brakeStr[10];

// 	// drive direction
// 	lcdSetCursorPosition(0, 0);									// position of drive direction
// 	if (direction) lcdPrintStr((uint8_t*)"FWD", 3);     		// print drive direction
// 	else lcdPrintStr((uint8_t*)"RVS", 3);

// 	// battery percentage
// 	lcdSetCursorPosition(12, 0); 								// set cursor for battery percentage value
// 	sprintf(battStr, "%.0d%%", (int)batteryPercentage); 		// sprintf it
// 	lcdPrintStr((uint8_t*)battStr, strlen(battStr));			// print the battery percentage value

// 	// speedometer		
// 	lcdSetCursorPosition(7, 0);                     			// set cursor for mph value
// 	sprintf(speedStr, "%.0d", (int)currentMPH);					// sprintf it
// 	lcdPrintStr((uint8_t*)speedStr, strlen(speedStr));			// print the current speed in MPH, cast to int to round to whole number
// 	lcdSetCursorPosition(7, 1);                     			// set cursor for units
// 	lcdPrintStr((uint8_t*)"mph", 3);                    		// print units

// 	// coast regen		
// 	lcdSetCursorPosition(0, 1);                      			// set cursor for CR
// 	sprintf(coastStr, "C:%.0d%%", (int)coastRegen);				// sprintf it
// 	lcdPrintStr((uint8_t*)coastStr, strlen(coastStr));  		// print coast regen value

// 	// brake regen		
// 	lcdSetCursorPosition(11, 1);                     			// set cursor for BR
// 	sprintf(brakeStr, "B: %d%%", (int)brakeRegen);				// sprintf it
// 	lcdPrintStr((uint8_t*)brakeStr, strlen(brakeStr));  		// print brake regen value
// }


// /**
//  * @brief battery state, bus voltage, rinehart voltage, power mode
//  * 
//  */
// void electricalSettings()
// {
// 	// get battery percentage
// 	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

// 	// init some char buffs for variables
// 	char battStr[10];
// 	char busVStr[10];

// 	// battery percentage
// 	lcdSetCursorPosition(0, 0);									// set cursor for battery title
// 	sprintf(battStr, "Batt:%d%%", (int)batteryPercentage);		// sprintf it
// 	lcdPrintStr((uint8_t*)battStr, strlen(battStr));			// print title

// 	// bus voltage
// 	lcdSetCursorPosition(11, 0);								// set cursor for bus voltage title
// 	sprintf(busVStr, "Bus:%d", (int)emusVoltage);				// sprintf it			
// 	lcdPrintStr((uint8_t*)busVStr, strlen(busVStr));			// print
// 	lcdSetCursorPosition(15, 1);                                // set cursor for units
// 	lcdPrintStr((uint8_t*)"V", 1);                              // print units

// 	/*	not planning on using this for the time being
// 	// rinehart voltage
// 	lcdSetCursorPosition(12, 0);                                // set cursor for rinehart voltage value
// 	lcdPrintStr(rinehartVoltage);                           	// print the rinehart voltage value
// 	lcdSetCursorPosition(15, 0);                                // set cursor for units
// 	lcdPrintStr("V");                                       	// print % sign
// 	*/

// 	// power mode
// 	lcdSetCursorPosition(0, 1);                                 // set cursor for mode text
// 	lcdPrintStr((uint8_t*)"Mode:", 5);							// print mode text
// 	lcdSetCursorPosition(5, 1);                                 // set cursor current mode setting
// 	if (powerMode == TUTORIAL) lcdPrintStr((uint8_t*)"TUTR", 4);
// 	if (powerMode == ECO) lcdPrintStr((uint8_t*)"ECO", 3);
// 	if (powerMode == EXPERT) lcdPrintStr((uint8_t*)"EXPT", 4);
// 	else lcdPrintStr((uint8_t*)(uint8_t*)"ERR!", 4);
// }


// /**
//  * @brief ride height, wheel rpm, coast regen, brake regen
//  *
//  */
// void rideSettings()
// {
// 	// init some char buffs for variables
// 	char rideStr[10];
// 	char wheelStr[10];

// 	// ride height
// 	lcdSetCursorPosition(0, 0);									// set cursor for front left ride height value
// 	sprintf(rideStr, "%d", (int)rideHeightFL);					// sprintf it
// 	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print front left ride height value

// 	lcdSetCursorPosition(2, 0);				  					// spacer
// 	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

// 	lcdSetCursorPosition(3, 0);									// set cursor for front right ride height value
// 	sprintf(rideStr, "%d", (int)rideHeightFR);					// sprintf it
// 	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print front right ride height value

// 	lcdSetCursorPosition(5, 0);									// set cursor for "<- Ride"
// 	lcdPrintStr((uint8_t*)"<-Ride", 6);							// print

// 	lcdSetCursorPosition(0, 1);                  				// set cursor for back left ride height value
// 	sprintf(rideStr, "%d", (int)rideHeightBL);					// sprintf it
// 	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));         	// print back left ride height value

// 	lcdSetCursorPosition(2, 1);				  					// spacer
// 	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

// 	lcdSetCursorPosition(3, 1);                  				// set cursor for back right ride height value
// 	sprintf(rideStr, "%d", (int)rideHeightBR);					// sprintf it
// 	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print back right ride height value

// 	lcdSetCursorPosition(6, 1);                  				// set cursor for "RPM->"
// 	lcdPrintStr((uint8_t*)"RPM->", 5);                   		// print the "RPM->"

// 	// wheel speed
// 	lcdSetCursorPosition(11, 0);								// set cursor for front left wheelspeed value
// 	sprintf(wheelStr, "%d", (int)wheelSpeedFL);					// sprintf it
// 	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print front left wheelspeed value

// 	lcdSetCursorPosition(13, 0);								// spacer
// 	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

// 	lcdSetCursorPosition(14, 0);                 				// set cursor for front right wheelspeed value
// 	sprintf(wheelStr, "%d", (int)wheelSpeedFR);					// sprintf it
// 	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print front right wheelspeed value

// 	lcdSetCursorPosition(11, 1);                 				// set cursor for back left wheelspeed value
// 	sprintf(wheelStr, "%d", (int)wheelSpeedBL);					// sprintf it
// 	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print back left wheelspeed value

// 	lcdSetCursorPosition(13, 1);								// set cursor for "-"
// 	lcdPrintStr((uint8_t*)"-", 1);								// print the "-"

// 	lcdSetCursorPosition(14, 1);								// set cursor for back right wheelspeed value
// 	sprintf(wheelStr, "%d", (int)wheelSpeedBR);					// sprintf it
// 	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print value for back right wheelspeed value
// }

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

