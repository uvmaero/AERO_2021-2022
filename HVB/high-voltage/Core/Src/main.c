/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// analog buffer
#define ADC_BUF_LEN 4086

// precharge
#define PRECHARGE_COEFFICIENT       0.95          // 95% complete with precharge so it's probably safe to continue
#define NUM_COMMAND_MSG             10            // number of command messages we see from rinehart
#define NUM_VOLTAGE_CHECKS          500           // since we're checking at 10ms Interrupts, 500 would be 5 seconds. 
                                                  // precharge should be done in less than 2 seconds. 
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

// counters for Rinehart message sending in precharge
uint8_t rinehart_send_command_count = 0;

// inputs
uint32_t rinehartVoltage = 0;                 // read from CAN
uint32_t bmsVoltage = 0;					            // read from CAN
int vicoreTemp = 0;					                  // read from DMA, vicore temp
int DCDCEnable = 0;                           // dc-dc enable (0 = disabled, 1 = enabled)
int RTDButtonPressed = 0;                     // read this from CAN, if it's 1 we can finish precharge
uint32_t adc_buf[ADC_BUF_LEN];				        // adc read buffer

// output
int DCDCFault = 0;                            // the dc-dc fault indicator (0 = no fault, 1 = fault)
int readyToDrive = 0;					                // the car is ready to drive! (0 = not ready, 1 = ready)

// precharge states
enum prechargeStates
{
	PRECHARGE_OFF,
	PRECHARGE_ON,
	PRECHARGE_DONE,
	PRECHARGE_ERROR
};
uint8_t prechargeState = PRECHARGE_OFF;			  // set initial precharge state to OFF
uint8_t lastPrechargeState = PRECHARGE_OFF;
uint8_t voltageCheckCount = 0;

// CAN transmit 
uint32_t TxMailbox; 							            // CAN Bus Mail box variable
CAN_TxHeaderTypeDef txHeader0; 					      // CAN Bus Transmit Header BASE
CAN_TxHeaderTypeDef txHeader1; 					      // CAN Bus Transmit Header DATA
CAN_TxHeaderTypeDef txHeader2; 					      // CAN Bus Transmit Header DATA
uint8_t TxData[8];                            // CAN transmit buffer

// CAN recive
CAN_RxHeaderTypeDef rxHeader; 					      // CAN Bus Receive Header
uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 	// CAN Bus Receive Buffer

CAN_FilterTypeDef canFilter0; 					      // CAN Bus Filter for BMS
CAN_FilterTypeDef canFilter1;                 // CAN Bus Filter for Rinehart
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
void prechargeControl();

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

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  // init the CAN mailbox for BASE
  txHeader0.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader0.IDE = CAN_ID_STD;
  txHeader0.RTR = CAN_RTR_DATA;
  txHeader0.StdId = 0x086;
  txHeader0.ExtId = 0;
  txHeader0.TransmitGlobalTime = DISABLE;

  // init the CAN mailbox for DATA
  txHeader1.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader1.IDE = CAN_ID_STD;
  txHeader1.RTR = CAN_RTR_DATA;
  txHeader1.StdId = 0x087;
  txHeader1.ExtId = 0;
  txHeader1.TransmitGlobalTime = DISABLE;

  // header for rinehart (Parameter Command Message)
  txHeader2.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader2.IDE = CAN_ID_STD;
  txHeader2.RTR = CAN_RTR_DATA;
  txHeader2.StdId = 0x0C1;
  txHeader2.ExtId = 0;
  txHeader2.TransmitGlobalTime = DISABLE;

	HAL_CAN_Start(&hcan1); // Initialize CAN Bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt

  // start timers
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim13);

	// start ADC DMA
	HAL_ADC_Start_DMA(&hadc1, adc_buf, ADC_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  // init the CAN filter for BMS messages
    canFilter0.FilterIdHigh = 0x6B0 << 5;   // Orion ID: 0x6B0
  	canFilter0.FilterIdLow = 0x000;
    canFilter0.FilterMaskIdHigh = 0x6B0 << 5;
  	canFilter0.FilterMaskIdLow = 0x000;
    canFilter0.FilterBank = 0;
  	canFilter0.FilterMode = CAN_FILTERMODE_IDMASK;
  	canFilter0.FilterFIFOAssignment = CAN_RX_FIFO0;
  	canFilter0.FilterScale = CAN_FILTERSCALE_32BIT;
  	canFilter0.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilter0);

    // init the CAN filter for Rinehart messages
    canFilter1.FilterIdHigh = 0x0A7 << 5;      // Rinehart IDs: 0xA0 - 0xB1
  	canFilter1.FilterIdLow = 0x000;
    canFilter1.FilterMaskIdHigh = 0x0A7 << 5;
  	canFilter1.FilterMaskIdLow = 0x000;
    canFilter1.FilterBank = 1;
  	canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
  	canFilter1.FilterFIFOAssignment = CAN_RX_FIFO0;
  	canFilter1.FilterScale = CAN_FILTERSCALE_32BIT;
  	canFilter1.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilter1);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// *** functions *** //


/**
 * overwriting the weak function defined in the includes which is the ISR for the CAN interrupt
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    Error_Handler();

  // get rinehart bus voltage
  if (rxHeader.StdId == 0x0A7 && rxHeader.DLC == 8)    // sometimes rinehart sends 0 length messages so only read when there's data
  {
    // rinehart voltage is spread across the first 2 bytes
	  int rine1 = canRX[0];
    int rine2 = canRX[1];

    // combine the first two bytes and assign that to the rinehart voltage
    rinehartVoltage = (rine2 << 8) | rine1;
  }

  // get BMS total voltages
  if (rxHeader.StdId == 0x6B0)
  {
    // BMS voltage is spread across the first 2 bytes
	  int volt1 = canRX[2];
    int volt2 = canRX[3];

    // combine the first two bytes and assign that to the BMS voltage
    bmsVoltage = (volt1 << 8) | volt2; // orion has a pre-scaller of *10
  }
}


/**
 * @brief 
 * 
 */
void prechargeControl()
{
  // NOTE: Weird thing about Rinehart 0x0A7 message: 
  //      Sometimes it sends a 0 byte length data. Make sure we're only reading
  //      The value if the DLC is 8
	switch (prechargeState)
	{
		case (PRECHARGE_OFF):
			// set ready to drive to false
			readyToDrive = 0;

      // this state sends a message to rinehart
      if (lastPrechargeState != prechargeState)
      {
        // message is sent to rinehart to turn everything off
        TxData[0] = 1;          // parameter address. LSB
        TxData[1] = 0;          // parameter address. MSB
        TxData[2] = 1;          // Read / Write. 1 is write
        TxData[3] = 0;          // N/A
        TxData[4] = 0;          // Data. "0" off, "1": relay 1 on, "2": relay 2 on, "3": relay 1 and 2 on
        TxData[5] = 55;         // 55 means relay control
        TxData[6] = 0;          // N/A
        TxData[7] = 0;          // N/A

        // send message
        HAL_CAN_AddTxMessage(&hcan1, &txHeader2, TxData, &TxMailbox);
        
        // update last precharge state
        lastPrechargeState = prechargeState;
      }

      // move to precharge on
      prechargeState = PRECHARGE_ON;
		break;

		case (PRECHARGE_ON):
      // not ready to drive yet
      readyToDrive = 0;

      // turn on precharge relay
      // this state sends a message to rinehart to turn 
      if (lastPrechargeState != prechargeState)
      {
        // message is sent to rinehart to turn on precharge relay
        // precharge relay is on relay 1 from Rinehart
        TxData[0] = 1;            // parameter address. LSB
        TxData[1] = 0;            // parameter address. MSB
        TxData[2] = 1;            // Read / Write. 1 is write
        TxData[3] = 0;            // N/A
        TxData[4] = 1;            // Data. "0" off, "1": relay 1 on, "2": relay 2 on, "3": relay 1 and 2 on
        TxData[5] = 55;           // 55 means relay control
        TxData[6] = 0;            // N/A
        TxData[7] = 0;            // N/A

        // send message
        HAL_CAN_AddTxMessage(&hcan1, &txHeader2, TxData, &TxMailbox);
        
        // update last precharge state
        lastPrechargeState = prechargeState;
      }
      
			// ensure voltages are above correct values
			if ((rinehartVoltage > (bmsVoltage * PRECHARGE_COEFFICIENT)) && (bmsVoltage > 220))   // 220 so not just 1 pack can trigger this
      {
        prechargeState = PRECHARGE_DONE;
      }

      // if we do this for too long, move to error state
      // if (voltageCheckCount >= NUM_VOLTAGE_CHECKS)
      // {
      //   prechargeState = PRECHARGE_ERROR;
      // }
      
      // else
      // {
      //   voltageCheckCount++; // add to the counter. 
      // }
		break;

		case (PRECHARGE_DONE):
      // this state sends a message to rinehart to turn 
      if (lastPrechargeState != prechargeState)
      {
        // message is sent to rinehart to turn everything on
        // Keep precharge relay on and turn on main contactor
        TxData[0] = 1; // parameter address. LSB
        TxData[1] = 0; // parameter address. MSB
        TxData[2] = 1; // Read / Write. 1 is write
        TxData[3] = 0; // N/A
        TxData[4] = 3; // Data. "0" off, "1": relay 1 on, "2": relay 2 on, "3": relay 1 and 2 on
        TxData[5] = 55; // 55 means relay control
        TxData[6] = 0; // N/A
        TxData[7] = 0; // N/A

        // send message
        HAL_CAN_AddTxMessage(&hcan1, &txHeader2, TxData, &TxMailbox);

        // now that precharge is complete we can drive the car
        readyToDrive = 1;
          
        // update last precharge state
        lastPrechargeState = prechargeState;
      }

      // if rinehart voltage drops below battery, something's wrong, 
      // turn everything off
			// if (rinehartVoltage <= (bmsVoltage * (PRECHARGE_COEFFICIENT)-20)) // 20 is a magic number
      // {
      //   prechargeState = PRECHARGE_OFF; // something weird happened. Go to error
      // }
		break;

		case (PRECHARGE_ERROR):
			// the car is most definitly not ready to drive
      // set ready to drive off 
			readyToDrive = 0;

      if (lastPrechargeState != prechargeState)
      {
        // message is sent to rinehart to turn everything off
        TxData[0] = 1;            // parameter address. LSB
        TxData[1] = 0;            // parameter address. MSB
        TxData[2] = 1;            // Read / Write. 1 is write
        TxData[3] = 0;            // N/A
        TxData[4] = 0;            // Data. "0" off, "1": relay 1 on, "2": relay 2 on, "3": relay 1 and 2 on
        TxData[5] = 55;           // 55 means relay control
        TxData[6] = 0;            // N/A
        TxData[7] = 0;            // N/A

        // send message
        HAL_CAN_AddTxMessage(&hcan1, &txHeader2, TxData, &TxMailbox);
        
        // update last precharge state
        lastPrechargeState = prechargeState;
      }
		break;

		default:
      // fall back state, this indicates we did some undefined action that brought us here
      // we will move to PRECHARGE_ERROR to ensure readyToDrive stays false :)
			prechargeState = PRECHARGE_ERROR;
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // on __Hz interval
  if (htim == &htim14)
  {
    // build message for _____
    TxData[0] = readyToDrive;               // controled by precharge
    TxData[1] = DCDCFault;                  // 0 for now TODO: implement fault detection
    TxData[2] = vicoreTemp;                 // DMA update
    TxData[3] = rinehartVoltage & 0xFF;     // update on CAN message LSB
    TxData[4] = rinehartVoltage >> 8;       // update on CAN message MSB
    TxData[5] = bmsVoltage & 0xFF;          // update on CAN message LSB
    TxData[6] = bmsVoltage >> 8;            // update on CAN message MSB
    TxData[7] = 0x07;

    // send message
    HAL_CAN_AddTxMessage(&hcan1, &txHeader1, TxData, &TxMailbox);
  }

  // on __Hz interval 
  if(htim == &htim13)
  {
    // check precharge status
    prechargeControl();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // TODO: check analog values for the temperature conversion
  // Define threshold for when the fan should turn on

  // update vicor temp
  vicoreTemp = adc_buf[0];

  // set fan based on value
  if (vicoreTemp >= 2048)
  {
    // set the fan high
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  }

  else
  {
    // set fan low
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  }
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

