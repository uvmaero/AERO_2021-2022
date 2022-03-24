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

// inputs
#define PIN_DC_DC_FAULT             12            // DC DC fault indicator pin
#define PIN_VICOR_TEMP              17            // temperature inside vicore

// outputs 
#define PIN_DC_DC_ENABLE            11          // DC DC control pin

// CAN
#define PIN_CAN_PLUS                            // positve CAN wire
#define PIN_CAN_MINUS                           // negative CAN wire

// precharge
#define PRECHARGE_COEFFICIENT       0.9		      // 90% complete with precharge so it's probably safe to continue
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

// inputs
float rinehartVoltage = 0;				// read from CAN
float emusVoltage = 0;					// read from CAN
int DCDCEnable = 0;                     // dc-dc enable (0 = disabled, 1 = enabled)
float vicoreTemp = 0;                   // temperature of vicore
int RTDButtonPressed = 0;               // read this from CAN, if it's 1 we can finish precharge

// output
int DCDCFault = 0;                      // the dc-dc fault indicator (0 = no fault, 1 = fault)
int readyToDrive = 0;					          // the car is ready to drive! (0 = not ready, 1 = ready)

// precharge states
enum prechargeStates
{
	PRECHARGE_OFF,
	PRECHARGE_ON,
	PRECHARGE_DONE,
	PRECHARGE_ERROR
};
int prechargeState = PRECHARGE_OFF;			// set intial precharge state to OFF

// CAN
uint32_t canMailbox; 							            // CAN Bus Mail box variable
CAN_TxHeaderTypeDef txHeader0; 					      // CAN Bus Transmit Header BASE
CAN_TxHeaderTypeDef txHeader1; 					      // CAN Bus Transmit Header DATA
CAN_RxHeaderTypeDef rxHeader; 					      // CAN Bus Receive Header
uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 	// CAN Bus Receive Buffer
CAN_FilterTypeDef canFilter0; 					      // CAN Bus Filter for BMS
CAN_FilterTypeDef canFilter1;                 // CAN Bus Filter for Rinehart
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void prechargeControl();
void pollSensorData();
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
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  // init the CAN filter for BMS messages
    canFilter0.FilterIdHigh = 0x72 << 5;   // BMS IDs: 0 - 0x72
  	canFilter0.FilterIdLow = 0;
    canFilter0.FilterMaskIdHigh = 0x72 << 5;
  	canFilter0.FilterMaskIdLow = 0x00;
    canFilter0.FilterBank = 0;
  	canFilter0.FilterMode = CAN_FILTERMODE_IDMASK;
  	canFilter0.FilterFIFOAssignment = CAN_RX_FIFO0;
  	canFilter0.FilterScale = CAN_FILTERSCALE_32BIT;
  	canFilter0.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilter0);

    // init the CAN filter for Rinehart messages
    canFilter1.FilterIdHigh = 0xB1 << 5;      // Rinehart IDs: 0xA0 - 0xB1
  	canFilter1.FilterIdLow = 0xA0;
    canFilter1.FilterMaskIdHigh = 0xB1 << 5;
  	canFilter1.FilterMaskIdLow = 0xA0;
    canFilter1.FilterBank = 0;
  	canFilter1.FilterMode = CAN_FILTERMODE_IDMASK;
  	canFilter1.FilterFIFOAssignment = CAN_RX_FIFO0;
  	canFilter1.FilterScale = CAN_FILTERSCALE_32BIT;
  	canFilter1.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilter1);

  	// init the CAN mailbox for BASE
  	txHeader0.DLC = 8; // Number of bites to be transmitted max- 8
  	txHeader0.IDE = CAN_ID_STD;
  	txHeader0.RTR = CAN_RTR_DATA;
  	txHeader0.StdId = 0x86;
  	txHeader0.ExtId = 0x02;
  	txHeader0.TransmitGlobalTime = DISABLE;

  	// init the CAN mailbox for DATA
  	txHeader1.DLC = 8; // Number of bites to be transmitted max- 8
  	txHeader1.IDE = CAN_ID_STD;
  	txHeader1.RTR = CAN_RTR_DATA;
  	txHeader1.StdId = 0x87;
  	txHeader1.ExtId = 0x03;
  	txHeader1.TransmitGlobalTime = DISABLE;

	HAL_CAN_Start(&hcan1); // Initialize CAN Bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// poll sensors
	pollSensorData();

	// read CAN
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();

	// send CAN
	uint8_t csend0[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};  // BASE
	HAL_CAN_AddTxMessage(&hcan1, &txHeader0, csend0, &canMailbox); // Send Message

	uint8_t csend1[] = {readyToDrive, DCDCFault, vicoreTemp, 0x03, 0x04, 0x05, 0x06, 0x07}; 	// DATA
	HAL_CAN_AddTxMessage(&hcan1, &txHeader1, csend1, &canMailbox); // Send Message
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  if (rxHeader.StdId == 0xA7)
  {
    // rinehart voltage is spread across the first 2 bytes
	  int rine1 = canRX[0];
    int rine2 = canRX[1];
    // combine the first two bytes and assign that to the rinehart voltage
    rinehartVoltage = (rine1 << 8) | rine2;
  }

  // get BMS total voltages
  if (rxHeader.StdId == 0x1)
  {
    int volt1 = canRX[4];
    int volt2 = canRX[3];
    int volt3 = canRX[6];
    int volt4 = canRX[5];

    int emus1 = (volt1 << 8) | volt2;
    int emus2 = (volt3 << 8) | volt4;
    emusVoltage = (emus1 << 8) | emus2;   // if this doesn't work then change the 8 to 16
  }
}

/**
 * @brief 
 * 
 */
void pollSensorData()
{
  // get vicore temp 
  HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	vicoreTemp = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

  // get dc-dc fault status
  DCDCFault = HAL_GPIO_ReadPin(GPIOA, PIN_DC_DC_FAULT);
}


/**
 * @brief 
 * 
 */
void prechargeControl()
{
	switch (prechargeState)
	{
		case (PRECHARGE_OFF):
			// set ready to drive to false
			readyToDrive = 0;
      
      // if the dc dc system is on then move to precharge on
      if (DCDCEnable == 1)
        prechargeState = PRECHARGE_ON;

		break;

		case (PRECHARGE_ON):
		  // ensure voltages are above correct values
			if (rinehartVoltage >= (emusVoltage * PRECHARGE_COEFFICIENT))
			{ 
        // if the RTD button is pressed, move to precharge done 
        if (RTDButtonPressed == 1)
				  prechargeState = PRECHARGE_DONE;
			}

      // if we have a DCDC fault, end precharge
      if (DCDCFault == 1)
        prechargeState = PRECHARGE_ERROR;

		break;

		case (PRECHARGE_DONE):
			// now that precharge is complete we can drive the car
			readyToDrive = 1;
		break;

		case (PRECHARGE_ERROR):
			// the car is most definitly not ready to drive
			// probably requires hard reboot of systems to clear this state
			readyToDrive = 0;
		break;

		default:
      // fallback state, this indicates we did some undefined action that brought us here
      // we will move to PRECHARGE_ERROR to ensure readyToDrive stays false :)
			prechargeState = PRECHARGE_ERROR;
		break;
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

