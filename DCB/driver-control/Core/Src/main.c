/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#define ADC_BUF_LEN               4
#define MAX_PEDAL_SKEW            100
#define PEDAL_MAX                 1023

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

// CAN transmitting
CAN_TxHeaderTypeDef TxHeader;               // either daq or control idk which one address 
CAN_TxHeaderTypeDef TxHeader2;              // rinehart command message address
uint8_t TxData[8];
uint32_t TxMailbox;

// CAN reciving 
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_FilterTypeDef filter0;                  // filter for __________
CAN_FilterTypeDef filter1;                  // fiter for ___________

// signal variables (0 = off | 1 = on)
uint8_t imdFault = 0;             
uint8_t bmsFault = 0;
uint8_t switch_cooling = 0;
uint8_t switch_direction = 0;               // 0 = forward | 1 = reverse (this changes requires an inverter restart)

// analog pins
uint16_t adc_buf[ADC_BUF_LEN];
uint16_t pedal0 = 0;
uint16_t pedal1 = 1;
uint16_t pedalAverage = 0;

// state variables (0 = off | 1 = on)
uint8_t ready_to_drive = 0;                 // 0 until precharge is done
uint8_t buzzerState = 0;                    // for controlling the buzzer
uint8_t buzzerCounter = 0;                  // counter for how long the buzzer has been buzzing
uint8_t enableInverter = 0;                 // stores state of inverter, can only be 1 after buzzer is done buzzing 

// rinehart commands
uint16_t commandedTorque = 0;               // torque request sent to rinehart, init with 0 to prevent unintended acceleration 
uint16_t command_torque_limit = 0;          // max torque allowed to be requested from rinehart, init with 0 to prevent unintended accleration
enum mode                                   // create an enumeration for the drive modes
{
  SLOW,       // 50% power 
  ECO,        // 75% power
  FAST        // 100% power 
};
int driveMode = ECO;                        // set the inital drive mode of the car

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
uint16_t pedal_conversion(uint16_t pedal0, uint16_t pedal1);
long mapValue(long x, long in_min, long in_max, long out_min, long out_max);
uint16_t getCommandedTorque();

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

  // define TX header
  TxHeader.StdId = 0x093;
  TxHeader.ExtId = 0x0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
  
  // Rinehart command message
  TxHeader2.StdId = 0x0C0;
  TxHeader2.ExtId = 0x0;
  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.DLC = 8;
  TxHeader2.TransmitGlobalTime = DISABLE;

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

  // start interrupts
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

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

  filter0.FilterIdHigh = 0x082 << 5;
  filter0.FilterIdLow = 0x000;
  filter0.FilterMaskIdHigh = 0x082 << 5;
  filter0.FilterMaskIdLow = 0x000;
  filter0.FilterFIFOAssignment =  CAN_RX_FIFO0;
  filter0.FilterBank = 1;
  filter0.FilterMode = CAN_FILTERMODE_IDMASK;
  filter0.FilterScale = CAN_FILTERSCALE_32BIT;
  filter0.FilterActivation = ENABLE;
  filter0.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &filter0);

  // listen for HVB
  filter1.FilterIdHigh = 0x087 << 5;
  filter1.FilterIdLow = 0x000;
  filter1.FilterMaskIdHigh = 0x087 << 5;
  filter1.FilterMaskIdLow = 0x000;
  filter1.FilterFIFOAssignment =  CAN_RX_FIFO0;
  filter1.FilterBank = 2;
  filter1.FilterMode = CAN_FILTERMODE_IDMASK;
  filter1.FilterScale = CAN_FILTERSCALE_32BIT;
  filter1.FilterActivation = ENABLE;
  filter1.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &filter1);

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
  htim13.Init.Period = 500-1;
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
  htim14.Init.Period = 1000-1;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// re-map function
long mapValue(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// function to re-map the pedal value to a torque value based on the drive mode
uint16_t getCommandedTorque()
{
  // get the pedal average
  pedalAverage = pedal_conversion(pedal0, pedal1);

  // drive mode logic
  switch (driveMode)
  {
    case SLOW:
      command_torque_limit = 50;
      commandedTorque = mapValue(pedalAverage, 0, PEDAL_MAX, 0, command_torque_limit);
      return commandedTorque;
    break;

    case ECO:
      command_torque_limit = 75;
      commandedTorque = mapValue(pedalAverage, 0, PEDAL_MAX, 0, command_torque_limit);
      return commandedTorque;
    break;

    case FAST:
      command_torque_limit = 100;   // do not change this to more than 100 
      commandedTorque = mapValue(pedalAverage, 0, PEDAL_MAX, 0, command_torque_limit);
      return commandedTorque;
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      driveMode = ECO;

      // we don't want to send a torque if we are in an undefined state
      return commandedTorque = 0;
    break;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
    Error_Handler();
  }

  // read CAN data from Rear Control Board
  if (RxHeader.StdId == 0x082){
      imdFault = RxData[0];
      bmsFault = RxData[1];
  }

  // read CAN data from High Voltage Board 
  if (RxHeader.StdId == 0x087){
      ready_to_drive = RxData[0];
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // handle start button interrupt
  if (GPIO_Pin == GPIO_PIN_15)
  {
    // if ready to drive & the button has been pushed, start the buzzer
    if (ready_to_drive)
    {
      buzzerState = 1;
    }
  }
}

// Timer Interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Timer Interrupt on a __Hz interval
  if (htim == &htim13){

    // drive mode logic
    commandedTorque = getCommandedTorque();
    
    // build CONTROL CAN message
    TxData[0] = pedalAverage & 0xFF;
    TxData[1] = pedalAverage >> 8;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = switch_direction;
    TxData[5] = enableInverter;
    TxData[6] = commandedTorque & 0xFF;
    TxData[7] = commandedTorque >> 8;

    // send message
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData, &TxMailbox);
  }
  
  // Timer Interrupt on a __Hz interval
  if (htim == &htim14)
  {
    // sample cooling switch and drive direction switch
    switch_cooling = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    switch_direction = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

    // start button led logic
    if (ready_to_drive)
    {
      // turn the LED on
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    }
    else
    {
      // turn the LED off
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
      enableInverter = 0;
    }
    
    // buzzer logic
    if (buzzerState == 1){
      buzzerCounter++;
      if (buzzerCounter >= 20)   // buzzerCounter is being updated on a 10Hz interval, so after 20 cycles, 2 seconds have passed
      {
        buzzerState = 0;
        buzzerCounter = 0;
        enableInverter = 1;       // enable the inverter so that we can tell rinehart to turn inverter on
      }
    }

    // build CAN message 
    TxData[0] = pedal1 & 0xFF;
    TxData[1] = pedal1 >> 8;
    TxData[2] = switch_cooling;
    TxData[3] = pedalAverage & 0xFF;
    TxData[4] = buzzerState;
    TxData[6] = pedal0 & 0xFF;
    TxData[5] = pedal0 >> 8;
    TxData[7] = pedalAverage >>8;

    // send message
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

    // update LEDS and inverter drive direction 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, imdFault);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, bmsFault);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, buzzerState);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // read values from DMA
  pedal0 = adc_buf[2];
  pedal1 = adc_buf[3];

}

uint16_t pedal_conversion(uint16_t pedal0, uint16_t pedal1)
{
  // calculate the average of the two pedal potentiometer readings 
  pedalAverage = (pedal0 + pedal1) / 2;

  // ensure the pedal skew isn't dangerously out of bounds
  // if (pow(pedal0 - pedalAverage, 2) > MAX_PEDAL_SKEW || 
  //     pow(pedal1 - pedalAverage, 2) > MAX_PEDAL_SKEW ){
  //     pedalAverage = 0;
  // }

  return pedalAverage;
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

