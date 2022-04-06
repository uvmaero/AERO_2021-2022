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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hd44780_i2c.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// inputs
#define PIN_FRONT_LEFT_WHEEL			    0				// front left wheel speed sensor
#define PIN_FRONT_RIGHT_WHEEL			    1				// front right wheel speed sensor
#define PIN_FRONT_LEFT_SUSPENSION		    2				// front left suspension
#define PIN_FRONT_RIGHT_SUSPENSION			3				// front right suspension
#define PIN_BRAKE_0					        4				// brake sensor 1
#define PIN_BRAKE_1				            5				// brake sensor 2
#define PIN_PEDAL_0					        6				// go pedal sensor 1
#define PIN_PEDAL_1					        7				// go pedal sensor 2
#define PIN_COAST_REGEN					    8				// coast regeneration
#define PIN_BRAKE_REGEN					    9		        // brake regeneration
#define PIN_COOLING_TOGGLE			    	25				// toggle the cooling 
#define PIN_DRIVE_DIRECTION           		12				// drive direction toggle
#define PIN_START_BUTTON              		13    			// ready to drive button
#define PIN_LCD_BUTTON					    15				// LCD control button

#define ADC_BUF_LEN							10				// length of dma adc buffer

// outputs
#define PIN_LCD_SDA						    29				// LCD sda
#define PIN_LCD_SCL						    30				// LCD scl
#define PIN_RTD_LED							45				// RTD button LED
#define PIN_IHD							    20				// IHD Fault LED
#define PIN_AMS						        31				// AMS LED

// CAN
#define PIN_CAN_PLUS					    32				// positive CAN wire
#define PIN_CAN_MINUS					    33				// negative CAN wire

// wheel diameter
#define WHEEL_DIAMETER                		20.5    		// diameter of the wheels in inches

// pack voltage
#define MAX_PACK_VOLTAGE             	 	265   			// max pack voltage for calculating pack capacity percentage
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

// CAN
CAN_RxHeaderTypeDef rxHeader; 					      // CAN Bus Receive Header
CAN_TxHeaderTypeDef txHeader0; 					      // CAN Bus Transmit Header BASE
CAN_TxHeaderTypeDef txHeader1; 					      // CAN Bus Transmit Header Torque Setting
CAN_TxHeaderTypeDef txHeader2; 					      // CAN Bus Transmit Header DAQ Data
CAN_TxHeaderTypeDef txHeader3; 					      // CAN Bus Transmit Header Control Data
uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Bus Receive Buffer
CAN_FilterTypeDef canFilter; 					        // CAN Bus Filter
uint32_t canMailbox; 							            // CAN Bus Mail box variable

// rinehart & emus  
float rinehartVoltage = 0;				  		      // voltage in rinehart
float emusVoltage = 0;				  			        // emus bus voltage
int readyToDrive = 0;							            // ready to drive (0 = no, 1 = yes)

// inputs 
float coastRegen, brakeRegen;			    	      // coast and brake regen values 
float pedal0, pedal1;                 			  // pedal values
float brake0, brake1;                		  	  // brake values
uint8_t coastMap, brakeMap;						        // maps for coast and brake regen
float wheelSpeedFR = 0;               			  // read from sensor input
float wheelSpeedFL = 0;               			  // read from sensor input
float wheelSpeedBR = 0;               			  // this needs to be retrieved from CAN
float wheelSpeedBL = 0;               			  // this needs to be retrieved from CAN
float rideHeightFR = 0;               			  // read from sensor input
float rideHeightFL = 0;               			  // read from sensor input
float rideHeightBR = 0;               			  // this needs to be retrieved from CAN
float rideHeightBL = 0;               			  // this needs to be retrieved from CAN
int startButton = 0;             				      // start button state (0 is not active)
int cooling = 0;								              // cooling toggle switch
uint32_t adc_buf[ADC_BUF_LEN];                // adc buffer for dma

// outputs
int startButtonState = 0;              			  // RTD button LED toggle (0 is off)
int coolingState = 0;                     		// cooling toggle (0 is off)
int direction = 0;		                		    // drive direction (0 is forwards)

// screen enum
enum screens
{
	RACING_HUD,               					        // for driving the car
	RIDE_SETTINGS,            					        // view all ride style settings
	ELECTRICAL_SETTINGS       					        // view all electrical information
};
int currentScreen = RACING_HUD;   				    // set the default screen mode to the racing HUD

// power modes
enum powerModes
{
	TUTORIAL,           // 50% throttle power, for beginner AERO drivers
	ECO,                // 75% throttle power, battery savings
	EXPERT              // 100% throttle power, max speed and acceleration
};
int powerMode = EXPERT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void welcomeScreen();
void racingHUD();
void electricalSettings();
void rideSettings();
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // start up LCD display
  welcomeScreen();

  // TODO: FIX THE CAN MASK RANGES SO THEY ARE CORRECT 

  // init the CAN filter
	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterIdHigh = 0x000;
	canFilter.FilterIdLow = 0xFFF;
	canFilter.FilterMaskIdHigh = 0x000;
	canFilter.FilterMaskIdLow = 0xFFF;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterActivation = ENABLE;
	canFilter.SlaveStartFilterBank = 14;

	// init the CAN mailbox for BASE
	txHeader0.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader0.IDE = CAN_ID_STD;
	txHeader0.RTR = CAN_RTR_DATA;
	txHeader0.StdId = 0x90;
	txHeader0.ExtId = 0;
	txHeader0.TransmitGlobalTime = DISABLE;

	// init the CAN mailbox for Torque Setting 
	txHeader1.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader1.IDE = CAN_ID_STD;
	txHeader1.RTR = CAN_RTR_DATA;
	txHeader1.StdId = 0x91;
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

	HAL_CAN_ConfigFilter(&hcan1, &canFilter); // Initialize CAN Filter
	HAL_CAN_Start(&hcan1); // Initialize CAN Bus
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt

	// start the dma adc conversion
	HAL_ADC_Start_DMA(&hadc1, adc_buf, ADC_BUF_LEN);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  /* USER CODE END CAN1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
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
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX) != HAL_OK)
    Error_Handler();

  // get sensor data from rcb
  if (rxHeader.StdId == 0x81)
  {
	  wheelSpeedBL = canRX[0];
	  wheelSpeedBR = canRX[1];
	  rideHeightBL = canRX[2];
	  rideHeightBR = canRX[3];
  }

  // get ready to drive from high voltage for precharge complete
  if (rxHeader.StdId == 0x87)
	  readyToDrive = canRX[0];
}

void pollSensorData()
{
	// get front right wheel speed
	wheelSpeedFL = adc_buf[0];

	// get front left wheel speed
	wheelSpeedFR = adc_buf[1];

	// get front right ride height
	rideHeightFL = adc_buf[2];

	// get front left ride height
	rideHeightFR = adc_buf[3];

	// get brake 0
	brake0 = adc_buf[4];

	// get brake 1
	brake1 = adc_buf[5];

	// get pedal 0
	pedal0 = adc_buf[6];

	// get pedal 1
	pedal1 = adc_buf[7];

	// get coast regen
	coastRegen = adc_buf[8];

	// get brake regen
	brakeRegen = adc_buf[9];
}



/**
 * @brief welcome & boot screen
 * 
 */
void welcomeScreen()
{
	lcdInit(&hi2c1, 0x27, 2, 16);       			// init lcd (i2c reference, LCD address, lines, rows)
	lcdAutoscrollOff();								// turn off autoscroll
	lcdBacklightOn();								// turn on backlight
	lcdDisplayClear();                  			// clear the screen
	lcdSetCursorPosition(2, 0);         			// set the cursor
	lcdPrintStr((uint8_t*)"welcome AERO!", 13);   	// print
	lcdSetCursorPosition(2, 1);         			// next line
	lcdPrintStr((uint8_t*)"booting up...", 13);   	// print
	HAL_Delay(3000);								// delay 3 seconds so the screen can be read
	lcdDisplayClear();								// clear the display so the other screens can be printed
}


/**
 * @brief racing hud: mph(est), battery%, drive direction, coast regen, brake regen
 *
 */
void racingHUD()
{
	// get wheel speed
	float averageWheelSpeed = (wheelSpeedFR + wheelSpeedFL) / 2;

	// get current mph from wheel speed
	float currentMPH = ((averageWheelSpeed * WHEEL_DIAMETER) * (3.14159 * 60)) / 63360;

	// get battery percentage
	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

	// init some char buffs for variables
	char battStr[10];
	char speedStr[10];
	char coastStr[10];
	char brakeStr[10];

	// drive direction
	lcdSetCursorPosition(0, 0);									// position of drive direction
	if (direction) lcdPrintStr((uint8_t*)"FWD", 3);     		// print drive direction
	else lcdPrintStr((uint8_t*)"RVS", 3);

	// battery percentage
	lcdSetCursorPosition(12, 0); 								// set cursor for battery percentage value
	sprintf(battStr, "%.0d%%", (int)batteryPercentage); 		// sprintf it
	lcdPrintStr((uint8_t*)battStr, strlen(battStr));			// print the battery percentage value

	// speedometer		
	lcdSetCursorPosition(7, 0);                     			// set cursor for mph value
	sprintf(speedStr, "%.0d", (int)currentMPH);					// sprintf it
	lcdPrintStr((uint8_t*)speedStr, strlen(speedStr));			// print the current speed in MPH, cast to int to round to whole number
	lcdSetCursorPosition(7, 1);                     			// set cursor for units
	lcdPrintStr((uint8_t*)"mph", 3);                    		// print units

	// coast regen		
	lcdSetCursorPosition(0, 1);                      			// set cursor for CR
	sprintf(coastStr, "C:%.0d%%", (int)coastRegen);				// sprintf it
	lcdPrintStr((uint8_t*)coastStr, strlen(coastStr));  		// print coast regen value

	// brake regen		
	lcdSetCursorPosition(11, 1);                     			// set cursor for BR
	sprintf(brakeStr, "B: %d%%", (int)brakeRegen);				// sprintf it
	lcdPrintStr((uint8_t*)brakeStr, strlen(brakeStr));  		// print brake regen value
}


/**
 * @brief battery state, bus voltage, rinehart voltage, power mode
 * 
 */
void electricalSettings()
{
	// get battery percentage
	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

	// init some char buffs for variables
	char battStr[10];
	char busVStr[10];

	// battery percentage
	lcdSetCursorPosition(0, 0);									// set cursor for battery title
	sprintf(battStr, "Batt:%d%%", (int)batteryPercentage);		// sprintf it
	lcdPrintStr((uint8_t*)battStr, strlen(battStr));			// print title

	// bus voltage
	lcdSetCursorPosition(11, 0);								// set cursor for bus voltage title
	sprintf(busVStr, "Bus:%d", (int)emusVoltage);				// sprintf it			
	lcdPrintStr((uint8_t*)busVStr, strlen(busVStr));			// print
	lcdSetCursorPosition(15, 1);                                // set cursor for units
	lcdPrintStr((uint8_t*)"V", 1);                              // print units

	/*	not planning on using this for the time being
	// rinehart voltage
	lcdSetCursorPosition(12, 0);                                // set cursor for rinehart voltage value
	lcdPrintStr(rinehartVoltage);                           	// print the rinehart voltage value
	lcdSetCursorPosition(15, 0);                                // set cursor for units
	lcdPrintStr("V");                                       	// print % sign
	*/

	// power mode
	lcdSetCursorPosition(0, 1);                                 // set cursor for mode text
	lcdPrintStr((uint8_t*)"Mode:", 5);							// print mode text
	lcdSetCursorPosition(5, 1);                                 // set cursor current mode setting
	if (powerMode == TUTORIAL) lcdPrintStr((uint8_t*)"TUTR", 4);
	if (powerMode == ECO) lcdPrintStr((uint8_t*)"ECO", 3);
	if (powerMode == EXPERT) lcdPrintStr((uint8_t*)"EXPT", 4);
	else lcdPrintStr((uint8_t*)(uint8_t*)"ERR!", 4);
}


/**
 * @brief ride height, wheel rpm, coast regen, brake regen
 *
 */
void rideSettings()
{
	// init some char buffs for variables
	char rideStr[10];
	char wheelStr[10];

	// ride height
	lcdSetCursorPosition(0, 0);									// set cursor for front left ride height value
	sprintf(rideStr, "%d", (int)rideHeightFL);					// sprintf it
	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print front left ride height value

	lcdSetCursorPosition(2, 0);				  					// spacer
	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

	lcdSetCursorPosition(3, 0);									// set cursor for front right ride height value
	sprintf(rideStr, "%d", (int)rideHeightFR);					// sprintf it
	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print front right ride height value

	lcdSetCursorPosition(5, 0);									// set cursor for "<- Ride"
	lcdPrintStr((uint8_t*)"<-Ride", 6);							// print

	lcdSetCursorPosition(0, 1);                  				// set cursor for back left ride height value
	sprintf(rideStr, "%d", (int)rideHeightBL);					// sprintf it
	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));         	// print back left ride height value

	lcdSetCursorPosition(2, 1);				  					// spacer
	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

	lcdSetCursorPosition(3, 1);                  				// set cursor for back right ride height value
	sprintf(rideStr, "%d", (int)rideHeightBR);					// sprintf it
	lcdPrintStr((uint8_t*)rideStr, strlen(rideStr));			// print back right ride height value

	lcdSetCursorPosition(6, 1);                  				// set cursor for "RPM->"
	lcdPrintStr((uint8_t*)"RPM->", 5);                   		// print the "RPM->"

	// wheel speed
	lcdSetCursorPosition(11, 0);								// set cursor for front left wheelspeed value
	sprintf(wheelStr, "%d", (int)wheelSpeedFL);					// sprintf it
	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print front left wheelspeed value

	lcdSetCursorPosition(13, 0);								// spacer
	lcdPrintStr((uint8_t*)"-", 1);						  		// spacer

	lcdSetCursorPosition(14, 0);                 				// set cursor for front right wheelspeed value
	sprintf(wheelStr, "%d", (int)wheelSpeedFR);					// sprintf it
	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print front right wheelspeed value

	lcdSetCursorPosition(11, 1);                 				// set cursor for back left wheelspeed value
	sprintf(wheelStr, "%d", (int)wheelSpeedBL);					// sprintf it
	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print back left wheelspeed value

	lcdSetCursorPosition(13, 1);								// set cursor for "-"
	lcdPrintStr((uint8_t*)"-", 1);								// print the "-"

	lcdSetCursorPosition(14, 1);								// set cursor for back right wheelspeed value
	sprintf(wheelStr, "%d", (int)wheelSpeedBR);					// sprintf it
	lcdPrintStr((uint8_t*)wheelStr, strlen(wheelStr));			// print value for back right wheelspeed value
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);

	// poll sensor data
	pollSensorData();

	// read can messages
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();

	// send can messages
	// BASE
	uint8_t csend0[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	HAL_CAN_AddTxMessage(&hcan1, &txHeader0, csend0, &canMailbox); // Send Message

	// idk what i wrote this for, im sure ill figure it out at some point
	uint8_t csend1[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	HAL_CAN_AddTxMessage(&hcan1, &txHeader1, csend1, &canMailbox); // Send Message

	// DAQ DATA
	uint8_t csend2[] = {wheelSpeedFL, wheelSpeedFR, rideHeightFL, rideHeightFR, brake0, brake1, pedal0, pedal1};
	HAL_CAN_AddTxMessage(&hcan1, &txHeader2, csend2, &canMailbox); // Send Message

	// CONTROL DATA
	uint8_t csend3[] = {coastRegen, brakeRegen, coolingState, direction, 0x04, 0x05, 0x06, 0x07};
	HAL_CAN_AddTxMessage(&hcan1, &txHeader3, csend3, &canMailbox); // Send Message


	// check for lcd button press to change screens
	int oldScreen = currentScreen;
	if (HAL_GPIO_ReadPin(GPIOB, PIN_LCD_BUTTON) == 0)
	{
		currentScreen++;
		// loop back the first screen after reaching the last one
		if (currentScreen > RIDE_SETTINGS) currentScreen = RACING_HUD;
	}

	// clear screen if the screen mode has been changed
	if (currentScreen != oldScreen) lcdDisplayClear();

	// screen updates
	switch (currentScreen)
	{
		case RACING_HUD:
			racingHUD();
		break;

		case ELECTRICAL_SETTINGS:
			electricalSettings();
		break;

		case RIDE_SETTINGS:
			rideSettings();
		break;

		default:
			// go to racing hud because were not supposed to be here
			currentScreen = RACING_HUD;
		break;
	}
  }
}
  /* USER CODE END 5 */


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

