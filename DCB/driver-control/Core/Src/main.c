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
#include "LiquidCrystal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// inputs
#define PIN_START_BUTTON              		28    			// ready to drive button
#define PIN_DRIVE_DIRECTION           		26				// drive direction toggle
#define PIN_BRAKE_REGEN					    19		        // brake regeneration
#define PIN_COAST_REGEN					    18				// coast regeneration
#define PIN_COOLING_TOGGLE			    	25				// toggle the cooling 
#define PIN_FRONT_RIGHT_WHEEL			    11				// front right wheel speed sensor
#define PIN_FRONT_LEFT_WHEEL			    10				// front left wheel speed sensor
#define PIN_FRONT_RIGHT_SUSPENSION			13				// front right suspension
#define PIN_FRONT_LEFT_SUSPENSION		    12				// front left suspension
#define PIN_PEDAL_0					        16				// go pedal sensor 1
#define PIN_PEDAL_1					        17				// go pedal sensor 2
#define PIN_BRAKE_0					        14				// brake sensor 1
#define PIN_BRAKE_1				            15				// brake sensor 2
#define PIN_STEERING		    	        34				// THIS IS NOT CORRECT, JUST WASN'T LISTED IN DOC

// outputs
#define PIN_LCD_SDA						    29				// LCD sda
#define PIN_LCD_SCL						    30				// LCD scl
#define PIN_LCD_BUTTON					    27				// LCD control button
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
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

// CAN
CAN_RxHeaderTypeDef rxHeader; // CAN Bus Transmit Header
CAN_TxHeaderTypeDef txHeader; // CAN Bus Receive Header
uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // CAN Bus Receive Buffer
CAN_FilterTypeDef canFilter; // CAN Bus Filter
uint32_t canMailbox; // CAN Bus Mail box variable

// rinehart & emus
uint16_t rinehartVoltage = 0;				  // voltage in rinehart
uint16_t emusVoltage = 265.0;				  // emus bus voltage

// inputs
float coastRegen, brakeRegen;			    // coast and brake regen values 
float pedal0, pedal1;                 // pedal values
float brake0, brake1;                 // brake values
uint8_t coastMap, brakeMap;					  // maps for coast and brake regen
float wheelSpeedFR = 0;               // read from sensor input
float wheelSpeedFL = 0;               // read from sensor input
float wheelSpeedBR = 0;               // this needs to be retrieved from CAN
float wheelSpeedBL = 0;               // this needs to be retrieved from CAN
float rideHeightFR = 0;               // read from sensor input
float rideHeightFL = 0;               // read from sensor input
float rideHeightBR = 0;               // this needs to be retrieved from CAN
float rideHeightBL = 0;               // this needs to be retrieved from CAN
int startButtonState = 0;             // start button state (0 is not active)

// outputs
int RTDButtonLEDState = 0;            // RTD button LED toggle (0 is off)
int cooling = 0;                     	// cooling toggle (0 is off)
int direction = 0;		                // drive direction (0 is forwards)

// screen enum
enum screens
{
	RACING_HUD,               // for driving the car
	RIDE_SETTINGS,            // view all ride style settings
	ELECTRICAL_SETTINGS       // view all electrical information
};
int currentScreen = RACING_HUD;   // set the default screen mode to the racing HUD

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
	// init the lcd screen
	// LiquidCrystal lcd(rs_pin, enable_pin, PIN_LCD_SDA, PIN_LCD_SCL);   // we need to figure out the rs and enables pins!!!!!!!!!

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

	// init the CAN mailbox
	txHeader.DLC = 8; // Number of bites to be transmitted max- 8
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = 0x030;
	txHeader.ExtId = 0x02;
	txHeader.TransmitGlobalTime = DISABLE;

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
	MX_ADC1_Init();
	MX_CAN1_Init();

	/* USER CODE BEGIN 2 */

	// start up LCD display
	welcomeScreen();
	HAL_Delay(3000);      // just a little delay to give allow for some time to read the welcome screen

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
    // poll sensor data
    pollSensorData();

    // read can messages

    // send can messages
    uint8_t csend[] = {wheelSpeedFR, wheelSpeedFL, rideHeightFR, rideHeightFL, 0x00, 0x00, 0x00, startButtonState}; // Tx Buffer
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, csend, &canMailbox); // Send Message

    // check for lcd button press to change screeens
    if (HAL_GPIO_ReadPin(GPIOB, PIN_LCD_BUTTON) == 0)
    {
      currentScreen++;
      // loop back the first screen after reaching the last one 
      if (currentScreen == 3) currentScreen = RACING_HUD;
    }

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

	// ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  /*
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

void ADC_Select_CH_WSFL(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_WSFR(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_RHFL(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_RHFR(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_B0(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_B1(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_P0(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_P1(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_CR(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_Select_CH_BR(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  
  sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
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
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
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

void pollSensorData()
{
  // get front right wheel speed
  ADC_Select_CH_WSFR();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  wheelSpeedFR = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get front left wheel speed
  ADC_Select_CH_WSFL();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  wheelSpeedFL = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get front right ride height
  ADC_Select_CH_RHFR();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rideHeightFR = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get front left ride height
  ADC_Select_CH_RHFL();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  rideHeightFL = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get pedal 0
  ADC_Select_CH_P0();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  pedal0 = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get pedal 1
  ADC_Select_CH_P1();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  pedal1 = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get brake 0
  ADC_Select_CH_B0();
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brake0 = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get brake 1
  ADC_Select_CH_B1();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brake1 = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get coast regen
  ADC_Select_CH_CR();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  coastRegen = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  // get brake regen
  ADC_Select_CH_BR();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1000);
  brakeRegen = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
}

/**
 * @brief CAN read message function
 * 
 * @param hcan1
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);  // toggle PA3 LED
}

/**
 * @brief welcome & boot screen
 * 
 */
void welcomeScreen()
{
	lcd.begin(16, 2);                   // init lcd
	lcd.clear();                        // clear the screen
	lcd.setCursor(2, 0);                // set the cursor
	lcd.print("welcome AERO!");         // print
	lcd.setCursor(2, 1);                // next line
	lcd.print("booting up...");         // print
}


/**
 * @brief racing hud: mph(est), battery%, drive direction, coast regen, brake regen
 *
 */
void racingHUD()
{
	// get current MPH
	float averageWheelSpeed = (wheelSpeedFR + wheelSpeedFL) / 2;
	float currentMPH = averageWheelSpeed * WHEEL_DIAMETER * (3.14159) * 60 / 63360; // pi and inches in a mile

	// get battery percentage
	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

	// clear display
	lcd.clear();

	// drive direction
	lcd.setCursor(1, 0);                      // position of drive direction
	if (direction) lcd.print("FWD");          // print drive direction
	else lcd.print("RVS");

	// battery percentage
	lcd.setCursor(12, 0);                     // set cursor for battery percentage value
	lcd.print((int)batteryPercentage);        // print the battery percentage value
	lcd.setCursor(15, 0);                     // set cursor for % sign
	lcd.print("%%");                          // print % sign

	// speedometer
	lcd.setCursor(6, 0);                      // set cursor for mph value
	lcd.print((int)currentMPH);               // print the current speed in MPH, cast to int to round to whole number
	lcd.setCursor(6, 1);                      // set cursor for units
	lcd.print("mph");                         // print units

	// coast regen
	lcd.setCursor(1, 1);                      // set cursor for CR
	lcd.print("CR:");                         // print CR for coast regen
	lcd.setCursor(4, 1);                      // set cursor for coast regen value
	lcd.print((int)coastRegen);    			    	// print coast regen value
	lcd.setCursor(6, 1);                      // set cursor for percent sign
	lcd.print("%%");                          // print percent sign

	// brake regen
	lcd.setCursor(10, 1);                     // set cursor for BR
	lcd.print("BR:");                         // print BR for brake regen
	lcd.setCursor(12, 1);                     // set cursor for brake regen value
	lcd.print((int)brakeRegen);    		    		// print brake regen value
	lcd.setCursor(14, 1);                     // set cursor for percent sign
	lcd.print("%%");                          // print percent sign
}


/**
 * @brief battery state, bus voltage, rinehart voltage, power mode
 * 
 */
void electricalSettings()
{
	// get battery percentage
	float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

	// clear screen
	lcd.clear();

	// battery percentage
	lcd.setCursor(1, 0);                                  // set cursor for battery percentage value
	lcd.print(batteryPercentage);                         // print the current battery percentage value
	lcd.setCursor(4, 0);                                  // set cursor for % sign
	lcd.print("%%");                                      // print % sign

	// bus voltage
	lcd.setCursor(1, 1);                                  // set cursor for battery percentage value
	lcd.print(emusVoltage);                               // print the emus voltage value
	lcd.setCursor(4, 1);                                  // set cursor for units
	lcd.print("V");                                       // print units

	// rinehart voltage
	lcd.setCursor(12, 0);                                 // set cursor for rinehart voltage value
	lcd.print(rinehartVoltage);                           // print the rinehart voltage value
	lcd.setCursor(15, 0);                                 // set cursor for units
	lcd.print("V");                                       // print % sign

	// power mode
	lcd.setCursor(8, 1);                                  // set cursor for mode text
	lcd.print("Mode:");                                   // print mode text
	lcd.setCursor(12, 1);                                  // set cursor current mode setting
	if (powerMode == TUTORIAL) lcd.print("Tutorial");
	if (powerMode == ECO) lcd.print("Eco");
	if (powerMode == EXPERT) lcd.print("Expert");
}


/**
 * @brief ride height, wheel rpm, coast regen, brake regen
 *
 */
void rideSettings()
{
	// clear screen
	lcd.clear();

	// not sure what to do for suspension values yet so
	lcd.setCursor(6, 0);                  // ride height percentage text
	lcd.print("Ride %%");                 // print text

	lcd.setCursor(1, 0);                  // set cursor for wheel speed value
	lcd.print((int)rideHeightFL);         // print front left ride height value

	lcd.setCursor(3, 0);                  // set cursor for "-"
	lcd.print("-");                       // print the "-"

	lcd.setCursor(5, 0);                  // set cursor for wheelspeed value
	lcd.print((int)rideHeightFR);         // print front right ride height value

	lcd.setCursor(1, 1);                  // set cursor for wheelspeed value
	lcd.print((int)rideHeightBL);         // print back left ride height value

	lcd.setCursor(3, 1);                  // set cursor for "-"
	lcd.print("-");                       // print the "-"

	lcd.setCursor(5, 1);                  // set cursor for wheelspeed value
	lcd.print((int)rideHeightBR);         // print back right ride height value


	// wheel speed
	lcd.setCursor(10, 0);                 // set cursor for RPM text
	lcd.print("RPM");                     // print RPM text

	lcd.setCursor(10, 0);                 // set cursor for wheel speed value
	lcd.print((int)wheelSpeedFR);         // print value for front left

	lcd.setCursor(12, 0);                 // set cursor for "-"
	lcd.print("-");                       // print the "-"

	lcd.setCursor(14, 0);                 // set cursor for wheelspeed value
	lcd.print((int)wheelSpeedFL);         // print value for front right

	lcd.setCursor(10, 1);                 // set cursor for wheelspeed value
	lcd.print((int)wheelSpeedBL);         // print value rear left

	lcd.setCursor(12, 1);                 // set cursor for "-"
	lcd.print("-");                       // print the "-"

	lcd.setCursor(14, 1);                 // set cursor for wheelspeed value
	lcd.print((int)wheelSpeedBR);         // print value for rear right

	// coast regen
	lcd.setCursor(7, 0);                  // set cursor for CR text
	lcd.print("CR:");                     // print "CR:" for coast regen
	lcd.print((int)coastRegen);           // print coast regen value
	lcd.setCursor(9, 0);                  // set cursor for "%"
	lcd.print("%%");                      // print "%"

	// brake regen
	lcd.setCursor(7, 1);                  // set cursor for BR text
	lcd.print("BR:");                     // print "BR:" for brake regen
	lcd.print((int)brakeRegen);           // print brake regen value
	lcd.setCursor(9, 1);                  // set cursor for "%"
	lcd.print("%%");                      // print "%"
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

