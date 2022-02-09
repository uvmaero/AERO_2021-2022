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
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// inputs
#define PIN_START_BUTTON              28    // ready to drive button
#define PIN_DRIVE_DIRECTION           26		// drive direction toggle
#define PIN_BRAKE_REGEN					      19		// brake regeneration
#define PIN_COAST_REGEN					      18		// coast regeneration
#define PIN_COOLING_TOGGLE			    	25		// toggle the cooling 
#define PIN_FRONT_RIGHT_WHEEL			    11		// front right wheel speed sensor
#define PIN_FRONT_LEFT_WHEEL			    10		// front left wheel speed sensor
#define PIN_FRONT_RIGHT_SUSPENSION		13		// front right suspension
#define PIN_FRONT_LEFT_SUSPENSION		  12		// front left suspension
#define PIN_PEDAL_0					          16		// go pedal sensor 1
#define PIN_PEDAL_1					          17		// go pedal sensor 2
#define PIN_BRAKE_0					          14		// brake sensor 1
#define PIN_BRAKE_1					          15		// brake sensor 2
#define PIN_STEERING				          34		// THIS IS NOT CORRECT, JUST WASN'T LISTED IN DOC

// outputs
#define PIN_LCD_SDA						        29		// LCD sda
#define PIN_LCD_SCL						        30		// LCD scl
#define PIN_LCD_BUTTON					      27		// LCD control button
#define PIN_RTD_LED							      45		// RTD button LED
#define PIN_IHD							          20		// IHD Fault LED
#define PIN_AMS							          31		// AMS LED

// CAN
#define PIN_CAN_PLUS					        32		// positive CAN wire
#define PIN_CAN_MINUS					        33		// negative CAN wire

// wheel diameter
#define WHEEL_DIAMETER                16    // diameter of the wheels in inches

// pack voltage
#define MAX_PACK_VOLTAGE              265   // max pack voltage for calculating pack capacity percentage

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;

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
uint16_t coastRegen, brakeRegen;			// coast and brake regen values 
uint8_t coastMap, brakeMap;					  // maps for coast and brake regen
float wheelSpeedFR = 0;
float wheelSpeedFL = 0;
float wheelSpeedBR = 0;               // this needs to be retrieved from CAN
float wheelSpeedBL = 0;               // this needs to be retrieved from CAN
float rideHeightFR = 0;
float rideHeightFL = 0;
float rideHeightBR = 0;               // this needs to be retrieved from CAN
float rideHeightBL = 0;               // this needs to be retrieved from CAN

// outputs
int RTDButtonLED = 0;                 // RTD button LED toggle (0 is off)
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
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void welcomeScreen();
void racingHUD();
void electricalSettings();
void rideSettings();

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

  txHeader.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x030;
  txHeader.ExtId = 0x02;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_CAN_ConfigFilter(&hcan, &canFilter); // Initialize CAN Filter
  HAL_CAN_Start(&hcan); // Initialize CAN Bus
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);   // Initialize CAN Bus Rx Interrupt

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

  // start up LCD display
  welcomeScreen();
  HAL_Delay(3000);      // just a little delay to give allow for some time to read the welcome screen

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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 5;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

// *** functions *** //

/**
 * @brief CAN read message function
 * 
 * @param hcan
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, canRX); //Receive CAN bus message to canRX buffer
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);  // toggle PA3 LED
}


/**
 * @brief welcome & boot screen
 * 
 */
void welcomeScreen()
{
  lcd_init();                         // init lcd
  lcd_clear();                        // clear the screen
  lcd_put_cur(1, 0);                  // set the cursor
  lcd_send_string("welcome AERO!");   // print
  lcd_put_cur(2, 0);                  // next line
  lcd_send_string("booting up...");   // print
}


/**
 * @brief racing hud: mph(est), battery%, drive direction, coast regen, brake regen
 *
 */
void racingHUD()
{
  // get current MPH
  float averageWheelSpeed = (HAL_ADC_GetValue(PIN_FRONT_RIGHT_WHEEL) + HAL_ADC_GetValue(PIN_FRONT_LEFT_WHEEL)) / 2;
  float currentMPH = averageWheelSpeed * WHEEL_DIAMETER * (3.14159) * 60 / 63360; // pi and inches in a mile

  // get battery percentage
  float batteryPercentage = (emusVoltage / MAX_PACK_VOLTAGE) * 100;

  // get coast regen and brake regen values
  float coastRegen = HAL_ADC_GetValue(PIN_COAST_REGEN);
  float brakeRegen = HAL_ADC_GetValue(PIN_BRAKE_REGEN);

  // clear display
  lcd_clear();

  // drive direction
  lcd_put_cur(1, 0);                        // position of drive direction
  if (direction) lcd_send_string("FWD");    // print drive direction
  else lcd_send_string("BCK");

  // battery percentage
  lcd_put_cur(12, 0);                       // set cursor for battery percentage value
  lcd_send_data((int)batteryPercentage);    // print the battery percentage value
  lcd_put_cur(15, 0);                       // set cursor for % sign
  lcd_send_string("%%");                    // print % sign

  // speedometer
  lcd_put_cur(6, 1);                        // set cursor for mph value
  lcd_send_data((int)currentMPH);           // print the current speed in MPH, cast to int to round to whole number
  lcd_put_cur(9, 1);                        // set cursor for units
  lcd_send_string("mph");                   // print units

  // coast regen
  lcd_put_cur(1, 2);                        // set cursor for CR
  lcd_send_string("CR:");                   // print CR for coast regen
  lcd_put_cur(4, 2);                        // set cursor for coast regen value
  lcd_send_data((int)coastRegen);    				// print coast regen value
  lcd_put_cur(6, 2);                        // set cursor for percent sign
  lcd_send_string("%%");                    // print percent sign

  // brake regen
  lcd_put_cur(10, 2);                       // set cursor for BR
  lcd_send_string("BR:");                   // print BR for brake regen
  lcd_put_cur(12, 2);                       // set cursor for brake regen value
  lcd_send_data((int)brakeRegen);    				// print brake regen value
  lcd_put_cur(14, 2);                       // set cursor for percent sign
  lcd_send_string("%%");                    // print percent sign
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
  lcd_clear();

  // battery percentage
  lcd_put_cur(1, 0);                                        // set cursor for battery percentage value
  lcd_send_data(batteryPercentage);                         // print the current battery percentage value
  lcd_put_cur(4, 0);                                        // set cursor for % sign
  lcd_send_string("%%");                                    // print % sign

  // bus voltage
  lcd_put_cur(1, 1);                                        // set cursor for battery percentage value
  lcd_send_data(emusVoltage);                               // print the emus voltage value
  lcd_put_cur(4, 0);                                        // set cursor for units
  lcd_send_string("V");                                     // print units

  // rinehart voltage
  lcd_put_cur(12, 0);                                       // set cursor for rinehart voltage value
  lcd_send_data(rinehartVoltage);                           // print the rinehart voltage value
  lcd_put_cur(15, 0);                                       // set cursor for units
  lcd_send_string("V");                                     // print % sign

  // power mode
  lcd_put_cur(1, 2);                                        // set cursor for mode text
  lcd_send_string("Mode:");                                 // print mode text
  lcd_put_cur(8, 2);                                        // set cursor current mode setting
  if (powerMode == TUTORIAL) lcd_send_string("Tutorial");
  if (powerMode == ECO) lcd_send_string("Eco");
  if (powerMode == EXPERT) lcd_send_string("Expert");
}


/**
 * @brief ride height, wheel rpm, coast regen, brake regen
 *
 */
void rideSettings()
{
  // clear screen
  lcd_clear();

  // get suspension values
  rideHeightFR = HAL_ADC_GetValue(PIN_FRONT_RIGHT_SUSPENSION);
  rideHeightFL = HAL_ADC_GetValue(PIN_FRONT_LEFT_SUSPENSION);

  // get coast regen and brake regen values
  float coastRegen = HAL_ADC_GetValue(PIN_COAST_REGEN);
  float brakeRegen = HAL_ADC_GetValue(PIN_BRAKE_REGEN);

  // not sure what to do for suspension values yet so
  lcd_put_cur(6, 1);                    // ride height percentage text
  lcd_send_string("Ride %%");           // print text

  lcd_put_cur(1, 0);                    // set cursor for wheel speed value
  lcd_send_data((int)rideHeightFL);     // print front left ride height value

  lcd_put_cur(3, 0);                    // set cursor for "-"
  lcd_send_string("-");                 // print the "-"

  lcd_put_cur(5, 0);                    // set cursor for wheelspeed value
  lcd_send_data((int)rideHeightFR);     // print front right ride height value

  lcd_put_cur(1, 2);                    // set cursor for wheelspeed value
  lcd_send_data((int)rideHeightBL);     // print back left ride height value

  lcd_put_cur(3, 2);                    // set cursor for "-"
  lcd_send_string("-");                 // print the "-"

  lcd_put_cur(5, 2);                    // set cursor for wheelspeed value
  lcd_send_data((int)rideHeightBR);     // print back right ride height value


  // wheel speed
  // get the current wheel speeds
  float wheelSpeedFR = HAL_ADC_GetValue(PIN_FRONT_RIGHT_WHEEL);
  float wheelSpeedFL = HAL_ADC_GetValue(PIN_FRONT_LEFT_WHEEL);

  lcd_put_cur(10, 1);                   // set cursor for RPM text
  lcd_send_string("RPM");               // print RPM text

  lcd_put_cur(10, 0);                   // set cursor for wheel speed value
  lcd_send_data((int)wheelSpeedFR);     // print value for front left

  lcd_put_cur(12, 0);                   // set cursor for "-"
  lcd_send_string("-");                 // print the "-"

  lcd_put_cur(14, 0);                   // set cursor for wheelspeed value
  lcd_send_data((int)wheelSpeedFL);     // print value for front right

  lcd_put_cur(10, 2);                   // set cursor for wheelspeed value
  lcd_send_data((int)wheelSpeedBL);     // print value rear left

  lcd_put_cur(12, 2);                   // set cursor for "-"
  lcd_send_string("-");                 // print the "-"

  lcd_put_cur(14, 2);                   // set cursor for wheelspeed value
  lcd_send_data((int)wheelSpeedBR);     // print value for rear right

  // coast regen
  lcd_put_cur(7, 0);                    // set cursor for CR text
  lcd_send_string("CR:");               // print "CR:" for coast regen
  lcd_send_data((int)coastRegen);       // print coast regen value
  lcd_put_cur(9, 0);                    // set cursor for "%"
  lcd_send_string("%%");                // print "%"

  // brake regen
  lcd_put_cur(7, 2);                    // set cursor for BR text
  lcd_send_string("BR:");               // print "BR:" for brake regen
  lcd_send_data((int)brakeRegen);       // print brake regen value
  lcd_put_cur(9, 2);                    // set cursor for "%"
  lcd_send_string("%%");                // print "%"
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
