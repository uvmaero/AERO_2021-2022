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
#define PIN_START_BUTTON;             28    // ready to drive button
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
#define PIN_RTD_BUZZER					      00		// THIS IS NOT CORRECT, JUST WASN'T LISTED IN DOC
#define BUZZER_PERIOD					        2000	// time the buzzer is supposed to be on in milliseconds

// CAN
#define PIN_CAN_PLUS					        32		// positive CAN wire
#define PIN_CAN_MINUS					        33		// negative CAN wire

// precharge
#define PRECHARGE_COEFFICIENT			    0.9		// 90% complete with precharge so it's probably safe to continue

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

// inputs
uint16_t coastRegen, brakeRegen;			// coast and brake regen values 
bool cooling, RTDButton, direction;		// cooling toggle, RTD button state, drive direction
uint8_t coastMap, brakeMap;					  // maps for coast and brake regen

// rinehart & emus
uint16_t rinehartVoltage = 0;				  // voltage in rinehart
uint16_t emusVoltage = 265.0;				  // emus bus voltage

// precharge
bool readyToDrive = false;					  // car is ready to drive
bool RTDLED = false;						      // indicator LED in start button
bool buzzer = false;						      // buzzer is buzzing state
uint16_t timeSinceBuzzerStart = 0;		// counter to time buzzer buzz
bool prechargeStateEnter = false;			// allowed to enter precharge

enum prechargeState
{
	PRECHARGE_OFF,
	PRECHARGE_ON,
	PRECHARGE_DONE,
	PRECHARGE_ERROR
};
prechargeState = PRECHARGE_OFF;				// set intial precharge state to OFF


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */
void prechargeControl();
void RTDButtonChange(); 

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // precharge loop omg no way i used a do-while loop insane programming skills XD
    do
    {
      prechargeControl();
    } while (prechargeState != PRECHARGE_DONE);


    // update display
    

    // read CAN messages


    // send CAN messages

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

}

/* USER CODE BEGIN 4 */

// *** functions *** //

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
			readyToDrive = false;
      // move to PRECHARGE_ON due to this specific condition that doesn't exist yet
      // write that^ specific condition here
		break;

		case (PRECHARGE_ON):
		  // ensure voltages are above correct values
			if (rinehartVoltage >= (emusVoltage * 0.9))
			{
				// turn on ready to drive light
        HAL_GPIO_WritePin(GPIOB, PIN_RTD_LED, GPIO_PIN_SET);
        
				// move to precharge done state
				prechargeState = PRECHARGE_DONE;
			}
		break;

		case (PRECHARGE_DONE):
      // turn off the RTD Button LED
      HAL_GPIO_WritePin(GPIOB, PIN_RTD_LED, GPIO_PIN_RESET);

			// now that precharge is complete we can drive the car
			readyToDrive = true;
		break;

		case (PRECHARGE_ERROR):
			// the car is most definitly not ready to drive
			// requires hard reboot of systems to clear this state
			readyToDrive = false;

      // flash the RTD button LED to indicate we are in PRECHARGE_ERROR
      while (true)
      {
        HAL_GPIO_TogglePin(GPIOB, PIN_RTD_LED);
        HAL_Delay(50);
      }
		break;

		default:
      // fallback state, this indicates we did some undefined action that brought us here
      // we will move to PRECHARGE_ERROR to ensure readyToDrive stays false :)
			prechargeState = PRECHARGE_ERROR;
		break;
	}
}

/**
 * @brief 
 * 
 */
void RTDButtonChange()
{
  // if the precharge state is done and the button is being depressed
	if (prechargeState == PRECHARGE_DONE && RTDLED)
	{
    // turn off the indicator button in the RTD button
		HAL_GPIO_WritePin(GPIOB, PIN_RTD_LED, GPIO_PIN_SET);
    
		buzzer = true;				    // turn on the buzzer
		timeSinceBuzzerStart = 0;	// reset buzzer timer
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
