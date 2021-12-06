/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define XMODEM_SOH 0x01
#define XMODEM_STX 0x02
#define XMODEM_ETX 0x03
#define XMODEM_EOT 0x04
#define XMODEM_ACK 0x06
#define XMODEM_NAK 0x15
#define XMODEM_CAN 0x18
#define XMODEM_SUB 0x1A

#define XMODEM_DATA_LENGTH 128

#define APPLICATION_START_ADDRESS  0x0800C000 //(FLASH_BASE+ 48*1024)
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)( void );
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
uint32_t buffer[2000];
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
typedef enum
{
	IDLE = 0, SOH_RECEIVED, PN1_RECEIVED, PN2_RECEIVED, DATA_RECEIVED,
} XMODEM_RX_STATES;
XMODEM_RX_STATES xModemState = IDLE;
uint8_t checksum;
uint8_t packetNumber;
uint16_t dataIndex;
uint8_t dataBuffer[XMODEM_DATA_LENGTH];
uint32_t bootloaderDelay = 3000;
uint8_t firstFrame = 1;
uint32_t currentProgramAddress;
uint8_t txData[1];
uint8_t rxData[1];
/**
 * @brief  The application entry point.
 * @retval int
 */

void runApplication();
void EraseApplicationArea( void );
void ProgramAplicationAreaBlock(uint32_t address, uint32_t * data, uint32_t count );
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
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart4, rxData, 1);

	HAL_Delay(100);
	txData[0] = XMODEM_NAK;
	HAL_UART_Transmit(&huart4, txData, 1, 100);
	/* USER CODE END 2 */
	uint32_t _t = 0;
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		if(HAL_GetTick() - _t > 10)
		{
			bootloaderDelay -= 10;
			_t = HAL_GetTick();
		}

		if(bootloaderDelay < 10)
		{
			runApplication();
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void EraseApplicationArea( void )
{
    //unlock flash
    HAL_FLASH_Unlock();

    FLASH_Erase_Sector( FLASH_SECTOR_3, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_4, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_5, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_6, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_7, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_8, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_9, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_10, VOLTAGE_RANGE_3 );
    FLASH_Erase_Sector( FLASH_SECTOR_11, VOLTAGE_RANGE_3 );

    //lock flash
    HAL_FLASH_Lock();
}

void runApplication()
{

	pFunction Jump_To_Application = *( (pFunction*) ( APPLICATION_START_ADDRESS + 4 ) );

	__set_CONTROL( 0 );

	SysTick->CTRL = 0; //disable SysTick

	SCB->VTOR = APPLICATION_START_ADDRESS;

	/* Initialize user application's Stack Pointer */
	__set_MSP( *( (uint32_t*) APPLICATION_START_ADDRESS ) );

	Jump_To_Application();
}

void ProgramAplicationAreaBlock(uint32_t address, uint32_t * data, uint32_t count ){
	uint32_t i;

	if(address<APPLICATION_START_ADDRESS) return;

	//unlock flash
	HAL_FLASH_Unlock();

	for(i=0;i<count;i++){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *data++);
		address+=sizeof(uint32_t);
	}
	//lock flash
	HAL_FLASH_Lock();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	HAL_UART_Receive_IT(huart, rxData, 1);
	switch( xModemState )
	{
	case IDLE :
		if( XMODEM_SOH == rxData[0] )
		{
			xModemState = SOH_RECEIVED;
		}
		else if( XMODEM_EOT == rxData[0] )
		{
			//File received
			txData[0] = XMODEM_ACK;
			HAL_UART_Transmit(huart, txData, 1, 100);
			//run application after 100ms
			bootloaderDelay = 100;
		}
		break;
	case SOH_RECEIVED :
		packetNumber = rxData[0];
		xModemState = PN1_RECEIVED;
		break;
	case PN1_RECEIVED :
		if( ( 255 - packetNumber ) == rxData[0] )
		{
			xModemState = PN2_RECEIVED;
			dataIndex = 0;
			checksum = 0;
		}
		else
		{
			//ToDo: send XMODEM_CAN response
		}
		break;
	case PN2_RECEIVED :
		dataBuffer[dataIndex++] = rxData[0];
		checksum += rxData[0];
		if( dataIndex >= XMODEM_DATA_LENGTH )
		{
			xModemState = DATA_RECEIVED;
		}
		break;
	case DATA_RECEIVED :
		if( checksum == rxData[0] )
		{
			//frame received successfully
			if( firstFrame )
			{
				EraseApplicationArea();
				firstFrame = 0;
				currentProgramAddress = APPLICATION_START_ADDRESS;
			}

			//ToDo: write to flash
			ProgramAplicationAreaBlock( currentProgramAddress, (uint32_t *)dataBuffer, sizeof(dataBuffer)/sizeof(uint32_t) );

			currentProgramAddress += sizeof(dataBuffer);

			txData[0] = XMODEM_ACK;
			HAL_UART_Transmit(huart, txData, 1, 100);

			bootloaderDelay = 1000;
		}
		else
		{
			txData[0] = XMODEM_NAK;
			HAL_UART_Transmit(huart, txData, 1, 100);
		}
		xModemState = IDLE;
		break;
	}
	/* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
	 */
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

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
