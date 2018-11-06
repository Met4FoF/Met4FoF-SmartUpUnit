/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "httpserver-netconn.h"
#include "bma280.h"

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
//test only
#include "lwip/udp.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

osThreadId WebServerTID;
osThreadId blinkTID;
osThreadId DataProcessingTID;
osThreadId DataStreamingTID;

BMA280 Acc(GPIOG, SPI3_CS_Pin, &hspi3);

AccelDataStamped ACCData;

//MemPool For the data
osPoolDef(AccPool, ACCBUFFESEIZE, AccelDataStamped);
osPoolId AccPool;

//MessageQ for the time Stamped data
osMessageQDef(ACCMsgBuffer, ACCBUFFESEIZE, uint32_t);
osMessageQId ACCMsgBuffer;

/* USER CODE END PV */
#ifdef __cplusplus

extern "C" {

#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartWebserverThread(void const * argument);
void StartBlinkThread(void const * argument);
void StartDataProcessingThread(void const * argument);
void StartDataStreamingThread(void const * argument);
float getGVal(int index);
float getBMATemp();
#ifdef __cplusplus

}

#endif
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* Enable I-Cache-------------------------------------------------------------*/
	SCB_EnableICache();

	/* MCU Configuration----------------------------------------------------------*/

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART3_UART_Init();
	//MX_USB_OTG_FS_PCD_Init();
	MX_ADC1_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	Acc.init(AFS_2G, BW_1000Hz, normal_Mode, sleep_0_5ms);
	/* USER CODE BEGIN 2 */
	//create the defined Buffer and Pool for ACC data
	AccPool = osPoolCreate(osPool(AccPool));
	ACCMsgBuffer = osMessageCreate(osMessageQ(ACCMsgBuffer), NULL);
	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			128);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(blinkThread, StartBlinkThread, osPriorityLow, 0, 16);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(DataProcessingThread, StartDataProcessingThread, osPriorityHigh,
			0, 256);
	DataProcessingTID = osThreadCreate(osThread(DataProcessingThread), NULL);

	osThreadDef(DataStreamingThread, StartDataStreamingThread, osPriorityNormal,
			0, 2048);
	DataStreamingTID = osThreadCreate(osThread(DataStreamingThread), NULL);
	/* USER CODE END 2 */
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
	}
	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3
			| RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* StartDefaultTask function */
void StartWebserverThread(void const * argument) {
	/* init code for LWIP */
	MX_LWIP_Init();

	http_server_netconn_init();
	/* Infinite loop */
	for (;;) {
		osThreadTerminate(NULL);
	}
}

void StartBlinkThread(void const * argument) {
	while (1) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		osDelay(100);
	}
	osThreadTerminate(NULL);
}

void StartDataProcessingThread(void const * argument) {
//	static uint32_t porcessedCount = 0;
//	osEvent evt;
//	AccelDataStamped *rptr;
	while (1) {
//		evt = osMessageGet(ACCMsgBuffer, osWaitForever);
//		if (evt.status == osEventMessage) {
//			rptr = (AccelDataStamped*) evt.value.p;
//			ACCData = *rptr;
//			osPoolFree(AccPool, rptr);
//			porcessedCount++;
			osDelay(100);
		}

	osThreadTerminate(NULL);
}

void StartDataStreamingThread(void const * argument) {
	static uint32_t porcessedCount = 0;
	osEvent evt;
	AccelDataStamped *rptr;
	struct netconn *conn;
	struct netbuf *buf;
	ip_addr_t targetipaddr;
	char text[sizeof(ACCData)] = "";
	uint8_t IP_ADDRESS[4];
	IP_ADDRESS[0] = 192;
	IP_ADDRESS[1] = 168;
	IP_ADDRESS[2] = 0;
	IP_ADDRESS[3] = 1;
	IP4_ADDR(&targetipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2],
			IP_ADDRESS[3]);
	/* create a new connection */
	conn = netconn_new(NETCONN_UDP);

	/* connect the connection to the remote host */
	netconn_connect(conn, &targetipaddr, 7000);

	/* create a new netbuf */
	buf = netbuf_new();
	while (1) {
		evt = osMessageGet(ACCMsgBuffer, osWaitForever);
		if (evt.status == osEventMessage) {
		rptr = (AccelDataStamped*) evt.value.p;
		ACCData = *rptr;
		porcessedCount++;
			/* reference the data into the netbuf */
			netbuf_ref(buf, &*rptr, sizeof(ACCData));

			/* send the text */
			netconn_send(conn, buf);
			osPoolFree(AccPool, rptr);
		}
	}
	osThreadTerminate(NULL);
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
	static uint32_t captureCount = 0;
	static uint32_t MissedCount = 0;
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		AccelDataStamped *mptr;
		// ATENTION!! if buffer is full the allocation function is blocking aprox 60µs
		mptr = (AccelDataStamped *) osPoolAlloc(AccPool);
		if (mptr != NULL) {
			*mptr = Acc.GetStampedData(0x00000000,
					HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1),
					captureCount);
			//put dater pointer into MSGQ
			osStatus result = osMessagePut(ACCMsgBuffer, (uint32_t) mptr,
					osWaitForever);
		} else {
			MissedCount++;
		}
		captureCount++;
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		//osMessagePut(ACCBuffer,(uint32_t)mptr,osWaitForever);
	}
}

float getGVal(int index) {
	switch (index) {
	case 0:
		return ACCData.Data.x;
	case 1:
		return ACCData.Data.y;
	case 2:
		return ACCData.Data.z;
	default:
		int nan = 0x7F800001;
		return *(float*) &nan;
	}

}

float getBMATemp() {
	return ACCData.Data.temperature;
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
