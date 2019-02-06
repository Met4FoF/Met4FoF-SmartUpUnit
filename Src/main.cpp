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
// STM32 Hardware drivers
#include "dma.h"
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "lwip.h"
#include "spi.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

//Networkinterface and Webserver
#include "lwip/api.h"
#include "lwip/udp.h"
#include "lwip.h"
#include "httpserver-netconn.h"

#define USE_L3GD20 1
#define USE_BMA280 0
// Sensors
//#include "ADXL345.h"
#include "bma280.h"
// FOR MPU6050
//#include "I2Cdev.h"
//#include "MPU6050.h"

//FOR L3GD20
#if USE_L3GD20
#include "L3GD20.h"
#endif
//LCD
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"

//GPS Time Snyc
#include "GPSTimesyn.h"
#include "NMEAPraser.h"

osThreadId WebServerTID;
osThreadId blinkTID;
osThreadId DataProcessingTID;
osThreadId DataStreamingTID;
osThreadId LCDTID;


//I2Cdev I2CdevIface(&hi2c1);
//MPU6050 MPU6050Acc(I2CdevIface);

#if USE_L3GD20
L3GD20 Gyro(GPIOG, SPI3_CS_Pin, &hspi3);
osPoolDef(GyroPool, DATABUFFESEIZE, GyroDataStamped);
osPoolId GyroPool;

//MessageQ for the time Stamped data
osMessageQDef(GyroMsgBuffer, DATABUFFESEIZE, uint32_t);
osMessageQId GyroMsgBuffer;
#endif

//TODO update website
AccelDataStamped ACCData;
#if USE_BMA280

BMA280 Acc(GPIOG, SPI3_CS_Pin, &hspi3);
//MemPool For the data
osPoolDef(AccPool, DATABUFFESEIZE, AccelDataStamped);
osPoolId AccPool;

//MessageQ for the time Stamped data
osMessageQDef(ACCMsgBuffer, DATABUFFESEIZE, uint32_t);
osMessageQId ACCMsgBuffer;
#endif


//MessageQ for the GPS PPS Timestamps
osMessageQDef(GPSTimeBuffer, GPSBUFFERSIZE, uint32_t);
osMessageQId GPSTimeBuffer;

//MessageQ for the Refclock  PPS Timestamps
osMessageQDef(RefClockTimeBuffer, GPSBUFFERSIZE, uint32_t);
osMessageQId RefClockTimeBuffer;

// Network interface Ip
uint8_t ETH_IP_ADDRESS[4]={192,168,0,10};
// Target IP for udp straming
uint8_t UDP_TARGET_IP_ADDRESS[4]={192,168,0,1};

#ifdef __cplusplus

extern "C" {

#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartWebserverThread(void const * argument);
void StartBlinkThread(void const * argument);
void StartDataProcessingThread(void const * argument);
void StartDataStreamingThread(void const * argument);
void StartLCDThread(void const * argument);
void _Error_Handler(char * file, int line);
//TODO remove this getter functions and implement it in the dataprocessing
float getGVal(int index);
float getBMATemp();
#ifdef __cplusplus

}

#endif

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

	/* Enable D-Cache-------------------------------------------------------------*/
	SCB_EnableDCache();
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_DMA_Init();
	//MX_USB_OTG_FS_PCD_Init();
	MX_ADC1_Init();
	MX_SPI3_Init();
	MX_SPI5_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	#if USE_BMA280
	Acc.init(AFS_2G, BW_1000Hz, normal_Mode, sleep_0_5ms);
	#endif
	#if USE_L3GD20
	Gyro.init(GYRO_RANGE_2000DPS,GYRO_UPDATE_200_HZ);
	#endif
	//MPU6050Acc.initialize();
	// ADXL345
	//Go into standby mode to configure the device.
	//Acc.setPowerControl(0x00);
	//Acc.setResolution(ADXL345_AFS_FULL_RANGE);
	//Acc.setDataRate(ADXL345_3200HZ);
	//Activate DataRdy Interrupt
	//Acc.setInterruptEnableControl(0x80);

	//Acc.setInterruptMappingControl(0x00);
	//Measurement mode.
	//Acc.setPowerControl(0x08);
	// ADXL345


	/* USER CODE BEGIN 2 */
	//create the defined Buffer and Pool for ACC and GPS data
	#if USE_BMA280
	AccPool = osPoolCreate(osPool(AccPool));
	ACCMsgBuffer = osMessageCreate(osMessageQ(ACCMsgBuffer), NULL);
	#endif
	#if USE_L3GD20
	GyroPool = osPoolCreate(osPool(GyroPool));
	GyroMsgBuffer = osMessageCreate(osMessageQ(GyroMsgBuffer), NULL);
	#endif
	GPSTimeBuffer = osMessageCreate(osMessageQ(GPSTimeBuffer), NULL);

	RefClockTimeBuffer = osMessageCreate(osMessageQ(RefClockTimeBuffer), NULL);

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			128);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(blinkThread, StartBlinkThread, osPriorityLow, 0, 16);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(DataProcessingThread, StartDataProcessingThread, osPriorityNormal,
			0, 256);
	DataProcessingTID = osThreadCreate(osThread(DataProcessingThread), NULL);

	osThreadDef(DataStreamingThread, StartDataStreamingThread, osPriorityNormal,
			0, 2048);
	DataStreamingTID = osThreadCreate(osThread(DataStreamingThread), NULL);

	osThreadDef(LCDThread, StartLCDThread, osPriorityNormal,
			0, 256);

	LCDTID = osThreadCreate(osThread(LCDThread), NULL);
	/* USER CODE END 2 */

	//Start timer and arm inputcapture
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		/* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK) {
		/* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
	}
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK) {
		/* Starting Error */
		_Error_Handler(__FILE__, __LINE__);
	}
	initGPSTimesny();

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 400;
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

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
	 while (1) {
		osDelay(1000);
	}
	osThreadTerminate(NULL);
}

void  StartLCDThread(void const * argument) {
	ILI9341_Init();//initial driver setup to drive ili9341
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	char Temp_Buffer_text[40];
	ILI9341_Draw_Text("Met4FoF SmartUpUnit", 0, 0, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text,"Build.date:%s",__DATE__);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 20, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text,"Build.time:%s",__TIME__);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 40, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text, "IP      :%d.%d.%d.%d", ETH_IP_ADDRESS[0],ETH_IP_ADDRESS[1],ETH_IP_ADDRESS[2],ETH_IP_ADDRESS[3]);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text, "UPD Targ:%d.%d.%d.%d", UDP_TARGET_IP_ADDRESS[0],UDP_TARGET_IP_ADDRESS[1],UDP_TARGET_IP_ADDRESS[2],UDP_TARGET_IP_ADDRESS[3]);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
	//----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
	while (1) {
		osDelay(1000);
		timespec utc;
		timespec gps_time;
		lgw_gps_get(&utc,&gps_time, NULL, NULL);
		tm* current_time = localtime(&(utc.tv_sec));
		strftime(Temp_Buffer_text, 20, "%Y-%m-%d %H:%M:%S",current_time);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 100, WHITE, 2, BLUE);
	}
	osThreadTerminate(NULL);
}

void StartDataStreamingThread(void const * argument) {
	static uint32_t porcessedCount = 0;
	osEvent evt;
	osEvent evtGPS;
	osEvent evtRefClock;
	struct netconn *conn;
	struct netbuf *buf;
	//UDP target ip Adress
	ip_addr_t targetipaddr;
	IP4_ADDR(&targetipaddr, UDP_TARGET_IP_ADDRESS[0], UDP_TARGET_IP_ADDRESS[1], UDP_TARGET_IP_ADDRESS[2],
			UDP_TARGET_IP_ADDRESS[3]);
	/* create a new connection */
	conn = netconn_new(NETCONN_UDP);

	/* connect the connection to the remote host */
	netconn_connect(conn, &targetipaddr, 7000);

	/* create a new netbuf */
	buf = netbuf_new();
	while (1){
		#if USE_BMA280
		AccelDataStamped *rptr;
		//Delay =200 ms so the other routine is processed with 5 Hz >>1 Hz GPS PPS
		evt = osMessageGet(ACCMsgBuffer,200);
		struct timespec utc;
		if (evt.status == osEventMessage) {
			rptr = (AccelDataStamped*) evt.value.p;
			ACCData = *rptr;
			osMutexWait(GPS_ref_mutex_id, osWaitForever);
			lgw_cnt2utc(GPS_ref,rptr->RawTimerCount,&utc);
			rptr->UnixSecs=(uint32_t)(utc.tv_sec);
			rptr->NanoSecs=(uint32_t)(utc.tv_nsec);
			osMutexRelease(GPS_ref_mutex_id);
			porcessedCount++;
			uint8_t MSGBuffer[sizeof(ACCData)+4]={0};
			MSGBuffer[0]=0x41;
			MSGBuffer[1]=0x43;
			MSGBuffer[2]=0x43;
			MSGBuffer[3]=0x33;
			memcpy(&MSGBuffer[4],&*rptr, sizeof(ACCData));
			/* reference the data into the netbuf */
			netbuf_ref(buf, &MSGBuffer, sizeof(MSGBuffer));

			/* send the text */
			netconn_send(conn, buf);
			osPoolFree(AccPool, rptr);
		}
		#endif
#if USE_L3GD20
//Delay =200 ms so the other routine is processed with 5 Hz >>1 Hz GPS PPS
evt = osMessageGet(GyroMsgBuffer,200);
struct timespec utc;
if (evt.status == osEventMessage) {
	GyroDataStamped *rptr;
	rptr = (GyroDataStamped*) evt.value.p;
	osMutexWait(GPS_ref_mutex_id, osWaitForever);
	lgw_cnt2utc(GPS_ref,rptr->RawTimerCount,&utc);
	rptr->UnixSecs=(uint32_t)(utc.tv_sec);
	rptr->NanoSecs=(uint32_t)(utc.tv_nsec);
	osMutexRelease(GPS_ref_mutex_id);
	porcessedCount++;
	uint8_t MSGBuffer[sizeof(GyroDataStamped)+4]={0};
	MSGBuffer[0]=0x47;
	MSGBuffer[1]=0x59;
	MSGBuffer[2]=0x52;
	MSGBuffer[3]=0x33;
	memcpy(&MSGBuffer[4],&*rptr, sizeof(GyroDataStamped));
	/* reference the data into the netbuf */
	netbuf_ref(buf, &MSGBuffer, sizeof(MSGBuffer));

	/* send the text */
	netconn_send(conn, buf);
	osPoolFree(GyroPool, rptr);
}
#endif
		evtGPS = osMessageGet(GPSTimeBuffer,0);
		if (evtGPS.status == osEventMessage) {
			uint32_t rptrGPS =  evtGPS.value.v;
			uint8_t MSGBuffer[8]={0};
			MSGBuffer[0]=0x47;
			MSGBuffer[1]=0x50;
			MSGBuffer[2]=0x53;
			MSGBuffer[3]=0x54;
			memcpy(&MSGBuffer[4],&rptrGPS, sizeof(rptrGPS));
			/* reference the data into the netbuf */
			netbuf_ref(buf, &MSGBuffer, sizeof(MSGBuffer));

			/* send the text */
			netconn_send(conn, buf);
		}
		evtRefClock = osMessageGet(RefClockTimeBuffer,0);
		if (evtRefClock.status == osEventMessage) {
			uint32_t rptrRefClock =  evtRefClock .value.v;
			uint8_t MSGBuffer[8]={0};
			MSGBuffer[0]=0x53;
			MSGBuffer[1]=0x59;
			MSGBuffer[2]=0x4E;
			MSGBuffer[3]=0x54;
			memcpy(&MSGBuffer[4],&rptrRefClock , sizeof(rptrRefClock));
			/* reference the data into the netbuf */
			netbuf_ref(buf, &MSGBuffer, sizeof(MSGBuffer));

			/* send the text */
			netconn_send(conn, buf);
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
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
	//GPS testing change this to an que based aproche in the future
	static int32_t GPSMissedCpatureCount = 0;
	static int32_t RefClockMissedCpatureCount = 0;
	static uint32_t captureCount = 0;
	static uint32_t MissedCount = 0;
	static uint32_t GPSEdges=0;
#define GPSDEVIDER 1
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
#if USE_L3GD20
		GyroDataStamped *mptr;
		// ATENTION!! if buffer is full the allocation function is blocking aprox 60µs
		mptr = (GyroDataStamped *) osPoolAlloc(GyroPool);
		if (mptr != NULL) {
			*mptr = Gyro.GetStampedData(0x00000000,
					HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1),
					captureCount);
			//put dater pointer into MSGQ
			osStatus result = osMessagePut(GyroMsgBuffer, (uint32_t) mptr,
			osWaitForever);
		} else {
			MissedCount++;
		}
#endif
#if USE_BMA280
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
#endif
		captureCount++;
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		//osMessagePut(ACCBuffer,(uint32_t)mptr,osWaitForever);
	}
	else if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			osStatus result = osMessagePut(RefClockTimeBuffer,HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3),osWaitForever);
			if (result!=osOK){RefClockMissedCpatureCount++;}
		}

	else if (htim->Instance == TIM2
				&& htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			//pointer needs to be static otherwiese it would be deletet when jumping out of ISR
			static NMEASTamped *mptr_active=NULL;
			static NMEASTamped *mptr_old=NULL;
			static uint32_t GPScaptureCount = 0;
			uint32_t timestamp=0;
			timestamp=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);

			osStatus result = osMessagePut(GPSTimeBuffer,timestamp,osWaitForever);
			if (result != osOK) {
				GPSMissedCpatureCount++;
			}
			GPSEdges++;
			if(GPSEdges%GPSDEVIDER==0){
			if (GPScaptureCount > 0) {
					HAL_UART_DMAStop(&huart2);
					HAL_DMA_Abort(&hdma_usart2_rx);
					mptr_old = mptr_active;
					if(mptr_old!= NULL){
					osStatus result = osMessagePut(NMEABuffer,(uint32_t) mptr_old,osWaitForever);
					}
					mptr_active = (NMEASTamped *) osPoolAlloc(NMEAPool);
					if (mptr_active != NULL) {
						mptr_active->RawTimerCount = timestamp;
						mptr_active->CaptureCount = GPScaptureCount;
						mptr_active->NMEAMessage[sizeof(mptr_active->NMEAMessage)-1]=0;
						HAL_UART_Receive_DMA(&huart2, &(mptr_active->NMEAMessage[0]), sizeof(mptr_active->NMEAMessage)-1);
					}
					GPScaptureCount++;
			}

			else if (GPScaptureCount == 0) {
				mptr_active = (NMEASTamped *) osPoolAlloc(NMEAPool);
				uint32_t debugVar= (uint32_t)mptr_active;
				if (mptr_active != NULL) {
					(mptr_active->RawTimerCount) = timestamp;
					(mptr_active->CaptureCount) = GPScaptureCount;
					mptr_active->NMEAMessage[sizeof(mptr_active->NMEAMessage)-1]=0;
					HAL_UART_Receive_DMA(&huart2, &(mptr_active->NMEAMessage[0]), sizeof(mptr_active->NMEAMessage)-1);
					GPScaptureCount++;
				}
			}


			}
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
