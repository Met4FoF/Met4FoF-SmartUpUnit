/*
 * freertos_cubemx.h
 *
 *  Created on: 23.05.2019
 *      Author: seeger01
 */

#ifndef FREERTOS_CUBEMX_H_
#define FREERTOS_CUBEMX_H_
#include "httpserver-netconn.hpp"
#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//LCD
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"

//network
#include "lwip/api.h"
//#include "lwip/udp.h"
#include "lwip.h"
#include "SEGGER_RTT.h"
#include "tim64extender.h"
#define VERSION_MAJOR 0
#define VERSION_MINOR 7
#define VERSION_PATCH 2
const uint8_t UDP_TARGET_DEFAULT_IP_ADDRESS[4] = { 192, 168, 0, 200 };


#define NMEABUFFERSIZE 3
#define NMEBUFFERLEN 900
#define NMEAMINLEN 9
#define MAXNEMASENTENCECOUNT NMEBUFFERLEN/NMEAMINLEN


typedef struct {
	uint64_t RawTimerCount;
	uint32_t CaptureCount;
	uint8_t NMEAMessage[NMEBUFFERLEN]; //248 3 NMEA Sentences
	HAL_StatusTypeDef GPSUARTDMA_START_result; // result of the DMA start call
}NMEASTamped;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATAMAILBUFFERSIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE END Variables */
extern osThreadId IOTID;
extern osThreadId blinkTID;
extern osThreadId WebServerTID;
extern osThreadId LCDTID;
extern osThreadId DataStreamerTID;
extern osThreadId TempSensorTID;
extern osThreadId NmeaParserTID;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartWebserverThread(void const * argument);
extern void StartBlinkThread(void const * argument);
void StartLCDThread(void const * argument);
void StartDataStreamerThread(void const * argument);
void StartTempSensorThread(void const * argument);
void StartNmeaParserThread(void const * argument);
extern void MX_LWIP_Init(void);
extern void MX_FATFS_Init(void);

void MX_FREERTOS_Init(void);

void NTP_time_CNT_update(time_t t,uint32_t us);

/* (MISRA C 2004 rule 8.1) */
#ifdef __cplusplus
}
#endif
#endif /* FREERTOS_CUBEMX_H_ */
