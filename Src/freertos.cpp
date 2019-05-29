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
#include "httpserver-netconn.h"
//LCD
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"
#include "freertos_cubemx.h"

#include "SEGGER_RTT.h"

#include "pb.h"
#include "message.pb.h"
#include "pb_encode.h"

#include "rng.h"

#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId IOTID;
osThreadId blinkTID;
osThreadId WebServerTID;
osThreadId LCDTID;
osThreadId DataStreamerTID;
/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	IOTID = osThreadCreate(osThread(defaultTask), NULL);

	osThreadDef(blinkThread, StartBlinkThread, osPriorityLow, 0, 16);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			128);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(LCDThread, StartLCDThread, osPriorityNormal, 0, 256);

	LCDTID = osThreadCreate(osThread(LCDThread), NULL);

	osThreadDef(DataStreamerThread, StartDataStreamerThread, osPriorityNormal, 0, 1024);

	DataStreamerTID = osThreadCreate(osThread(DataStreamerThread), NULL);
		/* USER CODE BEGIN RTOS_THREADS */
		/* add threads, ... */
		/* USER CODE END RTOS_THREADS */

		/* USER CODE BEGIN RTOS_QUEUES */
		/* add queues, ... */
		/* USER CODE END RTOS_QUEUES */
	}

	/* USER CODE BEGIN Header_StartDefaultTask */
	/**
	 * @brief  Function implementing the defaultTask thread.
	 * @param  argument: Not used
	 * @retval None
	 */
	/* USER CODE END Header_StartDefaultTask */
	void StartDefaultTask(void const * argument)
	{
		/* init code for LWIP */
		MX_LWIP_Init();

		/* init code for FATFS */
		MX_FATFS_Init();

		/* USER CODE BEGIN StartDefaultTask */
		/* Infinite loop */
		for(;;)
		{
			osDelay(1);
		}
		/* USER CODE END StartDefaultTask */
	}

	void StartWebserverThread(void const * argument) {
		// wait until LWIP is inited
		osDelay(5000);
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

	void StartLCDThread(void const * argument) {
		//osDelay(3000);
		/* USER CODE BEGIN Variables */
		//TODO use real ip adress
		//uint8_t ETH_IP_ADDRESS[4] = { 192, 168, 0, 10 };
		// Target IP for udp straming
		//uint8_t UDP_TARGET_IP_ADDRESS[4] = { 192, 168, 0, 1 };
		ILI9341_Init();//initial driver setup to drive ili9341
		ILI9341_Fill_Screen(BLUE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
		char Temp_Buffer_text[40];
		/*
		 ILI9341_Draw_Text(" Wer das liest ist doof.", 0, 60, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Wie dat leest, is dom.", 0, 80, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Who reads that is stupid.", 0, 100, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Celui qui lit ca est stupide.", 0, 120, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Quienquiera que lea eso es estupido.", 0, 140, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Quem le isso e estupido.", 0, 160, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Chiunque lo legga e stupido.", 0, 180, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Ktokolwiek to czyta, jest glupi.", 0, 200, WHITE, 1.5, BLUE);
		 ILI9341_Draw_Text(" Kto by ehto ni chital, ehto glupo.", 0, 220, WHITE, 1.5, BLUE);
		 */
		ILI9341_Draw_Text("Met4FoF SmartUpUnit", 0, 0, WHITE, 2, BLUE);
		sprintf(Temp_Buffer_text, "Build.date:%s", __DATE__);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 20, WHITE, 2, BLUE);
		sprintf(Temp_Buffer_text, "Build.time:%s", __TIME__);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 40, WHITE, 2, BLUE);
		char iPadressBuffer[17]= {};
		ip4addr_ntoa_r(&(gnetif.ip_addr),iPadressBuffer,sizeof(iPadressBuffer));
		sprintf(Temp_Buffer_text, "IP %s",iPadressBuffer);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
		sprintf(Temp_Buffer_text, "UPD Targ:%d.%d.%d.%d", UDP_TARGET_IP_ADDRESS[0],
				UDP_TARGET_IP_ADDRESS[1], UDP_TARGET_IP_ADDRESS[2],
				UDP_TARGET_IP_ADDRESS[3]);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
		while (1) {
			osDelay(1000);
			//timespec utc;
			//timespec gps_time;
			//lgw_gps_get(&utc, &gps_time, NULL, NULL);
			//tm* current_time = localtime(&(utc.tv_sec));
			//strftime(Temp_Buffer_text, 20, "%Y-%m-%d %H:%M:%S", current_time);
			//ILI9341_Draw_Text(Temp_Buffer_text, 0, 100, WHITE, 2, BLUE);
		}
		osThreadTerminate(NULL);
	}


	void StartDataStreamerThread(void const * argument){
		//TODO add check that the if is up!! if this is not checked vPortRaiseBASEPRI( void ) infinity loop occurs
		osDelay(3000);
		struct netconn *conn;
		struct netbuf *buf;
		//UDP target ip Adress
		ip_addr_t targetipaddr;
		IP4_ADDR(&targetipaddr, UDP_TARGET_IP_ADDRESS[0], UDP_TARGET_IP_ADDRESS[1], UDP_TARGET_IP_ADDRESS[2],
				UDP_TARGET_IP_ADDRESS[3]);
		/* create a new connection */
		conn = netconn_new(NETCONN_UDP);

		/* connect the connection to the remote host */
		err_t net_conn_result=netconn_connect(conn, &targetipaddr, 7000);
		Check_LWIP_RETURN_VAL(net_conn_result);
		/* create a new netbuf */
		buf = netbuf_new();
		int i=0;
		static uint32_t ID;
		HAL_RNG_GenerateRandomNumber(&hrng,(uint32_t *) ID);
        //defining Protobuff output stream with Maximum Transfer unit (MTU) size of the networkpackages
		#define MTU_SIZE 500
		uint8_t ProtoBuffer[MTU_SIZE] = { 0 };
		pb_ostream_t ProtoStream = pb_ostream_from_buffer(ProtoBuffer, MTU_SIZE);
		while (1) {
			union Randombytes
			{
			    uint32_t asuint;
			    uint8_t asbyt[sizeof(uint32_t)];
			};
			Randombytes RandomNoise;
			HAL_RNG_GenerateRandomNumber(&hrng,(uint32_t *)RandomNoise.asuint);
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			DataMessage PrtotTestdata;
			PrtotTestdata.id=ID;
			PrtotTestdata.sample_number=i;
			PrtotTestdata.unix_time=0x80000000+i/10;
			PrtotTestdata.unix_time_nsecs=i%10*100000;
			PrtotTestdata.time_uncertainty=0xFFFFFFFF;
			PrtotTestdata.Data_01=cos((float)i/10);
			PrtotTestdata.has_Data_02=true;
			PrtotTestdata.Data_02=cos((float)i/20);
			PrtotTestdata.has_Data_03=true;
			PrtotTestdata.Data_03=cos((float)i/10)+(float)RandomNoise.asbyt[0]/1000;
			PrtotTestdata.has_Data_04=true;
			PrtotTestdata.Data_04=cos((float)i/20)+(float)RandomNoise.asuint/4.294e9-0.5;
			pb_encode(&ProtoStream,DataMessage_fields, &PrtotTestdata);
			//sending the buffer
			netbuf_ref(buf, &ProtoBuffer, ProtoStream.bytes_written);
			/* send the text */
			err_t net_conn_result=netconn_send(conn, buf);
			Check_LWIP_RETURN_VAL(net_conn_result);
			i++;
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			osDelay(100);
		}
		osThreadTerminate(NULL);
	}

	void Check_LWIP_RETURN_VAL(err_t retVal)
	{
					if (retVal!=ERR_OK){
						switch(retVal){
						case -1: SEGGER_RTT_printf(0,"LWIP ERR_MEM: Out of memory error.");break;
						case -2: SEGGER_RTT_printf(0,"LWIP ERR_BUF: Buffer error. ");break;
						case -3: SEGGER_RTT_printf(0,"LWIP ERR_TIMEOUT: Time Out. ");break;
						case -4: SEGGER_RTT_printf(0,"LWIP ERR_RTE: Routing problem. ");break;
						case -5: SEGGER_RTT_printf(0,"LWIP ERR_INPROGRESS: Operation in progress ");break;
						case -6: SEGGER_RTT_printf(0,"LWIP ERR_VAL: Illegal value");break;
						case -7: SEGGER_RTT_printf(0,"LWIP ERR_WOULDBLOCK: Operation would block.");break;
						case -8: SEGGER_RTT_printf(0,"LWIP ERR_USE: Address in use.");break;
						case -9: SEGGER_RTT_printf(0,"LWIP ERR_ALREADY: Already connecting.");break;
						case -10: SEGGER_RTT_printf(0,"LWIP ERR_ISCONN: Conn already established.");break;
						case -11: SEGGER_RTT_printf(0,"LWIP ERR_CONN: Not connected.");break;
						case -12: SEGGER_RTT_printf(0,"LWIP ERR_IF: Low-level netif error.");break;
						case -13: SEGGER_RTT_printf(0,"LWIP ERR_ABRT: Connection aborted.");break;
						case -14: SEGGER_RTT_printf(0,"LWIP ERR_RST: Connection reset.");break;
						case -15: SEGGER_RTT_printf(0,"LWIP ERR_CLSD: Connection closed. ");break;
						case -16: SEGGER_RTT_printf(0,"LWIP ERR_ARG: Illegal argument.");break;
						}
					}
	}
	/* Private application code --------------------------------------------------*/
	/* USER CODE BEGIN Application */

	/* USER CODE END Application */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
