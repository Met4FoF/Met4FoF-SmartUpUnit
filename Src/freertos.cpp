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



#include "MPU9250.h"
#include <math.h>

#include "adc.h"
#include "tim.h"
#include "rng.h"
#include "usart.h"
#include "dma.h"
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
MPU9250 IMU(SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, &hspi1,0);

osMailQDef(DataMail,DATAMAILBUFFERSIZE ,DataMessage);
osMailQId DataMail;
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
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	IOTID = osThreadCreate(osThread(defaultTask), NULL);

	osThreadDef(blinkThread, StartBlinkThread, osPriorityLow, 0, 512);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			512);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(LCDThread, StartLCDThread, osPriorityNormal, 0, 512);

	LCDTID = osThreadCreate(osThread(LCDThread), NULL);

	osThreadDef(DataStreamerThread, StartDataStreamerThread, osPriorityNormal, 0, 1024);

	DataStreamerTID = osThreadCreate(osThread(DataStreamerThread), NULL);

	initGPSTimesny();
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
			osDelay(10000);
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
		sprintf(Temp_Buffer_text, "IP %s",(const char *)&iPadressBuffer);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
		sprintf(Temp_Buffer_text, "UPD Targ:%d.%d.%d.%d", UDP_TARGET_IP_ADDRESS[0],
				UDP_TARGET_IP_ADDRESS[1], UDP_TARGET_IP_ADDRESS[2],
				UDP_TARGET_IP_ADDRESS[3]);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
		static int lcdupdatecnt=0;
		while (1) {
			osDelay(1000);
			lcdupdatecnt++;
			timespec utc;
			timespec gps_time;
			lgw_gps_get(&utc, &gps_time, NULL, NULL);
			tm* current_time = localtime(&(utc.tv_sec));
			strftime(Temp_Buffer_text, 20, "%Y-%m-%d %H:%M:%S", current_time);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 100, WHITE, 2, BLUE);
			if(lcdupdatecnt==10){
			lcdupdatecnt=0;
			ip4addr_ntoa_r(&(gnetif.ip_addr),iPadressBuffer,sizeof(iPadressBuffer));
			iPadressBuffer[17]= {};
			sprintf(Temp_Buffer_text, "IP %s    ",iPadressBuffer);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
			sprintf(Temp_Buffer_text, "UPD Targ:%d.%d.%d.%d", UDP_TARGET_IP_ADDRESS[0],
					UDP_TARGET_IP_ADDRESS[1], UDP_TARGET_IP_ADDRESS[2],
					UDP_TARGET_IP_ADDRESS[3]);
			}
		}
		osThreadTerminate(NULL);
	}


	void StartDataStreamerThread(void const * argument){
		//TODO only for testing! we need a mail Que here
		RandomData RandomID =getRandomData(&hrng);
		IMU.setBaseID(RandomID.asuint16[1]);
		IMU.begin();
		IMU.enableDataReadyInterrupt();
		//TODO add check that the if is up!! if this is not checked vPortRaiseBASEPRI( void ) infinity loop occurs
		osDelay(4000);
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
		static int i=0;
		static uint32_t ID;
		HAL_RNG_GenerateRandomNumber(&hrng,(uint32_t *) ID);
        //defining Protobuff output stream with Maximum Transfer unit (MTU) size of the networkpackages
		#define MTU_SIZE 1000
		uint8_t ProtoBufferData[MTU_SIZE] = { 0 };
		pb_ostream_t ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData, MTU_SIZE);
		uint8_t ProtoBufferDescription[MTU_SIZE] = { 0 };
		pb_ostream_t ProtoStreamDescription = pb_ostream_from_buffer(ProtoBufferDescription, MTU_SIZE);
		DataMail = osMailCreate(osMailQ(DataMail), NULL);
		//Start timer and arm inputcapture
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
		HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	          /* Enable ADC1 external trigger */
		HAL_ADC_Start_IT(&hadc1);


		while (1) {
#define SIMULATIONMODE 0
#if SIMULATIONMODE
			static int i=0;
			RandomData RandomNoise=getRandomData(&hrng);
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
			pb_encode(&ProtoStreamData,DataMessage_fields, &PrtotTestdata);
			//sending the buffer
			netbuf_ref(buf, &ProtoBufferData, ProtoStreamData.bytes_written);
			/* send the text */
			err_t net_conn_result =netconn_send(conn, buf);
			Check_LWIP_RETURN_VAL(net_conn_result);
			// reallocating buffer this is maybe performance intensive profile this
			ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData, MTU_SIZE);
			i++;
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			osDelay(100);
			#endif

#if !SIMULATIONMODE
			DataMessage *Datarptr;
					//Delay =200 ms so the other routine is processed with 5 Hz >>1 Hz GPS PPS
			osEvent DataEvent = osMailGet(DataMail,200);

			struct timespec SampelPointUtc;
			if (DataEvent.status == osEventMail) {
				Datarptr = (DataMessage*) DataEvent.value.p;
				HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
				lgw_cnt2utc(GPS_ref,Datarptr->unix_time_nsecs,&SampelPointUtc);
				Datarptr->unix_time=(uint32_t)(SampelPointUtc.tv_sec);
				Datarptr->unix_time_nsecs=(uint32_t)(SampelPointUtc.tv_nsec);
#endif
#if NULL
				if(ProtoStreamData.bytes_written>(MTU_SIZE-DataMessage_size)){
				//sending the buffer
					netbuf_ref(buf, &ProtoBufferData, ProtoStreamData.bytes_written);
				/* send the text */
					err_t net_conn_result =netconn_send(conn, buf);
					Check_LWIP_RETURN_VAL(net_conn_result);
				// reallocating buffer this is maybe performance intensive profile this
				//TODO profile this code
					ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData, MTU_SIZE);
				}

			pb_encode_ex(&ProtoStreamData,DataMessage_fields,Datarptr,PB_ENCODE_DELIMITED);
#endif
#if !SIMULATIONMODE
			pb_encode(&ProtoStreamData,DataMessage_fields,Datarptr);
			//sending the buffer
			netbuf_ref(buf, &ProtoBufferData, ProtoStreamData.bytes_written);
			/* send the text */
			err_t net_conn_result =netconn_send(conn, buf);
			Check_LWIP_RETURN_VAL(net_conn_result);
			// reallocating buffer this is maybe performance intensive profile this
			//TODO profile this code
			ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData, MTU_SIZE);
			i++;
			osMailFree(DataMail, Datarptr);
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			}
#endif
		}
		osThreadTerminate(NULL);
	}

	void Check_LWIP_RETURN_VAL(err_t retVal)
	{
		static uint32_t LWIP_RRT_PRINT_ErrorCount=0;
					if (retVal!=ERR_OK){
						switch(retVal){
						case -1: SEGGER_RTT_printf(0,"%u LWIP ERR_MEM: Out of memory error.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -2: SEGGER_RTT_printf(0,"%u LWIP ERR_BUF: Buffer error.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -3: SEGGER_RTT_printf(0,"%u LWIP ERR_TIMEOUT: Time Out.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -4: SEGGER_RTT_printf(0,"%u LWIP ERR_RTE: Routing problem.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -5: SEGGER_RTT_printf(0,"%u LWIP ERR_INPROGRESS: Operation in progress.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -6: SEGGER_RTT_printf(0,"%u LWIP ERR_VAL: Illegal value.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -7: SEGGER_RTT_printf(0,"%u LWIP ERR_WOULDBLOCK: Operation would block.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -8: SEGGER_RTT_printf(0,"%u LWIP ERR_USE: Address in use.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -9: SEGGER_RTT_printf(0,"%u LWIP ERR_ALREADY: Already connecting.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -10: SEGGER_RTT_printf(0,"%u LWIP ERR_ISCONN: Conn already established.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -11: SEGGER_RTT_printf(0,"%u LWIP ERR_CONN: Not connected.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -12: SEGGER_RTT_printf(0,"%u LWIP ERR_IF: Low-level netif error.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -13: SEGGER_RTT_printf(0,"%u LWIP ERR_ABRT: Connection aborted.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -14: SEGGER_RTT_printf(0,"%u LWIP ERR_RST: Connection reset.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -15: SEGGER_RTT_printf(0,"%u LWIP ERR_CLSD: Connection closed.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						case -16: SEGGER_RTT_printf(0,"%u LWIP ERR_ARG: Illegal argument.\r\n",LWIP_RRT_PRINT_ErrorCount);LWIP_RRT_PRINT_ErrorCount++;break;
						}
					}
	}

	RandomData getRandomData(RNG_HandleTypeDef *hrng){
		RandomData result={0};
		HAL_RNG_GenerateRandomNumber(hrng,(uint32_t *)result.asuint);
		return result;
	}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		static uint32_t Channel1CaptureCount=0;
		if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			Channel1CaptureCount++;
			DataMessage *mptr;
			mptr = (DataMessage *) osMailAlloc(DataMail,0);
			IMU.getData(mptr,HAL_TIM_ReadCapturedValue(&htim2,
					TIM_CHANNEL_1),Channel1CaptureCount);
			//mptr->has_Data_11=true;
			//mptr->Data_11=(float)HAL_ADC_PollForConversion(&hadc1, 1);
			osStatus result = osMailPut(DataMail, mptr);
		}
		if (htim->Instance == TIM2
					&& htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
				//pointer needs to be static otherwiese it would be deletet when jumping out of ISR
				static NMEASTamped *mptr = NULL;
				static uint32_t GPScaptureCount = 0;
				static uint8_t DMA_NMEABUFFER[NMEBUFFERLEN]={0};
					if (GPScaptureCount > 0) {
		                                //HAL_UART_RxCpltCallback(&huart7);
		                                if(HAL_UART_DMAStop(&huart7)!=HAL_OK)
		                                {
		                                SEGGER_RTT_printf(0,"HAL_UART_DMAStop returned not ok");
		                                }
		                                if(HAL_DMA_Abort(&hdma_uart7_rx)!=HAL_OK)
		                                {
		                                SEGGER_RTT_printf(0,"HAL_UART_DMAStop returned not ok");
		                                }
						mptr = (NMEASTamped *) osMailAlloc(NMEAMail,0);//The parameter millisec must be 0 for using this function in an ISR.
						if (mptr != NULL) {
							mptr->RawTimerCount = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);
							mptr->CaptureCount = GPScaptureCount;
							memcpy(&(mptr->NMEAMessage[0]),&(DMA_NMEABUFFER[0]),NMEBUFFERLEN);
		                                        osStatus result = osMailPut(NMEAMail, mptr);
						}
		                                //SEGGER_RTT_printf(0,"DMA BUFFER:=%s\n",DMA_NMEABUFFER);
		                                memset(DMA_NMEABUFFER, 0, sizeof(DMA_NMEABUFFER));
						HAL_StatusTypeDef DMA_START_result=HAL_UART_Receive_DMA(&huart7,
								DMA_NMEABUFFER,
								NMEBUFFERLEN - 1);
						GPScaptureCount++;
		                                if(DMA_START_result!=HAL_OK)
		                                {
		                                 SEGGER_RTT_printf(0,"DMA start ERROR");
		                                }
					}
					else if (GPScaptureCount == 0) {
						HAL_UART_Receive_DMA(&huart7,&(DMA_NMEABUFFER[0]),NMEBUFFERLEN - 1);
							GPScaptureCount++;
						}
			}
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}

	/* Private application code --------------------------------------------------*/
	/* USER CODE BEGIN Application */

	/* USER CODE END Application */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
