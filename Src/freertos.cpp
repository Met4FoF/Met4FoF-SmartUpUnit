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
//TODO clean up includes
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//LCD
#include "ILI9341/ILI9341_STM32_Driver.h"
#include "ILI9341/ILI9341_GFX.h"
#include "freertos_cubemx.h"

#include "SEGGER_RTT.h"

#include "pb.h"
#include "message.pb.h"
#include "pb_encode.h"

#include "MPU9250.h"
#include "bma280.h"
#include "MS5837.h"
#include "dummy_sensor.h"
#include "bmp280.h"

#include <math.h>

#include "adc.h"
#include "tim.h"
#include "rng.h"
#include "usart.h"
#include "dma.h"
#include "i2c.h"

#include "backupsram.h"

#include "lwip/apps/sntp.h"
#include "configmanager.hpp"
#include "lwip_return_ckeck.h"
//#include "fatfs.h"//fat file System
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
osThreadId TempSensorTID;

//TODO insert sensor manager array in config manager
//DummySensor Sensor0(0);
//DummySensor Sensor1(1);
//BMA280 Sensor0(SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, &hspi1, 0);
MPU9250 Sensor0(SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, &hspi1, 0);
MPU9250 Sensor1(SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, &hspi1, 1);
MS5837 TempSensor0(&hi2c1,MS5837::MS5837_02BA);
BMP280 AirPressSensor(hi2c1);
osMailQDef(DataMail, DATAMAILBUFFERSIZE, DataMessage);
osMailQId DataMail;

bool Lwip_anf_FAT_init_finished=false;

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
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	IOTID = osThreadCreate(osThread(defaultTask), NULL);

	osThreadDef(blinkThread, StartBlinkThread, osPriorityLow, 0, 256);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			512);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(LCDThread, StartLCDThread, osPriorityNormal, 0, 512);

	LCDTID = osThreadCreate(osThread(LCDThread), NULL);

	osThreadDef(DataStreamerThread, StartDataStreamerThread, osPriorityNormal,
			0, 4096);

	DataStreamerTID = osThreadCreate(osThread(DataStreamerThread), NULL);

	osThreadDef(TempSensorThread, StartTempSensorThread, osPriorityNormal, 0, 512);

	TempSensorTID = osThreadCreate(osThread(TempSensorThread), NULL);

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
void StartDefaultTask(void const * argument) {
	ConfigManager& configMan = ConfigManager::instance();


	/* init code for LWIP */
	MX_LWIP_Init();

	/* init code for FATFS */
	//MX_FATFS_Init();

	Lwip_anf_FAT_init_finished=true;
	/* USER CODE BEGIN StartDefaultTask */

	//TODO implent NPTP ip array
	ip_addr_t NTPIP = configMan.getUDPTargetIP();
	osDelay(5000);
	sntp_setserver(0, &NTPIP);
	sntp_init();
	/* Infinite loop */
	for (;;) {
		osDelay(10000);
		sntp_request(NULL);
	}
	/* USER CODE END StartDefaultTask */
}

void StartTempSensorThread(void const * argument) {
	while(! Lwip_anf_FAT_init_finished){
		osDelay(100);
	}
	ConfigManager& configMan = ConfigManager::instance();

	static uint32_t TempsensoreCaptureCount=0;
	uint32_t SensorID3=configMan.getSensorBaseID(3);
	TempSensor0.init(SensorID3);
	for (;;) {
		osDelay(2000);
		DataMessage *mptr;
		mptr = (DataMessage *) osMailAlloc(DataMail, 20);
		uint64_t timestamp = TIM_Get_64Bit_TimeStamp_Base(&htim2);
		struct timespec SampelPointUtc;
		if (xSemaphoreGPS_REF != NULL) {
			// See if we can obtain the semaphore.  If the semaphore is not
			// available wait 10 ticks to see if it becomes free.
			if ( xSemaphoreTake(xSemaphoreGPS_REF,
					(TickType_t ) 10) == pdTRUE) {
				uint32_t tmp_time_uncertainty = 0;
				lgw_cnt2utc(GPS_ref, timestamp, &SampelPointUtc,
						&tmp_time_uncertainty);
				mptr->time_uncertainty = tmp_time_uncertainty;
				xSemaphoreGive(xSemaphoreGPS_REF);
			} else {
				//We could not obtain the semaphore and can therefore not access
				// the shared resource safely.
				SEGGER_RTT_printf(0,
						"cnt to GPS time  UPDATE FAIL SEMAPHORE NOT READY !!!\n\r");
				taskYIELD()
				;
			}
		}
		//int MS5837::getData(DataMessage * Message,uint32_t unix_time,uint32_t unix_time_nsecs,uint32_t time_uncertainty,uint32_t CaptureCount)
		TempSensor0.getData(mptr,(uint32_t)SampelPointUtc.tv_sec,(uint32_t)SampelPointUtc.tv_nsec,40e6,TempsensoreCaptureCount);
		osStatus result = osMailPut(DataMail, mptr);
		TempsensoreCaptureCount++;
		osDelay(10);
		/*
	SEGGER_RTT_printf(0,"Scanning I2C bus:\r\n");

	HAL_StatusTypeDef result;
 	uint8_t i;
 	for (i=1; i<128; i++)
 	{

 	   // the HAL wants a left aligned i2c address
 	   // &hi2c1 is the handle
 	   // (uint16_t)(i<<1) is the i2c address left aligned
 	   // retries 2
 	   // timeout 2
 	   //
 	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
 	  {
 		 SEGGER_RTT_printf(0,"."); // No ACK received at that address
 	  }
 	  if (result == HAL_OK)
 	  {
 		 SEGGER_RTT_printf(0,"0x%X", i); // Received an ACK at that address
 	  }
 	}
 	SEGGER_RTT_printf(0,"\r\n");
		osDelay(1000);
*/

	}
}

void StartWebserverThread(void const * argument) {
	while(! Lwip_anf_FAT_init_finished){
		osDelay(100);
	}
	ConfigManager& configMan = ConfigManager::instance();
	// wait until LWIP is inited
	osDelay(5000);
	SEGGER_RTT_printf(0, "Starting Web Server\r\n");
	http_server_netconn_init();
	/* Infinite loop */
	for (;;) {
		osThreadTerminate(NULL);
	}
}

void StartBlinkThread(void const * argument) {
	osDelay(10000);
	while (1) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		//Sensor0.setAccSelfTest(0x00);//bytemask 0x00000xyz 1=selftest active 0=normal mesurment
		osDelay(1);
		//Sensor0.setGyroSelfTest(0x00);//bytemask 0x00000xyz 1=selftest active 0=normal mesurment
		osDelay(1000);
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		//Sensor0.setGyroSelfTest(0x07);//bytemask 0x00000xyz 1=selftest active 0=normal mesurment
		osDelay(1);
		//Sensor0.setAccSelfTest(0x07);//bytemask 0x00000xyz 1=selftest active 0=normal mesurment
		osDelay(500);
	}
	osThreadTerminate(NULL);
}

void StartLCDThread(void const * argument) {
	while(! Lwip_anf_FAT_init_finished){
		osDelay(100);
	}
	ConfigManager& configMan = ConfigManager::instance();
	osDelay(10);
	ILI9341_Init();		//initial driver setup to drive ili9341
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
	uint16_t BaseID = configMan.getBaseID();
	char Temp_Buffer_text[40];
	ILI9341_Draw_Text("Met4FoF SmartUpUnit", 0, 0, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text, "Rev:%d.%d.%d  ID:%x", VERSION_MAJOR,
			VERSION_MINOR, VERSION_PATCH, BaseID);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 20, WHITE, 2, BLUE);
	sprintf(Temp_Buffer_text, "Build:%s %s", __DATE__, __TIME__);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 40, WHITE, 1, BLUE);
	ip_addr_t UDPTargetIP = configMan.getUDPTargetIP();
	char iPadressBuffer[17] = { };
	ip4addr_ntoa_r(&(UDPTargetIP), iPadressBuffer, sizeof(iPadressBuffer));
	sprintf(Temp_Buffer_text, "UPD Targ:%s", iPadressBuffer);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
	ip4addr_ntoa_r(&(gnetif.ip_addr), iPadressBuffer, sizeof(iPadressBuffer));
	sprintf(Temp_Buffer_text, "IP %s", (const char *) &iPadressBuffer);
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
	static int lcdupdatecnt = 0;
	while (1) {
		osDelay(1000);
		lcdupdatecnt++;
		timespec utc;
		timespec gps_time;
		lgw_gps_get(&utc, &gps_time, NULL, NULL);
		tm* current_time = localtime(&(utc.tv_sec));
		strftime(Temp_Buffer_text, 20, "%Y-%m-%d %H:%M:%S", current_time);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 100, WHITE, 2, BLUE);

		//TODO fix nanospecs printf bug to reactivate --specs=nano.specs -u _printf_float -u _scanf_float to save 50 kb Code size
		sprintf(Temp_Buffer_text, "Counter Freq.: %lf Hz   ", GPS_ref.xtal_err);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 120, WHITE, 1, BLUE);
		sprintf(Temp_Buffer_text, "F std.: %lf Hz      ",
				GPS_ref.xtal_err_deviation);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 140, WHITE, 1, BLUE);
		sprintf(Temp_Buffer_text, "NTP Counter Freq.: %lf Hz   ",
				NTP_ref.xtal_err);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 160, WHITE, 1, BLUE);
		sprintf(Temp_Buffer_text, "NTP F std.: %lf Hz      ",
				NTP_ref.xtal_err_deviation);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 180, WHITE, 1, BLUE);
		tm* ntp_update_time = localtime(&(NTP_ref.utc.tv_sec));
		strftime(Temp_Buffer_text, 40, "NTP Updte: %Y-%m-%d %H:%M:%S",
				ntp_update_time);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 200, WHITE, 1, BLUE);

		sprintf(Temp_Buffer_text, "Counting happy: %i",lcdupdatecnt);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 210, WHITE, 1, BLUE);
		uint32_t startcount=configMan.getStartcount();
		sprintf(Temp_Buffer_text, "Start count: %i",startcount);
		ILI9341_Draw_Text(Temp_Buffer_text, 0, 220, WHITE, 1, BLUE);
		if (lcdupdatecnt %10==0) {
			iPadressBuffer[17]= {};
			ip4addr_ntoa_r(&(gnetif.ip_addr), iPadressBuffer,
					sizeof(iPadressBuffer));
			sprintf(Temp_Buffer_text, "IP %s    ", iPadressBuffer);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
			ip_addr_t UDPTargetIP = configMan.getUDPTargetIP();
			ip4addr_ntoa_r(&(UDPTargetIP), iPadressBuffer,
					sizeof(iPadressBuffer));
			iPadressBuffer[17]= {};
			sprintf(Temp_Buffer_text, "UPD Targ:%s", iPadressBuffer);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
		}
	}
	osThreadTerminate(NULL);
}

void StartDataStreamerThread(void const * argument) {
	while(! Lwip_anf_FAT_init_finished){
		osDelay(100);
	}
	ConfigManager& configMan = ConfigManager::instance();
	configMan.setADCCalCoevs(0, 0.00488040211169927, -10.029208660668372,
			4.6824163159348675e-3);
	configMan.setADCCalCoevs(1, 0.004864769104581888, -9.911472983085314,
			13.68572038605262e-3);
	configMan.setADCCalCoevs(2, 0.004884955868836948, -10.031544601902738,
			4.721804326558252e-3);

	//MPU9250
	uint32_t SensorID1=configMan.getSensorBaseID(1);

	Sensor1.setBaseID(SensorID1);
	Sensor1.begin();
	Sensor1.enableDataReadyInterrupt();

	//MPU9250
	uint32_t SensorID0=configMan.getSensorBaseID(0);

	Sensor0.setBaseID(SensorID0);
	Sensor0.begin();
	Sensor0.enableDataReadyInterrupt();

	//BMA280
	 // SET PS pin low
	/*
	 uint32_t SensorID0=configMan.getSensorBaseID(0);
	 HAL_GPIO_WritePin(GPIO1_2_GPIO_Port, GPIO1_2_Pin, GPIO_PIN_RESET);
	 Sensor0.setBaseID(SensorID0);
	 Sensor0.init(AFS_16G, BW_1000Hz, normal_Mode, sleep_0_5ms);
*/
	//Dummy Sensor
	/*
	 Sensor0.setBaseID(0);
	 Sensor1.setBaseID(1);crc
	 */
	SEGGER_RTT_printf(0,
			"UDID=%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX\n\r",
			UDID_Read8(0), UDID_Read8(1), UDID_Read8(2), UDID_Read8(3),
			UDID_Read8(4), UDID_Read8(5), UDID_Read8(6), UDID_Read8(7),
			UDID_Read8(8), UDID_Read8(9), UDID_Read8(10), UDID_Read8(11));
	osDelay(4000);
	uint32_t StartCount = configMan.getStartcount();
	SEGGER_RTT_printf(0, "StartCount is= %llu", StartCount);
	struct netconn *conn;
	struct netbuf *buf;
	//TODO REMOVE THIS AND INTEGRATE IT in web interface
	configMan.setUDPPort(7654);
	ip_addr_t targetipaddr;
	ip_addr_t settargetipaddr=configMan.getUDPTargetIP();
	if(settargetipaddr.addr==0x00000000)
		{

		uint8_t UDP_TARGET_IP_ADDRESS[4] = { 192, 168, 0, 200 };
		IP4_ADDR(&targetipaddr, UDP_TARGET_IP_ADDRESS[0], UDP_TARGET_IP_ADDRESS[1],
				UDP_TARGET_IP_ADDRESS[2], UDP_TARGET_IP_ADDRESS[3]);
		configMan.setUDPTargetIP(targetipaddr);
		}

	targetipaddr=configMan.getUDPTargetIP();
	/* create a new connection */
	conn = netconn_new(NETCONN_UDP);
	/* connect the connection to the remote host */
	err_t net_conn_result = netconn_connect(conn, &targetipaddr,
			configMan.getUDPPort());
	Check_LWIP_RETURN_VAL(net_conn_result);
	/* create a new netbuf */
	buf = netbuf_new();
	static int i = 0;
	//defining Protobuff output streams with Maximum Transfer unit (MTU) size of the networkpackages
#define MTU_SIZE 1000
	uint8_t ProtoBufferData[MTU_SIZE] = { 0 };
	pb_ostream_t ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData,
	MTU_SIZE);
	uint8_t ProtoBufferDescription[MTU_SIZE] = { 0 };
	pb_ostream_t ProtoStreamDescription = pb_ostream_from_buffer(
			ProtoBufferDescription,
			MTU_SIZE);

	const char DataString[4] = { 68, 65, 84, 65 };	//DATA Keyword
	const char DescriptionString[4] = { 68, 83, 67, 80 };	//DSCP Keyword
	pb_write(&ProtoStreamData, (const pb_byte_t*) &DataString, 4);
	pb_write(&ProtoStreamDescription, (const pb_byte_t*) &DescriptionString, 4);

	DataMail = osMailCreate(osMailQ(DataMail), NULL);
	//Start timer and arm inputcapture
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	/* Enable ADCs external trigger */
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);

	while (1) {
		DataMessage *Datarptr;
		//static uint32_t lastMessageId = 0;
		osEvent DataEvent = osMailGet(DataMail, 200);
		struct timespec SampelPointUtc;
		if (DataEvent.status == osEventMail) {
			Datarptr = (DataMessage*) DataEvent.value.p;
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			if(Datarptr->unix_time==0xFFFFFFFF){
				//calculate the exact time from raw time stamp encoded in Datarptr->time_uncertainty Datarptr->unix_time_nsecs as actual time stamp
				//if Datarptr->unix_time is not 0xFFFFFFFF the calculation has been done before and so this conversion will yield only garbage
			if (xSemaphoreGPS_REF != NULL) {
				/* See if we can obtain the semaphore.  If the semaphore is not
				 available wait 10 ticks to see if it becomes free. */
				if ( xSemaphoreTake(xSemaphoreGPS_REF,
						(TickType_t ) 10) == pdTRUE) {
					uint64_t timestamp = (uint64_t) (Datarptr->time_uncertainty)
							<< 32;
					timestamp += (uint64_t) (Datarptr->unix_time_nsecs);
					uint32_t tmp_time_uncertainty = 0;
					lgw_cnt2utc(GPS_ref, timestamp, &SampelPointUtc,
							&tmp_time_uncertainty);
					Datarptr->time_uncertainty = tmp_time_uncertainty;
					xSemaphoreGive(xSemaphoreGPS_REF);
				} else {
					/* We could not obtain the semaphore and can therefore not access
					 the shared resource safely. */
					SEGGER_RTT_printf(0,
							"cnt to GPS time  UPDATE FAIL SEMAPHORE NOT READY !!!\n\r");
					taskYIELD()
					;
				}
			}
			Datarptr->unix_time = (uint32_t) (SampelPointUtc.tv_sec);
			Datarptr->unix_time_nsecs = (uint32_t) (SampelPointUtc.tv_nsec);
			}
			//if (lastMessageId < Datarptr->sample_number) {
				//this check is just for extra security in normal operation this should never happen
				if (ProtoStreamData.bytes_written
						> (MTU_SIZE - (DataMessage_size))) {
					//sending the buffer
					netbuf_ref(buf, &ProtoBufferData,
							ProtoStreamData.bytes_written);
					/* send the text */
					err_t net_conn_result = netconn_send(conn, buf);
					Check_LWIP_RETURN_VAL(net_conn_result);
					// reallocating buffer this is maybe performance intensive profile this
					//TODO profile this code
					ProtoStreamData = pb_ostream_from_buffer(ProtoBufferData,
					MTU_SIZE);
					pb_write(&ProtoStreamData, (const pb_byte_t*) &DataString,
							4);
				}

				pb_encode_ex(&ProtoStreamData, DataMessage_fields, Datarptr,
				PB_ENCODE_DELIMITED);
				i++;
				//lastMessageId = Datarptr->sample_number;
			//}
			osMailFree(DataMail, Datarptr);
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
		}
		//TODO improve this code
		const uint32_t InfoUpdateTimems = 4000;
		static TickType_t lastInfoticks = 4000;//wait until all sensors are inited and have the right baseid
		if (xTaskGetTickCount() - lastInfoticks > InfoUpdateTimems) {
			lastInfoticks = xTaskGetTickCount();
			HAL_GPIO_TogglePin(LED_BT2_GPIO_Port, LED_BT2_Pin);

			//TODO improve this code with adding list of active sensors to configMan
#define NUMDESCRIPTIONSTOSEND 5
			DescriptionMessage_DESCRIPTION_TYPE Tosend[NUMDESCRIPTIONSTOSEND] =
					{ DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY,
							DescriptionMessage_DESCRIPTION_TYPE_UNIT,
							DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION,
							DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE,
					DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE};
			for (int i = 0; i < NUMDESCRIPTIONSTOSEND; i++) {
				DescriptionMessage Descriptionmsg;
				Sensor0.getDescription(&Descriptionmsg,
						(DescriptionMessage_DESCRIPTION_TYPE) Tosend[i]);
				pb_encode_ex(&ProtoStreamDescription, DescriptionMessage_fields,
						&Descriptionmsg, PB_ENCODE_DELIMITED);
				//sending the buffer
				netbuf_ref(buf, &ProtoBufferDescription,
						ProtoStreamDescription.bytes_written);
				/* send the text */
				err_t net_conn_result = netconn_send(conn, buf);
				Check_LWIP_RETURN_VAL(net_conn_result);
				// reallocating buffer this is maybe performance intensive profile this
				//TODO profile this code
				ProtoStreamDescription = pb_ostream_from_buffer(
						ProtoBufferDescription, MTU_SIZE);
				pb_write(&ProtoStreamDescription,
						(const pb_byte_t*) &DescriptionString, 4);

			}


			for (int i = 0; i < NUMDESCRIPTIONSTOSEND; i++) {
				DescriptionMessage Descriptionmsg;
				Sensor1.getDescription(&Descriptionmsg,
						(DescriptionMessage_DESCRIPTION_TYPE) Tosend[i]);
				pb_encode_ex(&ProtoStreamDescription, DescriptionMessage_fields,
						&Descriptionmsg, PB_ENCODE_DELIMITED);
				//sending the buffer
				netbuf_ref(buf, &ProtoBufferDescription,
						ProtoStreamDescription.bytes_written);
				/* send the text */
				err_t net_conn_result = netconn_send(conn, buf);
				Check_LWIP_RETURN_VAL(net_conn_result);
				// reallocating buffer this is maybe performance intensive profile this
				//TODO profile this code
				ProtoStreamDescription = pb_ostream_from_buffer(
						ProtoBufferDescription, MTU_SIZE);
				pb_write(&ProtoStreamDescription,
						(const pb_byte_t*) &DescriptionString, 4);

			}


			for (int i = 0; i < NUMDESCRIPTIONSTOSEND; i++) {
				DescriptionMessage Descriptionmsg;
				TempSensor0.getDescription(&Descriptionmsg,
						(DescriptionMessage_DESCRIPTION_TYPE) Tosend[i]);
				pb_encode_ex(&ProtoStreamDescription, DescriptionMessage_fields,
						&Descriptionmsg, PB_ENCODE_DELIMITED);
				//sending the buffer
				netbuf_ref(buf, &ProtoBufferDescription,
						ProtoStreamDescription.bytes_written);
				/* send the text */
				err_t net_conn_result = netconn_send(conn, buf);
				Check_LWIP_RETURN_VAL(net_conn_result);
				// reallocating buffer this is maybe performance intensive profile this
				//TODO profile this code
				ProtoStreamDescription = pb_ostream_from_buffer(
						ProtoBufferDescription, MTU_SIZE);
				pb_write(&ProtoStreamDescription,
						(const pb_byte_t*) &DescriptionString, 4);

			}
			/*
			 for (int DescriptionType =
			 DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY;
			 DescriptionType != DescriptionMessage_LAST;
			 DescriptionType++) {

			 if (ProtoStreamDescription.bytes_written
			 > (MTU_SIZE - (DescriptionMessage_size))) {
			 //sending the buffer
			 netbuf_ref(buf, &ProtoBufferDescription,
			 ProtoStreamDescription.bytes_written);
			 // send the text
			 err_t net_conn_result = netconn_send(conn, buf);
			 Check_LWIP_RETURN_VAL(net_conn_result);
			 // reallocating buffer this is maybe performance intensive profile this
			 //TODO profile this code
			 ProtoStreamDescription = pb_ostream_from_buffer(
			 ProtoBufferDescription, MTU_SIZE);
			 pb_write(&ProtoStreamDescription,
			 (const pb_byte_t*) &DescriptionString, 4);
			 }
			 DescriptionMessage Descriptionmsg;
			 Sensor1.getDescription(&Descriptionmsg,(DescriptionMessage_DESCRIPTION_TYPE) DescriptionType);
			 pb_encode_ex(&ProtoStreamDescription, DescriptionMessage_fields,
			 &Descriptionmsg, PB_ENCODE_DELIMITED);


			 }
			 */

		}

	}
	osThreadTerminate(NULL);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
	ConfigManager& configMan = ConfigManager::instance();
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	static uint32_t Channel1Tim1CaptureCount = 0;
	static uint32_t Channel2Tim1CaptureCount = 0;
	static uint32_t Channel3Tim1CaptureCount = 0;
	static uint32_t Channel4Tim1CaptureCount = 0;
	static uint32_t Channel1Tim2CaptureCount = 0;
	static uint32_t Channel3Tim2CaptureCount = 0;
	static uint32_t Channel4Tim2CaptureCount = 0;
	static uint32_t GPScaptureCount = 0;
	uint64_t timestamp21 = 0;
	uint64_t timestamp23 = 0;
	uint64_t timestamp24 = 0;
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		Channel1Tim2CaptureCount++;
		timestamp21 = TIM_Get_64Bit_TimeStamp_IC(htim);

		 DataMessage *mptr;
		 mptr = (DataMessage *) osMailAlloc(DataMail, 0);
		 Sensor1.getData(mptr, timestamp21, Channel1Tim2CaptureCount);
		 osStatus result = osMailPut(DataMail, mptr);
	}
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		Channel3Tim2CaptureCount++;
		if (Channel3Tim2CaptureCount % 1 == 0) {
			timestamp23 = TIM_Get_64Bit_TimeStamp_IC(htim);
			HAL_ADC_PollForConversion(&hadc1, 0);
			float adcVal1 = (float) HAL_ADC_GetValue(&hadc1);
			float adcVal2 = (float) HAL_ADC_GetValue(&hadc2);
			float adcVal3 = (float) HAL_ADC_GetValue(&hadc3);
			DataMessage *mptr0;
			mptr0 = (DataMessage *) osMailAlloc(DataMail, 0);
			Sensor0.getData(mptr0, timestamp23, Channel3Tim2CaptureCount);
			//TODO move this functionality into the sensor api!!!
			//Sensor0.addDescriptionStr(DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE,int Channel,const char * Description)
			//Sensor0.addDescriptionFloat(DescriptionMessage_DESCRIPTION_TYPE DESCRIPTION_TYPE,int Channel,float Description)
			//after configuring the channels the can be used with this command
			//Sensor0.addData(int Channel,float value)

			mptr0->has_Data_11 = true;
			mptr0->Data_11 = configMan.getADCVoltage(0, adcVal1);
			mptr0->has_Data_12 = true;
			mptr0->Data_12 = configMan.getADCVoltage(1, adcVal2);
			mptr0->has_Data_13 = true;
			mptr0->Data_13 = configMan.getADCVoltage(2, adcVal3);
			osStatus result = osMailPut(DataMail, mptr0);
		}
	}
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		Channel4Tim2CaptureCount++;
		timestamp24 = TIM_Get_64Bit_TimeStamp_IC(htim);

		//pointer needs to be static otherwiese it would be deletet when jumping out of ISR
		static NMEASTamped *mptr = NULL;
		static uint8_t DMA_NMEABUFFER[NMEBUFFERLEN] = { 0 };
		if (GPScaptureCount > 0) {
			//HAL_UART_RxCpltCallback(&huart7);
			HAL_UART_DMAStop(&huart7);
			HAL_DMA_Abort(&hdma_uart7_rx);
			mptr = (NMEASTamped *) osMailAlloc(NMEAMail, 0);//The parameter millisec must be 0 for using this function in an ISR.
			if (mptr != NULL) {
				mptr->RawTimerCount = timestamp24;
				mptr->CaptureCount = GPScaptureCount;
				memcpy(&(mptr->NMEAMessage[0]), &(DMA_NMEABUFFER[0]),
				NMEBUFFERLEN);
				osStatus result = osMailPut(NMEAMail, mptr);
			}
			//SEGGER_RTT_printf(0,"DMA BUFFER:=%s\n",DMA_NMEABUFFER);
			memset(DMA_NMEABUFFER, 0, sizeof(DMA_NMEABUFFER));
			HAL_StatusTypeDef DMA_START_result = HAL_UART_Receive_DMA(&huart7,
					DMA_NMEABUFFER,
					NMEBUFFERLEN - 1);
			GPScaptureCount++;
			if (DMA_START_result != HAL_OK) {
				SEGGER_RTT_printf(0, "DMA start ERROR");
			}
		} else if (GPScaptureCount == 0) {
			HAL_UART_Receive_DMA(&huart7, &(DMA_NMEABUFFER[0]),
			NMEBUFFERLEN - 1);
			GPScaptureCount++;
		}
	}

	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		//Channel1Tim1CaptureCount++;
		//timestamp=TIM_Get_64Bit_TimeStamp_IC(htim);
	}
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		//Channel4Tim1CaptureCount++;
		//timestamp=TIM_Get_64Bit_TimeStamp_IC(htim);
	}
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
