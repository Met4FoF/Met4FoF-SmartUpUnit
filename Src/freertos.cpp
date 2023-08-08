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
#include "configmanager.h"
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

#include "Met4FoFSensor.h"
#include "MPU9250.h"
#include "bma280.h"
#include "MS5837.h"
#include "Met4FoF_adc.h"
#include "bmp280.h"
#include "Met4FoFEdgeTS.h"
#include "Met4FoFGPSPub.h"
#include "Met4FoFLsm6dsrx.h"
#include "MAX31865.h"
#include "ADXL355.h"

#include "math.h"
#include <vector>


#include "adc.h"
#include "tim.h"
#include "rng.h"
#include "usart.h"
#include "dma.h"
#include "i2c.h"

#include "backupsram.h"

#include "lwip/apps/sntp.h"
#include "lwip_return_ckeck.h"

#include "NMEAPraser.h"
#include "lwip/apps/sntp.h"

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
osThreadId NmeaParserTID;

struct tref GPS_ref = { 0 };
struct tref NTP_ref = { 0 };

//MemPool For the NMEA data
osMailQDef(NMEAMail, NMEABUFFERSIZE, NMEASTamped);
osMailQId NMEAMail;
static uint8_t DMA_NMEABUFFER[NMEBUFFERLEN] = { 0 };

SemaphoreHandle_t xSemaphoreGPS_REF = NULL;
SemaphoreHandle_t xSemaphoreNTP_REF = NULL;


MPU9250 Sensor0(SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, &hspi1, 0);
MPU9250 Sensor1(SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, &hspi1, 1);
MPU9250 Sensor2(SENSOR_CS3_GPIO_Port, SENSOR_CS3_Pin, &hspi2, 2);
MPU9250 Sensor3(SENSOR_CS4_GPIO_Port, SENSOR_CS4_Pin, &hspi2, 3);
//Met4FoFLsm6dsrx Sensor0(SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, &hspi1, 0);
//ADXL355 Sensor0(SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, &hspi1, 0);
//MAX31865 Sensor1(SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, &hspi1, 1);
//vectSensors.push_back((Met4FoFSensor *)&Sensor0);
//BMA280 Sensor1(SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, &hspi1, 1);
//MS5837 TempSensor0(&hi2c1,MS5837::MS5837_02BA);
//BMP280 AirPressSensor(hi2c1);
//Met4FoF_adc Met4FoFADC(&hadc1,&hadc2,&hadc3,10);
Met4FoFGPSPub GPSPub(&GPS_ref, 20);
//vectSensors.push_back((Met4FoFSensor *)&GPS_ref);
//Met4FoFEdgeTS EdgePub0(1.0,30);
//Met4FoFEdgeTS EdgePub1(1.0,31);

 //Met4FoFEdgeTS Sensor0(1.0,0);
 //Met4FoFEdgeTS Sensor1(1.0,1);
 //vectSensors.push_back((Met4FoFSensor *)&Sensor1);
 //Met4FoFEdgeTS Sensor2(1.0,2);
 //vectSensors.push_back((Met4FoFSensor *)&Sensor2);
 //Met4FoFEdgeTS Sensor3(1.0,3);
 //vectSensors.push_back((Met4FoFSensor *)&Sensor3);


//std::vector<Met4FoFSensor *> Sensors;
const int numSensors = 5;
Met4FoFSensors::Met4FoFSensor *Sensors[numSensors] = { &Sensor0, &Sensor1, &Sensor2, &Sensor3,&GPSPub }; //,
osMailQDef(DataMail, DATAMAILBUFFERSIZE, DataMessage);
osMailQId DataMail;
static bool Lwip_init_finished = false;
static bool GPS_init_finished = true;
static bool Sensors_init_finished = false;
#define NUMDESCRIPTIONSTOSEND 6
DescriptionMessage_DESCRIPTION_TYPE Tosend[NUMDESCRIPTIONSTOSEND] = {
		DescriptionMessage_DESCRIPTION_TYPE_PHYSICAL_QUANTITY,
		DescriptionMessage_DESCRIPTION_TYPE_UNIT,
		DescriptionMessage_DESCRIPTION_TYPE_RESOLUTION,
		DescriptionMessage_DESCRIPTION_TYPE_MIN_SCALE,
		DescriptionMessage_DESCRIPTION_TYPE_MAX_SCALE,
		DescriptionMessage_DESCRIPTION_TYPE_HIERARCHY };

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

	osThreadDef(blinkThread, StartBlinkThread, osPriorityHigh, 0, 256);
	blinkTID = osThreadCreate(osThread(blinkThread), NULL);

	osThreadDef(WebserverTherad, StartWebserverThread, osPriorityNormal, 0,
			512);
	WebServerTID = osThreadCreate(osThread(WebserverTherad), NULL);

	osThreadDef(LCDThread, StartLCDThread, osPriorityNormal, 0, 512);

	LCDTID = osThreadCreate(osThread(LCDThread), NULL);

	osThreadDef(DataStreamerThread, StartDataStreamerThread, osPriorityNormal,
			0, 4096);

	DataStreamerTID = osThreadCreate(osThread(DataStreamerThread), NULL);

	osThreadDef(TempSensorThread, StartTempSensorThread, osPriorityNormal, 0,
			512);

	TempSensorTID = osThreadCreate(osThread(TempSensorThread), NULL);

	osThreadDef(NmeaParserThread, StartNmeaParserThread, osPriorityRealtime, 0,
			1024);

	NmeaParserTID = osThreadCreate(osThread(NmeaParserThread), NULL);

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
void StartDefaultTask(void const *argument) {
	ConfigManager &configMan = ConfigManager::instance();
	//Met4FoFSensors::vectMet4FoFSensors.push_back((Met4FoFSensor::Met4FoFSensor *)&Sensor0);
	ip_addr_t defaultUDPAdrr;
	defaultUDPAdrr.addr=0xC800A8C0;//0xC0A800C8;
	configMan.setUDPTargetIP(defaultUDPAdrr);

	uint32_t random32bit = 0;
	HAL_RNG_GenerateRandomNumber(&hrng, &random32bit); // genrate some random data for dhcp inting will be read from rng register by LWIP
	/* init code for LWIP */
	MX_LWIP_Init();

	/* init code for FATFS */
	//MX_FATFS_Init();
	Lwip_init_finished = true;
	/* USER CODE BEGIN StartDefaultTask */

	//TODO implent NPTP ip array
	ip_addr_t NTPIP = configMan.getUDPTargetIP();
	osDelay(5000);
	//Set the method of obtaining SNTP -> Use the method of obtaining from the server
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setserver(0, &NTPIP);
	sntp_init();
	/* Infinite loop */
	for (;;) {
		osDelay(1000);
		//sntp_request(NULL);
	}
	/* USER CODE END StartDefaultTask */
}

void StartNmeaParserThread(void const *argument) {
	NMEAMail = osMailCreate(osMailQ(NMEAMail), NULL);
	if (NMEAMail == NULL) {
		SEGGER_RTT_printf(0, "Fatal Error could not create NMEA Mail Que!!!\n");
	} else {
		SEGGER_RTT_printf(0, " Created NMEA Mail Que\n");
	}
	xSemaphoreGPS_REF = xSemaphoreCreateMutex();
	xSemaphoreNTP_REF = xSemaphoreCreateMutex();
	//Slave (TIM1) before Master (TIM2)
	//HAL_TIM_Base_Start(&htim1);
	//HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim2);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	uint32_t porcessedCount = 0;
	osEvent evt;

	struct timespec utc, gps_time;
	struct timespec last_utc;
	last_utc.tv_sec = 0;
	last_utc.tv_nsec = 0;
#define SAMETIMEUTCSYNCTRIESUNTIEREBOOT 1200 // wating 20 minutes to have an good GPS fix again
	static int SameTimeGPSSyncTryes = 0;
	while (1) {
		evt = osMailGet(NMEAMail, osWaitForever);
		if (evt.status == osEventMail) {
			GPS_init_finished = true;	// wait until first gps fix
			NMEASTamped *rptr;
			rptr = (NMEASTamped*) evt.value.p;
			//find all $ start tokens and '\n' end tokens
			int DollarIndexs[MAXNEMASENTENCECOUNT] = { 0 };
			int NewLineIndexs[MAXNEMASENTENCECOUNT] = { 0 };
			int DollarCount = 0;
			int NewLineCount = 0;
			//SEGGER_RTT_printf(0,"Parsing: NMEA Message\n %s\n",(rptr->NMEAMessage));
			for (int i = 0; i < sizeof(rptr->NMEAMessage); i++) {
				if (rptr->NMEAMessage[i]
						== '$'&&DollarCount<MAXNEMASENTENCECOUNT) {
					DollarIndexs[DollarCount] = i;
					DollarCount++;
				}
				if (rptr->NMEAMessage[i]
						== '\n'&&NewLineCount<MAXNEMASENTENCECOUNT) {
					NewLineIndexs[NewLineCount] = i;
					NewLineCount++;
				}
			}
			// for minimal len
			for (int i = 0; i < MAXNEMASENTENCECOUNT; i++) {
				if ((NewLineIndexs[i] - DollarIndexs[i]) < NMEAMINLEN) {
					NewLineIndexs[i] = 0;
					DollarIndexs[i] = 0;
				}
			}
			for (int i = 0; i <= DollarCount; i++) {
				//lgw_parse_nmea(const char *serial_buff, int buff_size)
				enum gps_msg latest_msg;
				latest_msg = lgw_parse_nmea(
						(const char*) &(rptr->NMEAMessage[DollarIndexs[i]]),
						NewLineIndexs[i] - DollarIndexs[i]);
			}
			if (xSemaphoreGPS_REF != NULL) {
				/* See if we can obtain the semaphore.  If the semaphore is not
				 available wait 10 ticks to see if it becomes free. */
				if ( xSemaphoreTake(xSemaphoreGPS_REF,
						(TickType_t ) 10) == pdTRUE) {
					//TODO COMPARE WITH NTP may be a second diference !!
					lgw_gps_get(&utc, &gps_time, NULL, NULL);

					if (last_utc.tv_sec != utc.tv_sec
							|| last_utc.tv_nsec != utc.tv_nsec) {
						lgw_gps_sync(&GPS_ref, rptr->RawTimerCount, utc,
								gps_time);
						last_utc = utc;
						SameTimeGPSSyncTryes = 0;
					} else {
						SEGGER_RTT_printf(0,
								"GPS SYNC UPDATE FAILED Time Already used ignoring this data point!\n%d/%d Times until SoftReset\n",
								SameTimeGPSSyncTryes,
								SAMETIMEUTCSYNCTRIESUNTIEREBOOT);
						SEGGER_RTT_printf(0,
								"__________NMEA MSG In BUffer__________\n");
						SEGGER_RTT_WriteString(0,
								(const char*) rptr->NMEAMessage);
						SEGGER_RTT_printf(0,
								"__________NMEA MSG END________________\n");
						if (rptr->GPSUARTDMA_START_result != HAL_OK) {
							SEGGER_RTT_printf(0,
									"Error cause by DMA Error and not by bad GPS reception\nPenalty 10 retry times\nRestarting UART interface\n");
							SameTimeGPSSyncTryes = SameTimeGPSSyncTryes + 10;
						} else {
							SameTimeGPSSyncTryes++;
						}
						if (SameTimeGPSSyncTryes
								== (SAMETIMEUTCSYNCTRIESUNTIEREBOOT - 1)) {
							SEGGER_RTT_printf(0,
									"Goodbye world preparing for reboot!\n");
						}
						if (SameTimeGPSSyncTryes
								> (SAMETIMEUTCSYNCTRIESUNTIEREBOOT)) {
							SEGGER_RTT_printf(0,
									"Better die than suffer!\n Calling resthandler\n");
							NVIC_SystemReset();
						}
					}
					DataMessage *mptr;
					mptr = (DataMessage*) osMailAlloc(DataMail, 0);
					if (mptr!=NULL){
						GPSPub.getData(mptr, rptr->RawTimerCount);
						osStatus result = osMailPut(DataMail, mptr);
					}
					xSemaphoreGive(xSemaphoreGPS_REF);
				} else {
					/* We could not obtain the semaphore and can therefore not access
					 the shared resource safely. */
					SEGGER_RTT_printf(0,
							"GPS SYNC UPDATE FAIL SEMAPHORE NOT READY !!!\n");
				}
			}
			osMailFree(NMEAMail, rptr);
			porcessedCount++;
		}

	}

	osThreadTerminate(NULL);
}

void StartTempSensorThread(void const *argument) {
	while (not Lwip_init_finished || not GPS_init_finished) {
		osDelay(100);
	}
	ConfigManager &configMan = ConfigManager::instance();

	static uint32_t TempsensoreCaptureCount = 0;
	//TempSensor0.init(SensorID3);
	for (;;) {
		osDelay(2000);
		/*
		 DataMessage *mptr;
		 mptr = (DataMessage *) osMailAlloc(DataMail, 20);
		 uint64_t timestamp = TIM_Get_64Bit_TimeStamp_Base(&htim2);
		 //int MS5837::getData(DataMessage * Message,uint32_t unix_time,uint32_t unix_time_nsecs,uint32_t time_uncertainty,uint32_t CaptureCount)
		 TempSensor0.getData(mptr,timestamp);
		 mptr->time_uncertainty=40e6;
		 osStatus result = osMailPut(DataMail, mptr);
		 TempsensoreCaptureCount++;
		 osDelay(10);
		 */
		/*
		 SEGGER_RTT_printf(0,"Scanning I2C bus:\r\n");

		 HAL_StatusTypeDef i2cresult;
		 uint8_t i;
		 for (i=1; i<128; i++)
		 {

		 // the HAL wants a left aligned i2c address
		 // &hi2c1 is the handle
		 // (uint16_t)(i<<1) is the i2c address left aligned
		 // retries 2
		 // timeout 2
		 //
		 i2cresult = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
		 if (i2cresult != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
		 {
		 SEGGER_RTT_printf(0,"."); // No ACK received at that address
		 }
		 if (i2cresult == HAL_OK)
		 {
		 SEGGER_RTT_printf(0,"0x%X", i); // Received an ACK at that address
		 }
		 }
		 SEGGER_RTT_printf(0,"\r\n");
		 */
		osDelay(1000);

	}
}

void StartWebserverThread(void const *argument) {
	while (not Lwip_init_finished) {
		osDelay(100);
	}
	ConfigManager &configMan = ConfigManager::instance();
	// wait until LWIP is inited
	osDelay(5000);
	SEGGER_RTT_printf(0, "Starting Web Server\r\n");
	http_server_netconn_init();
	/* Infinite loop */
	for (;;) {
		osThreadTerminate(NULL);
	}
}

void StartBlinkThread(void const *argument) {
	uint32_t loops = 0;
	ConfigManager &configMan = ConfigManager::instance();
	while (not Sensors_init_finished) {
		osDelay(100);
	}
	uint32_t lastSampleCount[4] = { 0 };
	bool justRestarted = true;
	bool justRestartedDelay[4] = { false };

	//MPU9250 *MPUSSenors[4] = { &Sensor0, &Sensor1, &Sensor2, &Sensor3 };
	while (1) {
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		/*
		//float temperatur=Sensor1.temperature();

		for (int i = 0; i < 4; i++) {
			MPU9250 *MPUSensor = MPUSSenors[i];
			float nominalFreq = MPUSensor->getNominalSamplingFreq();
			uint32_t actualSampleCount = MPUSensor->getSampleCount();
			float deltaSamples = actualSampleCount - lastSampleCount[i];
			lastSampleCount[i] = actualSampleCount;
			SEGGER_RTT_printf(0,
					"Sensor ID %u nopminal sample freq %d actual sample freq %d \n",
					i, (int) nominalFreq, (int) deltaSamples);
			bool freqToHigh = deltaSamples > (nominalFreq * 1.25);
			bool freqToLow = deltaSamples < (nominalFreq * 0.75);
			//TODO deal with to fast sensor corecctly maybe deactivating STM32 ISR investigate if power cycle of MPU is needed
			if (freqToHigh or freqToLow) {
				if (freqToHigh) {
					SEGGER_RTT_printf(0, "Sensor ID %u Freq to HIGH\n", i);
				} else {
					SEGGER_RTT_printf(0, "Sensor ID %u Freq to LOW\n", i);
				}

				if (justRestarted == false) {
					if (i == 0 || i == 1) {
						Sensor0.disableDataReadyInterrupt();
						Sensor1.disableDataReadyInterrupt();
					} else {
						Sensor2.disableDataReadyInterrupt();
						Sensor3.disableDataReadyInterrupt();
					}

					//MPU9250 reconfigure
					SEGGER_RTT_printf(0,
							"WARNING SAMPLE FREQ WATCHDOG TRIPPED !\n RESETING SENSOR\n");
					SEGGER_RTT_printf(0,
							"Sensor ID %u nominal sample freq %d actual sample freq %d \n",
							i, (int) nominalFreq, (int) deltaSamples);
					uint32_t MPUId = configMan.getSensorBaseID(i);
					MPUSensor->setBaseID(MPUId);
					MPUSensor->begin();
					MPUSensor->setGyroRange(MPU9250::GYRO_RANGE_250DPS);
					MPUSensor->setAccelRange(MPU9250::ACCEL_RANGE_4G);
					lastSampleCount[i] = 0;
					justRestarted = true;
					//MPUSensor->setSrd(1);

					if (i == 0 || i == 1) {
						Sensor0.enableDataReadyInterrupt();
						Sensor1.enableDataReadyInterrupt();
					} else {
						Sensor2.enableDataReadyInterrupt();
						Sensor3.enableDataReadyInterrupt();
					}
				} else {
					justRestarted = false;	// reset just restarted flag
				}

			}

		}
	 	 */
		//DataMessage IMUMsg;
		//uint64_t dummyTimeStamp=0;
		//Sensor0.getData(&IMUMsg,dummyTimeStamp);
		osDelay(1000);
	}
	osThreadTerminate(NULL);
}

void StartLCDThread(void const *argument) {
	while (not Lwip_init_finished) {
		osDelay(100);
	}
	ConfigManager &configMan = ConfigManager::instance();
	osDelay(10);
	HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_SET);
	ILI9341_Init();		//initial driver setup to drive ili9341
	ILI9341_Fill_Screen(BLUE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
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
	bool dhcpReady = dhcp_supplied_address(&gnetif);
	if (dhcpReady) {

		ip4addr_ntoa_r(&(gnetif.ip_addr), iPadressBuffer,
				sizeof(iPadressBuffer));
		sprintf(Temp_Buffer_text, "IP %s", (const char*) &iPadressBuffer);
	} else {
		sprintf(Temp_Buffer_text, "DCHP FAILD !!");
		dhcp_start(&gnetif);

	}
	ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
	static int lcdupdatecnt = 0;
	while (1) {
		osDelay(1000);

		lcdupdatecnt++;
		/*
		 timespec utc;
		 timespec gps_time;
		 lgw_gps_get(&utc, &gps_time, NULL, NULL);
		 tm* current_time = localtime(&(utc.tv_sec));
		 strftime(Temp_Buffer_text, 20, "%Y-%m-%d %H:%M:%S", current_time);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 100, WHITE, 2, BLUE);

		 //TODO fix nanospecs printf bug to reactivate --specs=nano.specs -u _printf_float -u _scanf_float to save 50 kb Code size
		 GPSCounterFreq=GPS_ref.xtal_err;
		 GPSCounterFreqUncer=GPS_ref.xtal_err_deviation;
		 NTPCounterFreq=NTP_ref.xtal_err;
		 NTPCounterFreqUncer=NTP_ref.xtal_err_deviation;
		 sprintf(Temp_Buffer_text, "Counter Freq.: %lf Hz   ",GPS_ref.xtal_err);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 120, WHITE, 1, BLUE);
		 sprintf(Temp_Buffer_text, "F std.: %lf Hz      ",GPSCounterFreqUncer);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 140, WHITE, 1, BLUE);
		 sprintf(Temp_Buffer_text, "NTP Counter Freq.: %lf Hz   ",NTPCounterFreq);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 160, WHITE, 1, BLUE);
		 sprintf(Temp_Buffer_text, "NTP F std.: %lf Hz      ",NTPCounterFreqUncer);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 180, WHITE, 1, BLUE);
		 tm* ntp_update_time = localtime(&(NTP_ref.utc.tv_sec));
		 strftime(Temp_Buffer_text, 40, "NTP Updte: %Y-%m-%d %H:%M:%S",
		 ntp_update_time);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 200, WHITE, 1, BLUE);

		 sprintf(Temp_Buffer_text, "Counting happy: %i",lcdupdatecnt);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 210, WHITE, 1, BLUE);
		 sprintf(Temp_Buffer_text, "Start count: %i",startcount);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 220, WHITE, 1, BLUE);

		 if (lcdupdatecnt %100==0) {
		 ILI9341_Init();		//initial driver setup to drive ili9341
		 ILI9341_Fill_Screen(BLUE);
		 ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		 uint16_t BaseID = configMan.getBaseID();
		 ILI9341_Draw_Text("Met4FoF SmartUpUnit", 0, 0, WHITE, 2, BLUE);
		 sprintf(Temp_Buffer_text, "Rev:%d.%d.%d  ID:%x", VERSION_MAJOR,
		 VERSION_MINOR, VERSION_PATCH, BaseID);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 20, WHITE, 2, BLUE);
		 sprintf(Temp_Buffer_text, "Build:%s %s", __DATE__, __TIME__);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 40, WHITE, 1, BLUE);
		 ip_addr_t UDPTargetIP = configMan.getUDPTargetIP();
		 ip4addr_ntoa_r(&(UDPTargetIP), iPadressBuffer, sizeof(iPadressBuffer));
		 sprintf(Temp_Buffer_text, "UPD Targ:%s", iPadressBuffer);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
		 ip4addr_ntoa_r(&(gnetif.ip_addr), iPadressBuffer, sizeof(iPadressBuffer));
		 sprintf(Temp_Buffer_text, "IP %s", (const char *) &iPadressBuffer);
		 ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
		 }
		 */
		if (lcdupdatecnt == 15) {
			iPadressBuffer[17] = { };
			ip4addr_ntoa_r(&(gnetif.ip_addr), iPadressBuffer,
					sizeof(iPadressBuffer));
			sprintf(Temp_Buffer_text, "IP %s    ", iPadressBuffer);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 60, WHITE, 2, BLUE);
			ip_addr_t UDPTargetIP = configMan.getUDPTargetIP();
			ip4addr_ntoa_r(&(UDPTargetIP), iPadressBuffer,
					sizeof(iPadressBuffer));
			iPadressBuffer[17] = { };
			sprintf(Temp_Buffer_text, "UPD Targ:%s", iPadressBuffer);
			ILI9341_Draw_Text(Temp_Buffer_text, 0, 80, WHITE, 2, BLUE);
		}

	}
	osThreadTerminate(NULL);
}

void StartDataStreamerThread(void const *argument) {
	while (not Lwip_init_finished || not GPS_init_finished) {
		osDelay(100);
	}
	ConfigManager &configMan = ConfigManager::instance();

	DataMail = osMailCreate(osMailQ(DataMail), NULL);
	if (DataMail == NULL) {
		SEGGER_RTT_printf(0, "Fatal Error could not create Data Mail Que!!!\n");
	} else {
		SEGGER_RTT_printf(0, " Created Data Mail Que\n");
	}
	//Set Base Id for all Sensors
	uint16_t baseID=configMan.getBaseID();
	for (int sensorcount = 0; sensorcount <= numSensors; sensorcount++) {
		// TODO Ad sanor manger to avid code doubling
		// and automatic loop over all aktive sensors
		Met4FoFSensors::Met4FoFSensor *sensor = Sensors[sensorcount];
		sensor->setBaseID(baseID);
	}

	MPU9250 *MPUSSenors[6] ={ &Sensor0, &Sensor1, &Sensor2, &Sensor3, &Sensor2, &Sensor3 };//TODO Fix bug in SPI2 and MPU intialsation witch leeds to failiure in first loop but succes if an other sensor gets inited before this makes absolutly no sense at all nasty workaround: init sernsor 2 fail --> init senor 3 -->init sensor 2 again succes
	for (int i = 0; i < 6; i++) {
		MPU9250 *MPUSensor = MPUSSenors[i];
		int retyCount = 0;
		while (retyCount < 10) {
			SEGGER_RTT_printf(0, "Initing Sensor %d\n", i);
			retyCount++;
			int initresult = MPUSensor->begin();
			SEGGER_RTT_printf(0, "Sensor init result  %d\n", initresult);
			if (initresult == 1) {
				break;
			}
		}
		MPUSensor->setGyroRange(MPU9250::GYRO_RANGE_250DPS);
		MPUSensor->setAccelRange(MPU9250::ACCEL_RANGE_4G);
		MPUSensor->setSrd(1);
		//MPU9250
	}
	for (int i = 0; i < 6; i++) {
		MPU9250 *MPUSensor = MPUSSenors[i];
		MPUSensor->enableDataReadyInterrupt();
	}

	SEGGER_RTT_printf(0, "Sensors Init Done\n");
	Sensors_init_finished=true;

	SEGGER_RTT_printf(0,
			"UDID=%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX\n",
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
	ip_addr_t settargetipaddr = configMan.getUDPTargetIP();
	// OPK check if the ip adress is not empty if empty use default address an store this adress in memory
	if (settargetipaddr.addr == 0x00000000) {
		IP4_ADDR(&targetipaddr, UDP_TARGET_DEFAULT_IP_ADDRESS[0],
				UDP_TARGET_DEFAULT_IP_ADDRESS[1],
				UDP_TARGET_DEFAULT_IP_ADDRESS[2],
				UDP_TARGET_DEFAULT_IP_ADDRESS[3]);
		configMan.setUDPTargetIP(targetipaddr);
	}

	targetipaddr = configMan.getUDPTargetIP();
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

	/* Enable ADCs external trigger */
	//TODO check if this belonges into the adc functionality
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);

	//arm inputcapture
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	while (1) {
		DataMessage *Datarptr;
		//static uint32_t lastMessageId = 0;
		osEvent DataEvent = osMailGet(DataMail, 100);
		struct timespec SampelPointUtc;
		if (DataEvent.status == osEventMail) {
			Datarptr = (DataMessage*) DataEvent.value.p;
			HAL_GPIO_TogglePin(LED_BT1_GPIO_Port, LED_BT1_Pin);
			if (Datarptr->unix_time == 0xFFFFFFFF) {
				//calculate the exact time from raw time stamp encoded in Datarptr->time_uncertainty Datarptr->unix_time_nsecs as actual time stamp
				//if Datarptr->unix_time is not 0xFFFFFFFF the calculation has been done before and so this conversion will yield only garbage
				if (xSemaphoreGPS_REF != NULL) {
					/* See if we can obtain the semaphore.  If the semaphore is not
					 available wait 10 ticks to see if it becomes free. */
					if ( xSemaphoreTake(xSemaphoreGPS_REF,
							(TickType_t ) 10) == pdTRUE) {
						uint64_t timestamp =
								(uint64_t) (Datarptr->time_uncertainty) << 32;
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
								"cnt to GPS time  UPDATE FAIL SEMAPHORE NOT READY !!!\n");
						taskYIELD();
					}
				}
				Datarptr->unix_time = (uint32_t) (SampelPointUtc.tv_sec);
				Datarptr->unix_time_nsecs = (uint32_t) (SampelPointUtc.tv_nsec);
			}
			if (Datarptr->unix_time != 0) {
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
			else
			{
				osMailFree(DataMail, Datarptr);// we havent sended the data but we must remove it frome the quue
			}
		}
		//TODO improve this code
		const uint32_t InfoUpdateTimems = 4000;
		static TickType_t lastInfoticks = 4000;
		TickType_t actualticks = xTaskGetTickCount();
		//wait until all sensors are inited and have the right baseid
		if (actualticks - lastInfoticks > InfoUpdateTimems) {
			lastInfoticks = xTaskGetTickCount();
			HAL_GPIO_TogglePin(LED_BT2_GPIO_Port, LED_BT2_Pin);
			//TODO improve this code with adding list of active sensors to configMan
			for (int sensorcount = 0; sensorcount < numSensors; sensorcount++) {
				// TODO Ad sanor manger to avid code doubling
				// and automatic loop over all aktive sensors
				Met4FoFSensors::Met4FoFSensor *Sensor = Sensors[sensorcount];
				for (int i = 0; i < NUMDESCRIPTIONSTOSEND; i++) {
					DescriptionMessage Descriptionmsg;
					Sensor->getDescription(&Descriptionmsg,
							(DescriptionMessage_DESCRIPTION_TYPE) Tosend[i]);
					pb_encode_ex(&ProtoStreamDescription,
							DescriptionMessage_fields, &Descriptionmsg,
							PB_ENCODE_DELIMITED);
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
			}

		}

	}
	osThreadTerminate(NULL);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	ConfigManager &configMan = ConfigManager::instance();
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	static uint32_t missedChannel1Tim4CaptureCount = 0;
	static uint32_t missedChannel2Tim4CaptureCount = 0;
	static uint32_t missedChannel3Tim4CaptureCount = 0;
	static uint32_t missedChannel4Tim4CaptureCount = 0;
	static uint32_t missedChannel1Tim2CaptureCount = 0;
	static uint32_t missedChannel3Tim2CaptureCount = 0;
	static uint32_t missedChannel4Tim2CaptureCount = 0;
	static uint32_t GPScaptureCount = 0;
	uint64_t timestamp = 0;
	static HAL_StatusTypeDef GPSUARTDMA_START_result = HAL_OK;
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
			//pointer needs to be static otherwiese it would be deletet when jumping out of ISR
			static NMEASTamped *mptr = NULL;
			if (GPScaptureCount > 0) {
				//STOP DMA transfers
				//HAL_UART_RxCpltCallback(&huart7);
				HAL_UART_DMAStop(&huart7);
				HAL_DMA_Abort(&hdma_uart7_rx);

				// Allocating Message from Pool
				mptr = (NMEASTamped*) osMailAlloc(NMEAMail, 0);	//The parameter millisec must be 0 for using this function in an ISR.
				if (mptr != NULL) {
					mptr->RawTimerCount = timestamp;
					mptr->CaptureCount = GPScaptureCount;
					memcpy(&(mptr->NMEAMessage[0]), &(DMA_NMEABUFFER[0]),
							NMEBUFFERLEN);
					//SEGGER_RTT_WriteString(0,(const char*)mptr->NMEAMessage); maybe to expencive in ISR moving to data processing thread
					mptr->GPSUARTDMA_START_result = GPSUARTDMA_START_result;
					osMailPut(NMEAMail, mptr);

				}
				//SEGGER_RTT_printf(0,"DMA BUFFER:=%s\n",DMA_NMEABUFFER);
				//Flushing Buffer for safety
				memset(DMA_NMEABUFFER, 0, sizeof(DMA_NMEABUFFER));
				GPSUARTDMA_START_result = HAL_UART_Receive_DMA(&huart7,
						DMA_NMEABUFFER,
						NMEBUFFERLEN - 1);
				GPScaptureCount++;
				if (GPSUARTDMA_START_result != HAL_OK) {
					SEGGER_RTT_printf(0, "DMA start ERROR ");
					switch (GPSUARTDMA_START_result) {
					case HAL_OK:
						break;
					case HAL_ERROR:
						SEGGER_RTT_printf(0, "HAL_ERROR\n");
						HAL_DMA_Abort(&hdma_uart7_rx);
						break;
					case HAL_BUSY:
						SEGGER_RTT_printf(0, "HAL_BUSY\n");
						HAL_DMA_Abort(&hdma_uart7_rx);
						break;
					case HAL_TIMEOUT:
						SEGGER_RTT_printf(0, "HAL_TIMEOUT\n");
						HAL_DMA_Abort(&hdma_uart7_rx);
						break;
					}
				}
			} else if (GPScaptureCount == 0) {
				HAL_UART_Receive_DMA(&huart7, &(DMA_NMEABUFFER[0]),
				NMEBUFFERLEN - 1);
				GPScaptureCount++;
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
			//SEGGER_RTT_printf(0,"TIM2CH1: %"PRIu64"\n",timestamp21);
			DataMessage *mptr = NULL;
			mptr = (DataMessage*) osMailAlloc(DataMail, 0);
			//DataMessage *mptrADC=NULL;
			//mptrADC = (DataMessage *) osMailAlloc(DataMail, 0);
			if (mptr != NULL) {
				Sensor0.getData(mptr, timestamp);
				osMailPut(DataMail, mptr);
			} else {
				missedChannel1Tim2CaptureCount++;
				Sensor0.increaseCaptureCountWORead();
				if ((missedChannel1Tim2CaptureCount % 100) == 0) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM2CH1 %u Times,\n",
							missedChannel1Tim2CaptureCount);
				}
				if (missedChannel1Tim2CaptureCount > 10000) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM2CH1 10000 Time stopping this sensor \n");
					//Sensor0.disableDataReadyInterrupt();
					missedChannel1Tim2CaptureCount = 0;
				}
			}
			/*
			 if(mptrADC != NULL){
			 Met4FoFADC.getData(mptrADC, timestamp21);
			 osMailPut(DataMail, mptrADC);
			 }
			 else
			 {
			 Met4FoFADC.increaseCaptureCountWORead();
			 SEGGER_RTT_printf(0, "MEM ERROR Could't allocate Message for TIM2CH1\n");
			 }
			 */

		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
			//SEGGER_RTT_printf(0,"TIM2CH3: %"PRIu64"\n",timestamp23);
			DataMessage *mptr = NULL;
			mptr = (DataMessage*) osMailAlloc(DataMail, 0);
			if (mptr != NULL) {
				Sensor1.getData(mptr, timestamp);
				osMailPut(DataMail, mptr);
			} else {
				missedChannel3Tim2CaptureCount++;
				Sensor1.getData(mptr, timestamp);
				if ((missedChannel3Tim2CaptureCount % 100) == 0) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM2CH3 %u Times,\n",
							missedChannel3Tim2CaptureCount);
				}
				if (missedChannel3Tim2CaptureCount > 10000) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM2CH3 10000 Time stopping this sensor \n");
					//Sensor1.disableDataReadyInterrupt();
					missedChannel3Tim2CaptureCount = 0;
				}
			}

		}
	}
	if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
			//SEGGER_RTT_printf(0,"TIM4CH1: %"PRIu64"\n",timestamp11);
			DataMessage *mptr = NULL;
			mptr = (DataMessage*) osMailAlloc(DataMail, 0);
			if (mptr != NULL) {
				Sensor2.getData(mptr, timestamp);
				osMailPut(DataMail, mptr);
			} else {
				missedChannel1Tim4CaptureCount++;
				Sensor2.getData(mptr, timestamp);
				if ((missedChannel1Tim4CaptureCount % 100) == 0) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM4CH1 %u Times,\n",
							missedChannel1Tim4CaptureCount);
				}
				if (missedChannel1Tim4CaptureCount > 10000) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM4CH1 10000 Time stopping this sensor \n");
					//Sensor2.disableDataReadyInterrupt();
					missedChannel1Tim4CaptureCount = 0;
				}
			}

		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
			//SEGGER_RTT_printf(0,"TIM4CH2: %" PRIu64"\n",timestamp12);
			DataMessage *mptr = NULL;
			mptr = (DataMessage*) osMailAlloc(DataMail, 0);
			if (mptr != NULL) {
				Sensor3.getData(mptr, timestamp);
				osMailPut(DataMail, mptr);
			} else {
				missedChannel2Tim4CaptureCount++;
				Sensor3.getData(mptr, timestamp);
				if ((missedChannel2Tim4CaptureCount % 100) == 0) {
					SEGGER_RTT_printf(0,
							"MEM ERROR Could't allocate Message for TIM4CH1 %u Times,\n",
							missedChannel2Tim4CaptureCount);
				}
				if (missedChannel2Tim4CaptureCount > 10000) {
					SEGGER_RTT_printf(0,
							" MEM ERROR Could't allocate Message for TIM4CH1 10000 Time stopping this sensor \n");
					//Sensor3.disableDataReadyInterrupt();
					missedChannel2Tim4CaptureCount = 0;
				}
			}

		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			timestamp = TIM_Get_64Bit_TimeStamp_IC(htim);
		}
	}
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void NTP_time_CNT_update(time_t t, uint32_t us) {
	uint64_t timestamp = TIM_Get_64Bit_TimeStamp_Base(&htim2);
	//TODO 64 bit timestamping not working !! with no channel

	timespec dummy = { 0 };
	timespec TmpUtc;
	TmpUtc.tv_sec = t;
	TmpUtc.tv_nsec = 1000 * us;
	if (xSemaphoreNTP_REF != NULL) {
		if ( xSemaphoreTake(xSemaphoreNTP_REF, ( TickType_t ) 10 ) == pdTRUE) {
			lgw_gps_sync(&NTP_ref, timestamp, TmpUtc, dummy);
			xSemaphoreGive(xSemaphoreNTP_REF);
		}
	}

	timespec GPSUtc;
	timespec NTPUtc;
	uint32_t GPS_time_uncertainty = 0;
	uint32_t NTP_time_uncertainty = 0;
	/* See if we can obtain the semaphore.  If the semaphore is not
	 available wait 10 ticks to see if it becomes free. */
	if ( xSemaphoreTake(xSemaphoreGPS_REF, ( TickType_t ) 10 ) == pdTRUE) {

		lgw_cnt2utc(GPS_ref, timestamp, &GPSUtc, &GPS_time_uncertainty);
		xSemaphoreGive(xSemaphoreGPS_REF);
	}

	if ( xSemaphoreTake(xSemaphoreNTP_REF, ( TickType_t ) 10 ) == pdTRUE) {
		uint32_t GPS_time_uncertainty = 0;
		lgw_cnt2utc(NTP_ref, timestamp, &NTPUtc, &NTP_time_uncertainty);
		xSemaphoreGive(xSemaphoreNTP_REF);
	}
	uint64_t deltaTime = ((uint32_t) NTPUtc.tv_sec - (uint32_t) GPSUtc.tv_sec)
			* 1e9;
	deltaTime += NTPUtc.tv_nsec - GPSUtc.tv_nsec;
	SEGGER_RTT_printf(0, "NTP-GPS time dif=%d ns\n", deltaTime);
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
