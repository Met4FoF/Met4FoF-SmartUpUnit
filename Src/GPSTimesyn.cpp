/*
 * gpstimesyn.c
 *
 *  Created on: 11.01.2019
 *      Author: seeger01
 */

#include <GPSTimesyn.hpp>
struct tref GPS_ref={0};
struct tref NTP_ref={0};

//MemPool For the data
osMailQDef (NMEAMail, NMEABUFFERSIZE , NMEASTamped);
osMailQId NMEAMail;


osThreadId NemaParserTID;

SemaphoreHandle_t xSemaphoreGPS_REF = NULL;
SemaphoreHandle_t xSemaphoreNTP_REF = NULL;

osThreadDef(NemaParserThread, StartNemaParserThread,osPriorityHigh , 0,
		1024);

int initGPSTimesny() {
	int retval = 1;
	NMEAMail = osMailCreate(osMailQ(NMEAMail), NULL);
	if (NMEAMail != NULL) {
		retval = 0;
	}
	NemaParserTID = osThreadCreate(osThread(NemaParserThread), NULL);
	return retval;
}

void StartNemaParserThread(void const * argument) {
	xSemaphoreGPS_REF = xSemaphoreCreateMutex();
	xSemaphoreNTP_REF = xSemaphoreCreateMutex();
	uint32_t porcessedCount = 0;
	osEvent evt;
	enum gps_msg latest_msg;
	struct timespec utc, gps_time;
	while (1) {
		evt = osMailGet(NMEAMail, osWaitForever);
		if (evt.status == osEventMail) {
			NMEASTamped *rptr;
			rptr = (NMEASTamped*) evt.value.p;

			//find all $ start tokens and '\n' end tokens
			int DollarIndexs[MAXNEMASENTENCECOUNT] = { 0 };
			int NewLineIndexs[MAXNEMASENTENCECOUNT] = { 0 };
			int DollarCount = 0;
			int NewLineCount = 0;
#if DEBUG_GPS == 1
                        SEGGER_RTT_printf(0,"Parsing: NMEA Message\n\r %s\n\r",(rptr->NMEAMessage));
#endif
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
					latest_msg = lgw_parse_nmea((const char*)&(rptr->NMEAMessage[DollarIndexs[i]]),NewLineIndexs[i] - DollarIndexs[i]);
			}
		    if( xSemaphoreGPS_REF != NULL )
		    {
		        /* See if we can obtain the semaphore.  If the semaphore is not
		        available wait 10 ticks to see if it becomes free. */
		        if( xSemaphoreTake(xSemaphoreGPS_REF, ( TickType_t ) 10 ) == pdTRUE )
		        {
		    //TODO COMPARE WITH NTP may be a second diference !!
			lgw_gps_get(&utc, &gps_time, NULL, NULL);
			lgw_gps_sync(&GPS_ref, rptr->RawTimerCount, utc, gps_time);
            xSemaphoreGive(xSemaphoreGPS_REF);
		        }
		        else
		        {
		            /* We could not obtain the semaphore and can therefore not access
		            the shared resource safely. */
#if DEBUG_GPS == 1
		        	SEGGER_RTT_printf(0,"GPS SYNC UPDATE FAIL SEMAPHORE NOT READY !!!\n\r");
#endif
		        }
		    }
			osMailFree(NMEAMail, rptr);
			porcessedCount++;
		}

	}

osThreadTerminate(NULL);
}

void NTP_time_CNT_update(time_t t,uint32_t us){
	uint64_t timestamp=TIM_Get_64Bit_TimeStamp_Base(&htim2);
	//TODO 64 bit timestamping not working !! with no channel

	timespec dummy={0};
	timespec TmpUtc;
	TmpUtc.tv_sec=t;
	TmpUtc.tv_nsec=1000*us;
    if( xSemaphoreNTP_REF != NULL )
    {
        if( xSemaphoreTake(xSemaphoreNTP_REF, ( TickType_t ) 10 ) == pdTRUE )
        {
        	lgw_gps_sync(&NTP_ref, timestamp, TmpUtc,dummy);
        	xSemaphoreGive(xSemaphoreNTP_REF);
        }
    }

	timespec GPSUtc;
	timespec NTPUtc;
	uint32_t GPS_time_uncertainty=0;
	uint32_t NTP_time_uncertainty=0;
    /* See if we can obtain the semaphore.  If the semaphore is not
    available wait 10 ticks to see if it becomes free. */
    if( xSemaphoreTake(xSemaphoreGPS_REF, ( TickType_t ) 10 ) == pdTRUE )
    {

	lgw_cnt2utc(GPS_ref,timestamp,&GPSUtc,&GPS_time_uncertainty);
    xSemaphoreGive(xSemaphoreGPS_REF);
    }

    if( xSemaphoreTake(xSemaphoreNTP_REF, ( TickType_t ) 10 ) == pdTRUE )
    {
	uint32_t GPS_time_uncertainty=0;
	lgw_cnt2utc(NTP_ref,timestamp,&NTPUtc,&NTP_time_uncertainty);
    xSemaphoreGive(xSemaphoreNTP_REF);
    }
    uint64_t deltaTime=((uint32_t)NTPUtc.tv_sec-(uint32_t)GPSUtc.tv_sec)*1e9;
    		deltaTime+=NTPUtc.tv_nsec-GPSUtc.tv_nsec;
    SEGGER_RTT_printf(0,"NTP-GPS time diff=%d ns\n\r",deltaTime);
}


