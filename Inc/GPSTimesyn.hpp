/*
 * GPSTimesyn.h
 *
 *  Created on: 11.01.2019
 *      Author: seeger01
 */

#ifndef GPSTIMESYN_HPP_
#define GPSTIMESYN_HPP_

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "cmsis_os.h"
#include <time.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "NMEAPraser.h"
#include "GPSTimesyn.hpp"
#include <time.h>       /* struct timespec */
#include <sys/timespec.h>

//for ntp snchronisation
#include "tim.h"
#include "freertos_cubemx.h"
#include "tim64extender.h"
#include "SEGGER_RTT.h"
#include "lwip/apps/sntp.h"

/*
 * see:
 * #define GPS_TIME_OFFSET_FROM_BUFFER_SEC 1
 * in NMEAPraser.c !!
 */



#define NMEABUFFERSIZE 3
#define NMEBUFFERLEN 1200
#define NMEAMINLEN 9
#define MAXNEMASENTENCECOUNT NMEBUFFERLEN/NMEAMINLEN
int initGPSTimesny();
void StartNemaParserThread(void const * argument);
void NTP_time_CNT_update(time_t t,uint32_t us);

typedef struct {
	uint64_t RawTimerCount;
	uint32_t CaptureCount;
	uint8_t NMEAMessage[NMEBUFFERLEN]; //248 3 NMEA Sentences
}NMEASTamped;

typedef struct {
	uint64_t RawTimerCount;
	uint32_t CaptureCount;
    struct timespec utc;        /*!> reference UTC time (from GPS/NMEA) */
    struct timespec gps_time;        /*!> reference GPS time (since 01.Jan.1980) */
}GPSDebugMsg;


extern osMailQId NMEAMail;

extern SemaphoreHandle_t xSemaphoreGPS_REF;
//TODO maybe (quite shure) needs mutex check that
extern struct tref GPS_ref;

extern SemaphoreHandle_t xSemaphoreNTP_REF;
//TODO maybe (quite shure) needs mutex check that
extern struct tref NTP_ref;


#ifdef __cplusplus
}
#endif
#endif /* GPSTIMESYN_HPP_ */
