/*
 * GPSTimesyn.h
 *
 *  Created on: 11.01.2019
 *      Author: seeger01
 */

#ifndef GPSTIMESYN_H_
#define GPSTIMESYN_H_

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
#include "GPSTimesyn.h"
#include "NMEAPraser.h"
#include <time.h>       /* struct timespec */

#define NMEABUFFERSIZE 10
#define NMEBUFFERLEN 246
#define NMEAMINLEN 9
#define MAXNEMASENTENCECOUNT NMEBUFFERLEN/NMEAMINLEN
int initGPSTimesny();
void StartNemaParserThread(void const * argument);

typedef struct {
	uint32_t RawTimerCount;
	uint32_t CaptureCount;
	uint8_t NMEAMessage[NMEBUFFERLEN]; //248 3 NMEA Sentences
}NMEASTamped;

//MemPool For the data
osPoolDef(NMEAPool, NMEABUFFERSIZE , NMEASTamped);
osPoolId NMEAPool;

osMessageQDef(NMEABuffer, NMEABUFFERSIZE,  uint32_t);
osMessageQId NMEABuffer;

osThreadId NemaParserTID;

osMutexDef (GPS_ref_mutex);    // Declare mutex
osMutexId  (GPS_ref_mutex_id); // Mutex ID
struct tref GPS_ref;


osThreadDef(NemaParserThread, StartNemaParserThread,osPriorityHigh , 0,
		256);




#ifdef __cplusplus
}
#endif
#endif /* GPSTIMESYN_H_ */
