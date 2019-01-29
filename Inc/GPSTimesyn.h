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

int initGPSTimesny();
void StartNemaParserThread(void const * argument);

typedef struct {
	uint32_t RawTimerCount;
	uint32_t CaptureCount;
	uint8_t NMEAMessage[248]; //248 3 NMEA Sentences
}NMEASTamped;

//MemPool For the data
osPoolDef(NMEAPool, NMEABUFFERSIZE , NMEASTamped);
osPoolId NMEAPool;

osMessageQDef(NMEABuffer, NMEABUFFERSIZE,  uint32_t);
osMessageQId NMEABuffer;

osThreadId NemaParserTID;


osThreadDef(NemaParserThread, StartNemaParserThread, osPriorityNormal, 0,
		256);




#ifdef __cplusplus
}
#endif
#endif /* GPSTIMESYN_H_ */
