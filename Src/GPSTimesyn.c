/*
 * gpstimesyn.c
 *
 *  Created on: 11.01.2019
 *      Author: seeger01
 */

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



int initGPSTimesny(){
	int retval =1;
	NMEAPool = osPoolCreate(osPool(NMEAPool));
	//MessageQ for the time Stamped data
	if(NMEAPool!=NULL){
		retval=0;
	}
	NMEABuffer = osMessageCreate(osMessageQ(NMEABuffer), NULL);
	if(NMEAPool!=NULL){
		retval=0;
	}
	NemaParserTID = osThreadCreate(osThread(NemaParserThread), NULL);
	return retval;
}



void StartNemaParserThread(void const * argument) {
	uint32_t porcessedCount=0;
	osEvent evt;
	enum gps_msg latest_msg;
	while (1) {
		evt = osMessageGet(NMEABuffer, osWaitForever);
		if (evt.status == osEventMessage) {
			NMEASTamped *rptr;
			rptr = (NMEASTamped*) evt.value.p;

			//find all $ start tokens and '\n' end tokens
			int DollarIndexs[MAXNEMASENTENCECOUNT]={0};
			int NewLineIndexs[MAXNEMASENTENCECOUNT]={0};
			int DollarCount=0;
			int NewLineCount=0;
			for(int i=0;i<sizeof(rptr->NMEAMessage);i++){
				if(rptr->NMEAMessage[i]=='$'&&DollarCount<MAXNEMASENTENCECOUNT)
				{
					DollarIndexs[DollarCount]=i;
					DollarCount++;
			    }
				if(rptr->NMEAMessage[i]=='\n'&&NewLineCount<MAXNEMASENTENCECOUNT)
				{
					NewLineIndexs[NewLineCount]=i;
					NewLineCount++;
			    }
			}
			// for minimal len
			for(int i=0;i<MAXNEMASENTENCECOUNT;i++)
			{
				if((NewLineIndexs[i]-DollarIndexs[i])<NMEAMINLEN){
					NewLineIndexs[i]=0;
					DollarIndexs[i]=0;
				}
			}
			for(int i=0;i<MAXNEMASENTENCECOUNT;i++)
			{
				if(DollarIndexs[i]!=0){
				latest_msg=lgw_parse_nmea(&(rptr->NMEAMessage[DollarIndexs[i]]),NewLineIndexs[i]-DollarIndexs[i]);
				}
			}


		    	//latest_msg=lgw_parse_nmea(&(rptr->NMEAMessage[Dollar]),NewLine-Dollar);
			osPoolFree(NMEAPool, rptr);
			porcessedCount++;
		}

	}

	osThreadTerminate(NULL);
}
