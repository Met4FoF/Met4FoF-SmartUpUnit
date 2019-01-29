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
	//NMEASTamped DebugData;
	while (1) {
		evt = osMessageGet(NMEABuffer, osWaitForever);
		if (evt.status == osEventMessage) {
			NMEASTamped *rptr;
			rptr = (NMEASTamped*) evt.value.p;
			//memcpy(&DebugData, &*rptr, sizeof(NMEASTamped));
		    int Dollar=0;
		    int NewLine=1;
		    bool StartParsing=true;
		    while(StartParsing)
		    {
		    for(int i=0;i<sizeof(rptr->NMEAMessage)-Dollar;i++){
		    	if(rptr->NMEAMessage[i+Dollar]=='$'){
		    		Dollar=Dollar+i;
		    		break;
		    	}
		    	if(i+Dollar==sizeof(rptr->NMEAMessage)-8){
		    		StartParsing=false;
		    		break;
		    	}
		    }
		    for(int i=0;i<sizeof(rptr->NMEAMessage)-NewLine;i++){
		    	if(rptr->NMEAMessage[i+NewLine]=='\n'){
		    		NewLine=NewLine+i;
		    		break;
		    	}
		    	if(i+NewLine==sizeof(rptr->NMEAMessage)-8){
		    		StartParsing=false;
		    		break;
		    	}
		    }
		    if(StartParsing){
		    	latest_msg=lgw_parse_nmea(&(rptr->NMEAMessage[Dollar]),NewLine-Dollar);
		    }
		    NewLine++;
		    Dollar++;
		    }
			osPoolFree(NMEAPool, rptr);
			porcessedCount++;
		}

	}

	osThreadTerminate(NULL);
}


