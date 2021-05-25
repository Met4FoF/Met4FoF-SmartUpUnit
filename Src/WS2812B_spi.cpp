/*
 * WS2812B_spi.cpp
 *
 *  Created on: 20.05.2021
 *      Author: benes
 */


#include "WS2812B_spi.h"
#include <stdlib.h>
WS2812B::WS2812B(SPI_HandleTypeDef* spi,uint16_t numLeds)
{
	  _numLeds=numLeds;
	  ledData=(uint8_t *)malloc(numLeds*3);
	  outputData=(uint8_t *)malloc(24+numLeds*9+2);
	  _spi=spi;
	  if(ledData!=NULL and outputData!=NULL)
	  {
	  bufferAlloced=true;
	  }

}

void WS2812B::setLed(uint16_t led,uint8_t r,uint8_t g,uint8_t b)
{
	ledData[led*3]=r;
	ledData[led*3+1]=g;
	ledData[led*3+2]=b;
}
void WS2812B::calculateData()
{
	outputData[0]=0xFF;
	outputData[sizeof(outputData)-1]=0xFF;
for(uint32_t i=0;i<_numLeds;i++)
{
	uint32_t tmp=mask;
	uint32_t DataPos3=0;
	//loop over evry bit in the data byte
	for(uint8_t j=0;j<8;j++){
		uint32_t DataBit=ledData[i]&(1<<j);
		if(DataBit>0){
			DataPos3=1<<(3*j+1);// set bit at pos 3j+1 to 1 this is the pos Dj in  ID7O ID6O ID5O ID4O ID3O ID2O ID1O ID0O
		}
		else
		{
			DataPos3=0;
		}
		tmp=tmp|DataPos3;
	}
	outputData[i*3+2+23]=tmp; //LSB here send last        +23 for reset preamble
	outputData[i*3+1+23]=tmp>>8;						//+23 for reset preamble
	outputData[i*3+0+23]=tmp>>16; //MSB here send first   +23 for reset preamble
}
}


