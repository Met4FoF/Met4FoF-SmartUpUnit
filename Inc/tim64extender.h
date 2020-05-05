/*
 * tim64extender.h
 *
 *  Created on: 30.07.2019
 *      Author: seeger01
 */

#ifndef TIM64EXTENDER_H_
#define TIM64EXTENDER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "stdbool.h"

uint64_t TIM_Get_64Bit_TimeStamp_IC(TIM_HandleTypeDef * htim);
uint64_t TIM_Get_64Bit_TimeStamp_Base(TIM_HandleTypeDef * htim);
#ifdef __cplusplus
}
#endif
#endif /* TIM64EXTENDER_H_ */
