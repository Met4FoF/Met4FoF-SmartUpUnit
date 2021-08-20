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
#define TIM32OLDTIMERVALMIN 0xFF000000 // if an inputcaputure value is biger than this its prppably an old one
#define TIM16OLDTIMERVALMIN 0xF000 // if an inputcaputure value is biger than this its prppably an old one
uint64_t TIM_Get_64Bit_TimeStamp_IC(TIM_HandleTypeDef * htim);
uint64_t TIM_Get_64Bit_TimeStamp_Base(TIM_HandleTypeDef * htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
#endif /* TIM64EXTENDER_H_ */
