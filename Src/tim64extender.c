/*
 * tim64extender.c
 *
 *  Created on: 30.07.2019
 *      Author: seeger01
 */

/* Includes ------------------------------------------------------------------*/
#include "tim64extender.h"
#include "SEGGER_RTT.h"
//Top 32 bit for inputcapture values
static uint64_t tim_update_counts = 0;
static uint64_t tim_upper_bits_mask = 0;
static bool tim2justOverflown=false;
static bool tim4justOverflown=false;


uint64_t TIM_Get_64Bit_TimeStamp_Base(TIM_HandleTypeDef *htim) {
	uint64_t timestamp=0;
	if (htim->Instance == TIM4) {
		uint32_t MSB = __HAL_TIM_GetCounter(&htim4);
		uint32_t LSB = __HAL_TIM_GetCounter(&htim1) + 1;
		uint64_t timestamp_raw = (MSB << 16)+LSB;
		if(tim4justOverflown==false){
			timestamp = tim_upper_bits_mask + timestamp_raw;
		}
		else{
			SEGGER_RTT_printf(0,"INFO TIM4  First Capture after Update\n");
			tim4justOverflown=false;
			if (timestamp_raw>TIM32OLDTIMERVALMIN){
				timestamp = (tim_upper_bits_mask-1) + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO Old Value\n");
			}
			else
			{
				timestamp = tim_upper_bits_mask + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO New Value\n");
			}

		}


	}

	if (htim->Instance == TIM2) {
		uint64_t timestamp_raw = __HAL_TIM_GetCounter(&htim2);
		if(tim2justOverflown==false){
			timestamp = tim_upper_bits_mask + timestamp_raw;
		}
		else{
			SEGGER_RTT_printf(0,"INFO TIM2  First Capture after Update\n");
			tim2justOverflown=false;
			if (timestamp_raw>TIM32OLDTIMERVALMIN){
				timestamp = (tim_upper_bits_mask-1) + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO Old Value\n");
			}
			else
			{
				timestamp = tim_upper_bits_mask + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO New Value\n");
			}

		}

		}
	return timestamp;
}

uint64_t TIM_Get_64Bit_TimeStamp_IC(TIM_HandleTypeDef *htim) {
	uint64_t timestamp=0;
	if (htim->Instance == TIM2) {
		uint32_t timestamp_raw=0;
		switch (htim->Channel) {
		case HAL_TIM_ACTIVE_CHANNEL_1:
			timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
			timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			timestamp_raw = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
			break;
		default:
			timestamp_raw = 0;
			SEGGER_RTT_printf(0,"WARNING!!! TIM2 UNHANDLED CHANNEL AS INTERRUPT SOURCE\n");
		}
		if(tim2justOverflown==false){
			timestamp = tim_upper_bits_mask + timestamp_raw;
		}
		else{
			SEGGER_RTT_printf(0,"INFO TIM2  First Capture after Update\n");
			tim2justOverflown=false;
			if (timestamp_raw>TIM32OLDTIMERVALMIN){
				timestamp = (tim_upper_bits_mask-1) + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO Old Value\n");
			}
			else
			{
				timestamp = tim_upper_bits_mask + timestamp_raw;
				SEGGER_RTT_printf(0,"INFO New Value\n");
			}

		}
	}
	if (htim->Instance == TIM4) {
		uint32_t MSB = 0;
		uint32_t LSB = 0;
		switch (htim->Channel) {
		case HAL_TIM_ACTIVE_CHANNEL_1:
			//this is the nromal case
			MSB = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
			LSB = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1) + 2;
			break;

		case HAL_TIM_ACTIVE_CHANNEL_2:
			//this is the nromal case
			MSB = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
			LSB = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2) + 2;
			break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
			//this is the nromal case
			MSB = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
			LSB = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3) + 2;
			break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
			//this is the nromal case
			MSB = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
			LSB = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4) + 2;
			break;
		default:
			MSB = 0;
			LSB = 0;
			SEGGER_RTT_printf(0,"WARNING!!! TIM4 UNHANDLED CHANNEL AS INTERRUPT SOURCE\n");
		}
		uint32_t timestamp_raw=(MSB << 16)+LSB;
if(tim4justOverflown==false){
	timestamp = tim_upper_bits_mask + timestamp_raw;
}
else{
	SEGGER_RTT_printf(0,"INFO TIM4  First Capture after Update\n");
	tim4justOverflown=false;
	if (timestamp_raw>TIM32OLDTIMERVALMIN){
		timestamp = (tim_upper_bits_mask-1) + timestamp_raw;
		SEGGER_RTT_printf(0,"INFO Old Value\n");
	}
	else
	{
		timestamp = tim_upper_bits_mask + timestamp_raw;
		SEGGER_RTT_printf(0,"INFO New Value\n");
	}

}

	}
	return timestamp;
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM14) {
		HAL_IncTick();
	}

	if (htim->Instance == TIM4) { // TIM2 is'nt handled since TIM4 and TIM2 are synchronous
		tim_update_counts++;
		tim_upper_bits_mask = (uint64_t) (tim_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
		tim2justOverflown=true;
		tim4justOverflown=true;
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}
