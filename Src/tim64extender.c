/*
 * tim64extender.c
 *
 *  Created on: 30.07.2019
 *      Author: seeger01
 */

/* Includes ------------------------------------------------------------------*/
#include "tim64extender.h"
#include "SEGGER_RTT.h"
//Top 32 bit for timer2 inputcapture values
static uint64_t tim2_update_counts = 0;
static uint64_t tim2_upper_bits_mask = 0;
//Top 48 bit for timer2 inputcapture values
static uint64_t tim4_update_counts = 0;
static uint64_t tim4_upper_bits_mask = 0;


uint64_t TIM_Get_64Bit_TimeStamp_Base(TIM_HandleTypeDef *htim) {
	uint64_t timestamp = 0;
	// RACE CONDITION CECKING !!
	// this code occures as well in  void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) in stm32f7xx_hal_tim.c
	// this function is called every time an Timer generates an interrupt event and adds it do the Nested Vectored Interrupt Controller (NVIC)
	// the timer generates an global interrupt(IRQ#44 TIM2_IRQHandler) witch is always the same regardless the reason for the the interrupt (input capture, output compare, upadte)
	// so the HAL_TIM_IRQHandler() is called to determin the source of the ISR request, there fore the startus registers of the timer are read.
	// in that order:
	// /* Capture compare 1 event */
	// /* Capture compare 2 event */
	// /* Capture compare 3 event */
	// /* Capture compare 4 event */
	// /* TIM Update event */
	// ...
	// if an update and an inputcapure event occures while jumping or processing  in the HAL_TIM_IRQHandler() then the inputcapure event will be
	// procesed before the update event and the tim2_upper_bits_mask is not set right so we have to check the timer values and decive if they are from befor or after the overflow.
	uint64_t tim2_upper_bits_mask_race_condition = 0;
	bool tim2_race_condition_up_date = false;
	if (htim->Instance == TIM2) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
				//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
				SEGGER_RTT_printf(0,"WARNING!!! TIM2 TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
				tim2_race_condition_up_date = true;
				tim2_upper_bits_mask_race_condition = tim2_upper_bits_mask;
				tim2_update_counts++;
				tim2_upper_bits_mask = (uint64_t) (tim2_update_counts - 1)<< 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
	}
	uint64_t tim4_upper_bits_mask_race_condition = 0;
	bool tim4_race_condition_up_date = false;
	if (htim->Instance == TIM4) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
				//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
				SEGGER_RTT_printf(0,"WARNING!!! TIM4 TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
				tim4_race_condition_up_date = true;
				tim4_upper_bits_mask_race_condition = tim4_upper_bits_mask;
				tim4_update_counts++;
				tim4_upper_bits_mask = (uint64_t) (tim4_update_counts - 1)
						<< 16;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
	}
	if (htim->Instance == TIM4) {
		uint32_t MSB = __HAL_TIM_GetCounter(&htim4);
		uint32_t LSB = __HAL_TIM_GetCounter(&htim1) + 1;
		uint64_t timestamp_raw = (MSB << 16)+LSB;
		if (tim4_race_condition_up_date == false) {
			//this is the nromal case
			timestamp = tim4_upper_bits_mask+timestamp_raw;
		} else {
			if (timestamp_raw < TIM32OLDTIMERVALMIN)
			//the timer has overflowen tak the updateted bitmask
			{
				timestamp = tim4_upper_bits_mask +  timestamp_raw;
			} else {
				//this is an old value using the old bitmask
				timestamp = tim4_upper_bits_mask_race_condition+timestamp_raw;
			}
			tim4_race_condition_up_date = false;
		}

	}

	if (htim->Instance == TIM2) {
		uint64_t timestamp_raw = __HAL_TIM_GetCounter(&htim2);
		if (tim2_race_condition_up_date == false) {
			//this is the nromal case
			timestamp = tim2_upper_bits_mask+timestamp_raw;
		} else {
			if (timestamp_raw < TIM32OLDTIMERVALMIN)
			//the timer has overflowen take the updateted bitmask
			{
				timestamp = tim2_upper_bits_mask+timestamp_raw;
			} else {
				//this is an old value using the old bitmask
				timestamp = tim2_upper_bits_mask_race_condition+timestamp_raw;
			}
			tim2_race_condition_up_date = false;
		}

	}
	return timestamp;
}

uint64_t TIM_Get_64Bit_TimeStamp_IC(TIM_HandleTypeDef *htim) {

	//TODO add channel mapping
	uint64_t timestamp = 0;
	// RACE CONDITION CECKING !!
	// this code occures as well in  void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) in stm32f7xx_hal_tim.c
	// this function is called every time an Timer generates an interrupt event and adds it do the Nested Vectored Interrupt Controller (NVIC)
	// the timer generates an global interrupt(IRQ#44 TIM2_IRQHandler) witch is always the same regardless the reason for the the interrupt (input capture, output compare, upadte)
	// so the HAL_TIM_IRQHandler() is called to determin the source of the ISR request, there fore the startus registers of the timer are read.
	// in that order:
	// /* Capture compare 1 event */
	// /* Capture compare 2 event */
	// /* Capture compare 3 event */
	// /* Capture compare 4 event */
	// /* TIM Update event */
	// ...
	// if an update and an inputcapure event occures while jumping or processing  in the HAL_TIM_IRQHandler() then the inputcapure event will be
	// procesed before the update event and the tim2_upper_bits_mask is not set right so we have to check the timer values and decive if they are from befor or after the overflow.
#define TIM32OLDTIMERVALMIN 0xFF000000 // if an inputcaputure value is biger than this its prppably an old one
	static uint64_t tim2_upper_bits_mask_race_condition = 0;
	static bool tim2_race_condition_up_date = false;
	static uint64_t tim4_upper_bits_mask_race_condition = 0;
	static bool tim4_race_condition_up_date = false;

	if (htim->Instance == TIM2) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
			SEGGER_RTT_printf(0,"WARNING!!! TIM2 TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
			tim2_race_condition_up_date = true;
			tim2_upper_bits_mask_race_condition = tim2_upper_bits_mask;
			tim2_update_counts++;
			tim2_upper_bits_mask = (uint64_t) (tim2_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
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

			if (tim2_race_condition_up_date == false) {
				//this is the nromal case
				timestamp = tim2_upper_bits_mask +timestamp_raw;
			} else {
				if (timestamp_raw < TIM32OLDTIMERVALMIN)
				//the timer has overflowen tak the updateted bitmask
				{
					timestamp = tim2_upper_bits_mask + (uint64_t) timestamp_raw;
				} else {
					//this is an old value using the old bitmask
					timestamp = tim2_upper_bits_mask_race_condition
							+ (uint64_t) timestamp_raw;
				}
				tim2_race_condition_up_date = false;
			}

	}

	if (htim->Instance == TIM4) {
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
			if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
			//the flag gets cleared to prevent HAL_TIM_PeriodElapsedCallback() calling and therfore double increasment
			SEGGER_RTT_printf(0,"WARNING!!! TIM4 TIMER OVERFLOW DETECTED OUTSIDE OF UPDATEEVENTHANDLER\n START SPECIAL HANDLING CHECK RESULTS OF THIS MESURMENT CYCLE");
			tim4_race_condition_up_date = true;
			tim4_upper_bits_mask_race_condition = tim4_upper_bits_mask;
			tim4_update_counts++;
			tim4_upper_bits_mask = (uint64_t) (tim4_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1
			}
		}
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


	if (tim4_race_condition_up_date == false) {
		//this is the nromal case
		timestamp = tim4_upper_bits_mask + (MSB << 16)+LSB;
	} else {
		uint32_t timestamp_raw = (MSB << 16)+LSB;
		if (timestamp_raw < TIM32OLDTIMERVALMIN)
		//the timer has overflowen tak the updateted bitmask
		{
			timestamp = tim4_upper_bits_mask + (uint64_t) timestamp_raw;
		} else {
			//this is an old value using the old bitmask
			timestamp = tim4_upper_bits_mask_race_condition
					+ (uint64_t) timestamp_raw;
		}
		tim4_race_condition_up_date = false;
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
	if (htim->Instance == TIM2) {
		tim2_update_counts++;
		tim2_upper_bits_mask = (uint64_t) (tim2_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1

	}
	if (htim->Instance == TIM4) {
		{
			tim4_update_counts++;
			tim4_upper_bits_mask = (uint64_t) (tim4_update_counts - 1) << 32;//timer gets initaled with set upodateflag but we want to start at zero therfore -1

		}
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}
