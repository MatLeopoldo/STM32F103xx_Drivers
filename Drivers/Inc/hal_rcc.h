/*
 * hal_rcc.h
 *
 *  Created on: 2 de abr de 2022
 *      Author: matheus
 */

#ifndef HAL_RCC_H_
#define HAL_RCC_H_

#include <stdint.h>
#include "stm32f103xx.h"

#define HSI_OSC_CLOCK	(8000000U)
#define HSE_OSC_CLOCK	(8000000U) /* It depends on the crystal oscillator value */

#define APB1_NUM	(1U)
#define APB2_NUM	(2U)

typedef enum
{
	HSI_CLK_SOURCE = 0,
	HSE_CLK_SOURCE,
	PLL_CLK_SOURCE
} RCC_ClockSource_enum;

typedef enum
{
	AHB_SYSCLK_DIV2 = 0x08,
	AHB_SYSCLK_DIV4,
	AHB_SYSCLK_DIV8,
	AHB_SYSCLK_DIV16,
	AHB_SYSCLK_DIV64,
	AHB_SYSCLK_DIV128,
	AHB_SYSCLK_DIV256,
	AHB_SYSCLK_DIV512
} RCC_AHBPrescaler_enum;

typedef enum
{
	APB_HCLK_DIV2 = 0x04,
	APB_HCLK_DIV4,
	APB_HCLK_DIV8,
	APB_HCLK_DIV16
} RCC_APBPrescaler_enum;


uint32_t RCC_GetSysClock(void);
uint32_t RCC_GetAHBClock(void);
uint32_t RCC_GetAPBClock(uint8_t APBNum);

#endif /* HAL_RCC_H_ */
