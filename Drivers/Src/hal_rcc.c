/*
 * hal_rcc.c
 *
 *  Created on: 2 de abr de 2022
 *      Author: matheus
 */
#include "hal_rcc.h"
#include "utils_defs.h"

#define PLL_MUL_MAX_VALUE (16U)

/* Local Functions Prototypes */
static uint32_t RCC_GetPLLClock(void);


uint32_t RCC_GetSysClock(void)
{
	uint8_t ClkSource;

	ClkSource = (uint8_t)(RCC->CFGR >> RCC_CFGR_SWS) & 0x03;

	switch(ClkSource)
	{
		case HSI_CLK_SOURCE:
		{
			return HSI_OSC_CLOCK;
		}
		case HSE_CLK_SOURCE:
		{
			return HSE_OSC_CLOCK;
		}
		case PLL_CLK_SOURCE:
		{
			return RCC_GetPLLClock();
		}
		default:
		{
			/* An error occurred */
			return 0;
		}
	}
}

uint32_t RCC_GetAHBClock(void)
{
	uint8_t AHBPrescaler;

	/* Get AHB Prescaler */
	AHBPrescaler = (uint8_t)(RCC->CFGR >> RCC_CFGR_HRPE) & 0x0F;

	switch(AHBPrescaler)
	{
		case AHB_SYSCLK_DIV2:
		{
			return RCC_GetSysClock() / 2;
		}
		case AHB_SYSCLK_DIV4:
		{
			return RCC_GetSysClock() / 4;
		}
		case AHB_SYSCLK_DIV8:
		{
			return RCC_GetSysClock() / 8;
		}
		case AHB_SYSCLK_DIV16:
		{
			return RCC_GetSysClock() / 16;
		}
		case AHB_SYSCLK_DIV64:
		{
			return RCC_GetSysClock() / 64;
		}
		case AHB_SYSCLK_DIV128:
		{
			return RCC_GetSysClock() / 128;
		}
		case AHB_SYSCLK_DIV256:
		{
			return RCC_GetSysClock() / 256;
		}
		case AHB_SYSCLK_DIV512:
		{
			return RCC_GetSysClock() / 512;
		}
		default:
		{
			return RCC_GetSysClock();
		}
	}
}

uint32_t RCC_GetAPBClock(uint8_t APBNum)
{
	uint8_t APBPrescaler;

	/* Get AHB Prescaler */
	if(APBNum == APB1_NUM)
	{
		APBPrescaler = (uint8_t)(RCC->CFGR >> RCC_CFGR_PRE1) & 0x07;
	}
	else if(APBNum == APB2_NUM)
	{
		APBPrescaler = (uint8_t)(RCC->CFGR >> RCC_CFGR_PRE2) & 0x07;
	}
	else
	{
		return 0;
	}

	switch(APBPrescaler)
	{
		case APB_HCLK_DIV2:
		{
			return RCC_GetAHBClock() / 2;
		}
		case APB_HCLK_DIV4:
		{
			return RCC_GetAHBClock() / 4;
		}
		case APB_HCLK_DIV8:
		{
			return RCC_GetAHBClock() / 8;
		}
		case APB_HCLK_DIV16:
		{
			return RCC_GetAHBClock() / 16;
		}
		default:
		{
			return RCC_GetAHBClock();
		}
	}
}

/************************************
 *
 * Local Functions
 *
 ************************************/
static uint32_t RCC_GetPLLClock(void)
{
	uint8_t PLLMul;
	uint32_t PLLClock;

	/* Get PLL Clock Souce */
	if(BIT_TST(RCC->CFGR, RCC_CFGR_PLL_SRC))
	{
		if(BIT_TST(RCC->CFGR, RCC_CFGR_PLL_XTPRE))
		{
			/* HSE Clock selected */
			PLLClock = HSE_OSC_CLOCK;
		}
		else
		{
			/* HSE Clock divided by 2 selected */
			PLLClock = HSE_OSC_CLOCK / 2;
		}
	}
	else
	{
		/* HSI Clock divided by 2 selected */
		PLLClock = HSI_OSC_CLOCK / 2;
	}

	/* Get PLL Multiplication Factor */
	PLLMul = (uint8_t)(RCC->CFGR >> RCC_CFGR_PLL_MUL) & 0x0F;

	/* Calculate PLL CLock */
	if(PLLMul == PLL_MUL_MAX_VALUE)
	{
		PLLClock = PLLClock * PLLMul;
	}
	else
	{
		PLLClock = PLLClock * (PLLMul + 1);
	}

	return PLLClock;
}

