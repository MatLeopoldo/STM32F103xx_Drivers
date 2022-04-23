/*
 * hal_usart.c
 *
 *  Created on: 23 de abr de 2022
 *      Author: matheus
 */
#include <assert.h>

#include "hal_usart.h"
#include "hal_rcc.h"
#include "utils_defs.h"


void USART_Init(USART_Handle_t *const pUSARTHandle)
{

}


void USART_Reset(USART_RegDef_t *const pUSARTx)
{
	/* Check the parameters */
	assert(IS_USART_REG_VALID(pUSARTx));

	if(pUSARTx == USART1)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_USART1_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_USART1_RST);
	}
	else if(pUSARTx == USART2)
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_USART2_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_USART2_RST);
	}
	else if(pUSARTx == USART3)
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_USART3_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_USART3_RST);
	}
	else if(pUSARTx == UART4)
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_UART4_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_UART4_RST);
	}
	else
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_UART5_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_UART5_RST);
	}
}


void USART_EnableClock(USART_RegDef_t *const pUSARTx)
{
	/* Check the parameters */
	assert(IS_USART_REG_VALID(pUSARTx));

	if(pUSARTx == USART1)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_USART1_EN);
	}
	else if(pUSARTx == USART2)
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART2_EN);
	}
	else if(pUSARTx == USART3)
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_USART3_EN);
	}
	else if(pUSARTx == UART4)
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_UART4_EN);
	}
	else
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_UART5_EN);
	}
}


void USART_DisableClock(USART_RegDef_t *const pUSARTx)
{
	/* Check the parameters */
	assert(IS_USART_REG_VALID(pUSARTx));

	if(pUSARTx == USART1)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_USART1_EN);
	}
	else if(pUSARTx == USART2)
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_USART2_EN);
	}
	else if(pUSARTx == USART3)
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_USART3_EN);
	}
	else if(pUSARTx == UART4)
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_UART4_EN);
	}
	else
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_UART5_EN);
	}
}
