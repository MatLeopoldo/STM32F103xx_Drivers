/*
 * hal_nvic.c
 *
 *  Created on: 22 de jan de 2022
 *      Author: matheus
 */
#include "hal_nvic.h"
#include "utils_defs.h"


void NVIC_IRQEnable(uint8_t IntrNum)
{
	uint8_t RegOffset = IntrNum / NUM_ITR_ISER_ICER;
	uint8_t RegPos = IntrNum % NUM_ITR_ISER_ICER;

	BIT_SET(*(NVIC_ISER0 + RegOffset), RegPos);
	BIT_CLR(*(NVIC_ICER0 + RegOffset), RegPos);
}

void NVIC_IRQDisable(uint8_t IntrNum)
{
	uint8_t RegOffset = IntrNum / NUM_ITR_ISER_ICER;
	uint8_t RegPos = IntrNum % NUM_ITR_ISER_ICER;

	BIT_SET(*(NVIC_ICER0 + RegOffset), RegPos);
	BIT_CLR(*(NVIC_ISER0 + RegOffset), RegPos);
}


void NVIC_IRQChangePriority(uint8_t IntrNum, uint8_t PriorVal)
{
	uint8_t RegOffset = IntrNum / NUM_ITR_IPR;
	uint8_t RegPos = (IntrNum % NUM_ITR_IPR) * IPR_ITR_LEN;

	*(NVIC_IPR0 + RegOffset) &= (uint32_t) ~(0xFFU << RegPos);
	*(NVIC_IPR0 + RegOffset) |= (uint32_t) (PriorVal << RegPos);
}


