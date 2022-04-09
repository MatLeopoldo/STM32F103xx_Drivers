/*
 * hal_i2c.c
 *
 *  Created on: 2 de abr de 2022
 *      Author: matheus
 */
#include <assert.h>
#include "hal_i2c.h"
#include "hal_rcc.h"
#include "utils_defs.h"


#define SM_FREQ_MAX (100000U)
#define FM_FREQ_MAX (400000U)

#define I2C_FREQ_MAX 	(50000000U)
#define I2C_FREQ_MIN_SM (2000000U)
#define I2C_FREQ_MIN_FM	(4000000U)



void I2C_Init(I2C_Handle_t *const pI2CHandle)
{
	uint16_t CCR;
	uint32_t APB1Clk;

	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	/* Get I2C Input Clock */
	APB1Clk = RCC_GetAPBClock(APB1_NUM);

	/* Set input clock frequency value (APB1 Clock) */
	pI2CHandle->pI2Cx->CR2 &= ~((uint32_t) (0x3FU << I2C_CR2_FREQ));
	pI2CHandle->pI2Cx->CR2 |=
			(uint32_t)(CONVERT_HZ_TO_MHZ(APB1Clk) << I2C_CR2_FREQ) & 0x3FU;

	/* Configure I2C Master Clock */
	if((pI2CHandle->Config.MasterClock <= SM_FREQ_MAX) &&
		(APB1Clk >= I2C_FREQ_MIN_SM))
	{
		/* Standard Mode */
		BIT_CLR(pI2CHandle->pI2Cx->CCR, I2C_CCR_FS);
		CCR = (uint16_t)(APB1Clk / (2 * pI2CHandle->Config.MasterClock)) & 0x7FFU;

	}
	else if((pI2CHandle->Config.MasterClock <= FM_FREQ_MAX) &&
			(APB1Clk >= I2C_FREQ_MIN_FM))
	{
		/* Fast Mode */
		BIT_SET(pI2CHandle->pI2Cx->CCR, I2C_CCR_FS);

		if(pI2CHandle->Config.DutyFastMode == I2C_DUTY_2)
		{
			/* Duty Cycle: Tlow = Thigh */
			BIT_CLR(pI2CHandle->pI2Cx->CCR, I2C_CCR_FS);
			CCR = (uint16_t)(APB1Clk / (2 * pI2CHandle->Config.MasterClock)) & 0x7FFU;
		}
		else
		{
			/* Duty Cycle: 9*Tlow = 16*Thigh */
			BIT_SET(pI2CHandle->pI2Cx->CCR, I2C_CCR_FS);
			CCR = (uint16_t)(APB1Clk / (25 * pI2CHandle->Config.MasterClock)) & 0x7FFU;
		}
	}
	else
	{
		/* ERROR */
		CCR = 0;
	}

	/* Set Master Clock Frequency */
	pI2CHandle->pI2Cx->CCR &= ~((uint32_t) 0x7FFU << I2C_CCR_CCR);
	pI2CHandle->pI2Cx->CCR |= ((uint32_t) CCR  << I2C_CCR_CCR);

	/* Configure Slave Address */
	if(pI2CHandle->Config.AddrMode == I2C_ADDR_7BITS)
	{
		BIT_CLR(pI2CHandle->pI2Cx->OAR1, I2C_OAR1_ADDMODE);
		pI2CHandle->pI2Cx->OAR1 &= ~((uint32_t) (0x7FU << I2C_OAR1_ADD1));
		pI2CHandle->pI2Cx->OAR1 |=
				(uint32_t) ((pI2CHandle->Config.SlaveAddr & 0x7FU) << I2C_OAR1_ADD1);
	}
	else
	{
		BIT_SET(pI2CHandle->pI2Cx->OAR1, I2C_OAR1_ADDMODE);
		pI2CHandle->pI2Cx->OAR1 &= ~((uint32_t) (0x3FFU << I2C_OAR1_ADD0));
		pI2CHandle->pI2Cx->OAR1 |=
				(uint32_t) ((pI2CHandle->Config.SlaveAddr & 0x3FFU) << I2C_OAR1_ADD0);
	}

	/* Configure ACK Mode */
	if(pI2CHandle->Config.AckMode == I2C_ACK_EN)
	{
		BIT_SET(pI2CHandle->pI2Cx->CR1, I2C_CR1_ACK);
	}
	else
	{
		BIT_CLR(pI2CHandle->pI2Cx->CR1, I2C_CR1_ACK);
	}

	/* Configure Clock Stretching */
	if(pI2CHandle->Config.ClkStretch == I2C_CLKSTRETCH_EN)
	{
		BIT_CLR(pI2CHandle->pI2Cx->CR1, I2C_CR1_NOSTRETCH);
	}
	else
	{
		BIT_SET(pI2CHandle->pI2Cx->CR1, I2C_CR1_NOSTRETCH);
	}
}


void I2C_Reset(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameters */
	assert(IS_I2C_REG_VALID(pI2Cx));

	if(pI2Cx == I2C1)
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_I2C1_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_I2C1_RST);
	}
	else
	{
		BIT_SET(RCC->APB1RSTR, RCC_APB1RSTR_I2C2_RST);
		BIT_CLR(RCC->APB1RSTR, RCC_APB1RSTR_I2C2_RST);
	}
}


void I2C_EnableClock(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameters */
	assert(IS_I2C_REG_VALID(pI2Cx));

	if(pI2Cx == I2C1)
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C1_EN);
	}
	else
	{
		BIT_SET(RCC->APB1ENR, RCC_APB1ENR_I2C2_EN);
	}
}


void I2C_DisableClock(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameters */
	assert(IS_I2C_REG_VALID(pI2Cx));

	if(pI2Cx == I2C1)
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_I2C1_EN);
	}
	else
	{
		BIT_CLR(RCC->APB1ENR, RCC_APB1ENR_I2C2_EN);
	}
}

void I2C_MasterWrite(I2C_RegDef_t *const pI2Cx, uint16_t SlaveAddr, uint8_t *pData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_NOT_NULL(pData));
	assert(IS_I2C_REG_VALID(pI2Cx));


}

void I2C_MasterRead(I2C_RegDef_t *const pI2Cx, uint16_t SlaveAddr, uint8_t *pData, uint16_t Size)
{

}

void I2C_StartCommunication(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameter */
	assert(IS_I2C_REG_VALID(pI2Cx));

	BIT_SET(pI2Cx->CR1, I2C_CR1_PE);
}

void I2C_StopCommunication(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameter */
	assert(IS_I2C_REG_VALID(pI2Cx));

	/* Wait until the peripheral is not busy. */
	while(!BIT_TST(pI2Cx->SR2, I2C_SR2_BUSY));

	BIT_CLR(pI2Cx->CR1, I2C_CR1_PE);
}



