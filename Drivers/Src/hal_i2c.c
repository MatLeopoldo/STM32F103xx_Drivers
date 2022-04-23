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

#define I2C_WRITE	(0U)
#define I2C_READ	(1U)


/* Local Functions Prototypes */
static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *const pI2Cx, uint8_t Flag);
static void I2C_GenerateStartCondition(I2C_RegDef_t *const pI2Cx);
static void I2C_SendAddr(I2C_RegDef_t *const pI2Cx, uint8_t SlaveAddr, uint8_t RWFlag);
static void I2C_ClearAddrFlag(I2C_RegDef_t *const pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *const pI2Cx);
static void I2C_ControlACK(I2C_RegDef_t *const pI2Cx, uint8_t ACKCommand);
static void I2C_StartEVHandler(I2C_Handle_t *const pI2CHandle);
static void I2C_AddressEVHandler(I2C_Handle_t *const pI2CHandle);
static void I2C_BTFEVHandler(I2C_Handle_t *const pI2CHandle);
static void I2C_TxEmptyEVHandler(I2C_Handle_t *const pI2CHandle);
static void I2C_RxNonEmptyEVHandler(I2C_Handle_t *const pI2CHandle);
static void I2C_StopDetectionEVHandler(I2C_Handle_t *const pI2CHandle);


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
	/* Check the par_t PerifState;ameters */
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

void I2C_MasterWrite(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_NOT_NULL(pData));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_SB));

	I2C_SendAddr(pI2CHandle->pI2Cx, SlaveAddr, I2C_WRITE);
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_ADDR));

	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

	while(Size > 0)
	{
		/* Wait until Tx is empty */
		while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_TXE));

		/* Put data into the DR register */
		pI2CHandle->pI2Cx->DR = (uint32_t) *pData;
		pData++;
		Size--;
	}

	/* Wait until Tx is empty and Byte Transfer was finished */
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_TXE));
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_BTF));

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_MasterRead(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size)
{
	/* Check the parameters */
	assert(IS_NOT_NULL(pData));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));


	I2C_ControlACK(pI2CHandle->pI2Cx, ENABLE);

	/* Enable Acknowledge PEC Position */
	if(Size == 2U)
	{
		BIT_SET(pI2CHandle->pI2Cx->CR1, I2C_CR1_POS);
	}

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_SB));

	I2C_SendAddr(pI2CHandle->pI2Cx, SlaveAddr, I2C_READ);
	while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_ADDR));

	/* Receive Data based on the number of bytes desired */
	if(Size == 1U)
	{
		I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);

		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		/* Wait until Rx is not empty */
		while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

		/* Get data from DR register */
		*pData = (uint8_t) pI2CHandle->pI2Cx->DR;
	}
	else if(Size == 2U)
	{
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

		I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);

		/* Wait until Data N-1 is into the shift register */
		while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_BTF));

		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		/* Get data from DR register */
		*pData = (uint8_t) pI2CHandle->pI2Cx->DR;
		pData++;
		*pData = (uint8_t) pI2CHandle->pI2Cx->DR;

		/* Disable Acknowlegde PEC Position */
		BIT_CLR(pI2CHandle->pI2Cx->CR1, I2C_CR1_POS);
	}
	else
	{
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

		while(Size > 0)
		{
			/* Wait until Rx is not empty */
			while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_RXNE));

			/* Configure the peripheral to send an NACK after the last
			 * data being received */
			if(Size == 3U)
			{
				/* Wait until Data N-1 is into the shift register */
				while(!I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_BTF));

				I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);

				/* Get data N-2 from DR register */
				*pData = (uint8_t) pI2CHandle->pI2Cx->DR;
				pData++;
				Size--;

				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			/* Get data from DR register */
			*pData = (uint8_t) pI2CHandle->pI2Cx->DR;
			pData++;
			Size--;
		}
	}
}


uint8_t I2C_MasterWriteIT(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size)
{
	uint8_t PerifState;

	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_NOT_NULL(pData));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	PerifState = pI2CHandle->TxRxState;

	if(PerifState == I2C_STATE_FREE)
	{
		/* Initialize Tx context */
		pI2CHandle->TxRxState = I2C_STATE_TXBUSY;
		pI2CHandle->TxBuffer = pData;
		pI2CHandle->TxCounter = Size;
		pI2CHandle->DevAddr = SlaveAddr;

		/* Enable Interruptions */
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITEVTEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITBUFEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITERREN);

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	}

	return PerifState;
}


uint8_t I2C_MasterReadIT(I2C_Handle_t *const pI2CHandle, uint8_t SlaveAddr, uint8_t *pData, uint16_t Size)
{
	uint8_t PerifState;

	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_NOT_NULL(pData));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	PerifState = pI2CHandle->TxRxState;

	if(PerifState == I2C_STATE_FREE)
	{
		/* Initialize Tx context */
		pI2CHandle->TxRxState = I2C_STATE_RXBUSY;
		pI2CHandle->RxBuffer = pData;
		pI2CHandle->RxDataLen = Size;
		pI2CHandle->RxCounter = Size;
		pI2CHandle->DevAddr = SlaveAddr;

		I2C_ControlACK(pI2CHandle->pI2Cx, ENABLE);

		/* Enable Acknowledge PEC Position */
		if(Size == 2U)
		{
			BIT_SET(pI2CHandle->pI2Cx->CR1, I2C_CR1_POS);
		}

		/* Enable Interruptions */
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITEVTEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITBUFEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITERREN);

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	}

	return PerifState;
}


void I2C_ControlSlaveIT(I2C_Handle_t *const pI2CHandle, uint8_t ITControl)
{
	if(ITControl == ENABLE)
	{
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITEVTEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITBUFEN);
		BIT_SET(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITERREN);
	}
	else
	{
		BIT_CLR(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITEVTEN);
		BIT_CLR(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITBUFEN);
		BIT_CLR(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITERREN);
	}

}


void I2C_EventsHandler(I2C_Handle_t *const pI2CHandle)
{
	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	if(BIT_TST(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITEVTEN))
	{
		if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_SB))
		{
			I2C_StartEVHandler(pI2CHandle);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_ADDR))
		{
			I2C_AddressEVHandler(pI2CHandle);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_BTF))
		{
			I2C_BTFEVHandler(pI2CHandle);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_STOPF))
		{
			I2C_StopDetectionEVHandler(pI2CHandle);
		}
		else if(BIT_TST(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITBUFEN))
		{
			if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_TXE))
			{
				if(BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_MSL))
				{
					I2C_TxEmptyEVHandler(pI2CHandle);
				}
				else
				{
					/* Verify if Slave device is in tx mode */
					if(BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_TRA))
					{
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
					}
				}
			}
			else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_RXNE))
			{
				if(BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_MSL))
				{
					I2C_RxNonEmptyEVHandler(pI2CHandle);
				}
				else
				{
					/* Verify if Slave device is in rx mode */
					if(!BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_TRA))
					{
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
					}
				}
			}
			else
			{
				// Do nothing
			}
		}
	}

}


void I2C_ErrorsHandler(I2C_Handle_t *const pI2CHandle)
{
	/* Check the parameters */
	assert(IS_NOT_NULL(pI2CHandle));
	assert(IS_I2C_REG_VALID(pI2CHandle->pI2Cx));

	if(BIT_TST(pI2CHandle->pI2Cx->CR2, I2C_CR2_ITERREN))
	{
		if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_BERR))
		{
			/* Clear Bus Error Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_BERR);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_BERR);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_ARLO))
		{
			/* Clear Arbitration Lost Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_ARLO);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_ARLO);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_AF))
		{
			/* Clear Acknowledge Failure Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_AF);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_AF);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_OVR))
		{
			/* Clear Overrun/Underrun  Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_OVR);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_OVR);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_PEC_ERR))
		{
			/* Clear PEC Error Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_PEC_ERR);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_PECERR);
		}
		else if(I2C_GetStatusFlag(pI2CHandle->pI2Cx, I2C_SR1_TIMEOUT))
		{
			/* Clear Timeout Flag */
			BIT_SET(pI2CHandle->pI2Cx->SR1, I2C_SR1_TIMEOUT);
			I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_TIMEOUT);
		}
		else
		{
			// Do nothing
		}
	}
}


void I2C_SlaveWrite(I2C_RegDef_t *const pI2Cx, uint8_t Data)
{
	pI2Cx->DR = (uint32_t) Data;
}


uint8_t I2C_SlaveRead(I2C_RegDef_t *const pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}


void I2C_StartCommunication(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameter */
	assert(IS_I2C_REG_VALID(pI2Cx));

	/* Enable Peripheral */
	BIT_SET(pI2Cx->CR1, I2C_CR1_PE);
}


void I2C_StopCommunication(I2C_RegDef_t *const pI2Cx)
{
	/* Check the parameter */
	assert(IS_I2C_REG_VALID(pI2Cx));

	/* Wait until the peripheral is not busy. */
	while(!BIT_TST(pI2Cx->SR2, I2C_SR2_BUSY));

	/* Disable Peripheral */
	BIT_CLR(pI2Cx->CR1, I2C_CR1_PE);
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	/* Weak Implementation that will be overwritten in the main application */
}

/************************************
 *
 * Local Functions
 *
 ************************************/
static uint8_t I2C_GetStatusFlag(I2C_RegDef_t *const pI2Cx, uint8_t Flag)
{
	return BIT_TST(pI2Cx->SR1, Flag);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *const pI2Cx)
{
	BIT_SET(pI2Cx->CR1, I2C_CR1_START);
}

static void I2C_SendAddr(I2C_RegDef_t *const pI2Cx, uint8_t SlaveAddr, uint8_t RWFlag)
{
	pI2Cx->DR = (uint32_t)(((SlaveAddr & 0x7F) << 1) + (RWFlag & 0x01));
}

static void I2C_ClearAddrFlag(I2C_RegDef_t *const pI2Cx)
{
	uint32_t RegRead;

	/* Read SR1 and SR2 Register to clear ADDR flag */
	RegRead = pI2Cx->SR1;
	RegRead = pI2Cx->SR2;

	(void)RegRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *const pI2Cx)
{
	BIT_SET(pI2Cx->CR1, I2C_CR1_STOP);
}

static void I2C_ControlACK(I2C_RegDef_t *const pI2Cx, uint8_t ACKCommand)
{
	if(ACKCommand == ENABLE)
	{
		BIT_SET(pI2Cx->CR1, I2C_CR1_ACK);
	}
	else
	{
		BIT_CLR(pI2Cx->CR1, I2C_CR1_ACK);
	}
}

static void I2C_StartEVHandler(I2C_Handle_t *const pI2CHandle)
{
	if(pI2CHandle->TxRxState == I2C_STATE_TXBUSY)
	{
		I2C_SendAddr(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
	}
	else
	{
		I2C_SendAddr(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
	}
}

static void I2C_AddressEVHandler(I2C_Handle_t *const pI2CHandle)
{
	/* Verify if it's a master reception operation of 1 byte */
	if(BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_STATE_RXBUSY)
		{
			if(pI2CHandle->RxDataLen == 1U)
			{
				I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);

				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
			}
			else if(pI2CHandle->RxDataLen == 2U)
			{
				I2C_ClearAddrFlag(pI2CHandle->pI2Cx);

				I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);
			}
			else
			{
				I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
			}
		}
		else
		{
			I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		}
	}
	else
	{
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
	}


}

static void I2C_BTFEVHandler(I2C_Handle_t *const pI2CHandle)
{
	if(BIT_TST(pI2CHandle->pI2Cx->SR2, I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_STATE_TXBUSY)
		{
			if(pI2CHandle->TxCounter == 0)
			{
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
			}
		}
		else if(pI2CHandle->TxRxState == I2C_STATE_RXBUSY)
		{
			if(pI2CHandle->RxDataLen == 2U)
			{
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxBuffer++;
				*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxBuffer++;

				/* Disable Acknowlegde PEC Position */
				BIT_CLR(pI2CHandle->pI2Cx->CR1, I2C_CR1_POS);

				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
			}
			else
			{
				I2C_ControlACK(pI2CHandle->pI2Cx, DISABLE);

				*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxBuffer++;
				pI2CHandle->RxCounter--;

				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
				pI2CHandle->RxBuffer++;
				pI2CHandle->RxCounter--;
			}
		}
		else
		{
			// Do nothing
		}
	}
}

static void I2C_StopDetectionEVHandler(I2C_Handle_t *const pI2CHandle)
{
	pI2CHandle->pI2Cx->CR1 |= 0x0000U;

	I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOPF);
}

static void I2C_TxEmptyEVHandler(I2C_Handle_t *const pI2CHandle)
{
	if(pI2CHandle->TxCounter > 0)
	{
		pI2CHandle->pI2Cx->DR = (uint32_t)*pI2CHandle->TxBuffer;
		pI2CHandle->TxBuffer++;
		pI2CHandle->TxCounter--;
	}
}

static void I2C_RxNonEmptyEVHandler(I2C_Handle_t *const pI2CHandle)
{
	if(pI2CHandle->RxDataLen == 1U)
	{
		*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxCounter--;
	}
	else if(pI2CHandle->RxDataLen > 2U)
	{
		if(pI2CHandle->RxCounter != 3U)
		{
			*pI2CHandle->RxBuffer = (uint8_t) pI2CHandle->pI2Cx->DR;
			pI2CHandle->RxBuffer++;
			pI2CHandle->RxCounter--;
		}
	}
	else
	{
		// Do nothing (Wait for BTF Event)
	}

	if(pI2CHandle->RxCounter == 0U)
	{
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
	}
}

