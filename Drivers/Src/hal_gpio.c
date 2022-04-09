/*
 * hal_gpio.c
 *
 *  Created on: 15 de jan de 2022
 *      Author: matheus
 */

#include <assert.h>
#include "hal_gpio.h"
#include "utils_defs.h"

/* Local Functions Prototypes */
static void GPIO_ConfigInterrupt(GPIO_Handle_t *const pGPIOPin);


void GPIO_InitPin(GPIO_Handle_t *const pGPIOPin)
{
	uint8_t RegIndex;
	uint8_t RegPos;

	/* Check the parameters */
	assert(IS_NOT_NULL(pGPIOPin));
	assert(IS_GPIO_PORT_VALID(pGPIOPin->pGPIOx));
	assert(IS_GPIO_PIN_VALID(pGPIOPin->Number));

	/* Get GPIO_CR Index (CRL or CRH) */
	RegIndex = pGPIOPin->Number / NUM_CR_REG_PINS;

	/* Get GPIO_CR Pin Position */
	RegPos = (pGPIOPin->Number % NUM_CR_REG_PINS) * CR_PIN_DATA_LEN;

	/* Clear the corresponding bits */
	pGPIOPin->pGPIOx->CR[RegIndex] &= (uint32_t) ~(0x0000000FU << RegPos);

	switch(pGPIOPin->Mode)
	{
		case GPIO_OUT_PUSH_PULL:
		{
			pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)((GPIO_MODE_OUTPUT_PP + (pGPIOPin->Speed & 0x03)) << RegPos);
			break;
		}

		case GPIO_OUT_OPEN_DRAIN:
		{
			pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)((GPIO_MODE_OUTPUT_OD + (pGPIOPin->Speed & 0x03)) << RegPos);
			break;
		}

		case AFIO_OUT_PUSH_PULL:
		{
			pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)((AFIO_MODE_OUTPUT_PP + (pGPIOPin->Speed & 0x03)) << RegPos);
			break;
		}

		case AFIO_OUT_OPEN_DRAIN:
		{
			pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)((AFIO_MODE_OUTPUT_OD + (pGPIOPin->Speed & 0x03)) << RegPos);
			break;
		}

		case GPIO_IN_ANALOG:
		{
			pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)(GPIO_MODE_INPUT_AN << RegPos);
			break;
		}

		case GPIO_IN_FLOATING:
		case GPIO_ITR_FALLING:
		case GPIO_ITR_RISING:
		case GPIO_ITR_FALL_RIS:
		{
			/* Configure Pin Mode and Pull Mode */
			switch(pGPIOPin->PullMode)
			{
				case GPIO_PULL_UP:
				{
					pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)(GPIO_MODE_INPUT_PULL << RegPos);
					/* Enable Pull-up */
					BIT_SET(pGPIOPin->pGPIOx->ODR, pGPIOPin->Number);
					break;
				}


				case GPIO_PULL_DOWN:
				{
					pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)(GPIO_MODE_INPUT_PULL << RegPos);
					/* Enable Pull-down */
					BIT_CLR(pGPIOPin->pGPIOx->ODR, pGPIOPin->Number);
					break;
				}

				case GPIO_PULL_DISABLED:
				default:
				{
					pGPIOPin->pGPIOx->CR[RegIndex] |= (uint32_t)(GPIO_MODE_INPUT_FL << RegPos);
				}
			}

			/* COnfigure Pin Interruption (if necessary) */
			if(pGPIOPin->Mode >= GPIO_ITR_FALLING)
			{
				GPIO_ConfigInterrupt(pGPIOPin);
			}

			break;
		}

		default:
		{
			// Do nothing
		}
	}

}


void GPIO_ResetReg(GPIO_RegDef_t *const pGPIOx)
{
	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));

	if(pGPIOx == GPIOA)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPA_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPA_RST);
	}
	else if(pGPIOx == GPIOB)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPB_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPB_RST);
	}
	else if(pGPIOx == GPIOC)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPC_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPC_RST);
	}
	else if(pGPIOx == GPIOD)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPD_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPD_RST);
	}
	else if(pGPIOx == GPIOE)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPE_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPE_RST);
	}
	else if(pGPIOx == GPIOF)
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPF_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPF_RST);
	}
	else
	{
		BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_IOPG_RST);
		BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_IOPG_RST);
	}
}


void AFIO_ResetReg(void)
{
	BIT_SET(RCC->APB2RSTR, RCC_APB2RSTR_AFIO_RST);
	BIT_CLR(RCC->APB2RSTR, RCC_APB2RSTR_AFIO_RST);
}


void GPIO_EnableClock(GPIO_RegDef_t *const pGPIOx)
{
	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));

	if(pGPIOx == GPIOA)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPA_EN);
	}
	else if(pGPIOx == GPIOB)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPB_EN);
	}
	else if(pGPIOx == GPIOC)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPC_EN);
	}
	else if(pGPIOx == GPIOD)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPD_EN);
	}
	else if(pGPIOx == GPIOE)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPE_EN);
	}
	else if(pGPIOx == GPIOF)
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPF_EN);
	}
	else
	{
		BIT_SET(RCC->APB2ENR, RCC_APB2ENR_IOPG_EN);
	}
}

void GPIO_DisableClock(GPIO_RegDef_t *const pGPIOx)
{
	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));

	if(pGPIOx == GPIOA)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPA_EN);
	}
	else if(pGPIOx == GPIOB)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPB_EN);
	}
	else if(pGPIOx == GPIOC)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPC_EN);
	}
	else if(pGPIOx == GPIOD)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPD_EN);
	}
	else if(pGPIOx == GPIOE)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPE_EN);
	}
	else if(pGPIOx == GPIOF)
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPF_EN);
	}
	else
	{
		BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_IOPG_EN);
	}
}


void AFIO_EnableClock(void)
{
	BIT_SET(RCC->APB2ENR, RCC_APB2ENR_AFIO_EN);
}


void AFIO_DisableClock(void)
{
	BIT_CLR(RCC->APB2ENR, RCC_APB2ENR_AFIO_EN);
}


void GPIO_Write(GPIO_RegDef_t *const pGPIOx, uint8_t Pin, uint8_t Value)
{
	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));
	assert(IS_GPIO_PIN_VALID(Pin));

	if(Value == HIGH)
	{
		BIT_SET(pGPIOx->ODR, Pin);
	}
	else
	{
		BIT_CLR(pGPIOx->ODR, Pin);
	}

}


void GPIO_Toggle(GPIO_RegDef_t *const pGPIOx, uint8_t Pin)
{
	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));
	assert(IS_GPIO_PIN_VALID(Pin));

	BIT_TOGGLE(pGPIOx->ODR, Pin);

}


uint8_t GPIO_Read(GPIO_RegDef_t *const pGPIOx, uint8_t Pin)
{
	uint8_t Value = 0;

	/* Check data consistency */
	assert(IS_GPIO_PORT_VALID(pGPIOx));
	assert(IS_GPIO_PIN_VALID(Pin));

	Value = (uint8_t) (pGPIOx->IDR >> Pin) & 0x01U;

	return Value;
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	/* Check data consistency */
	assert(IS_GPIO_PIN_VALID(PinNumber));

	BIT_SET(EXTI->PR, PinNumber);
}

/********************************
 * Auxiliary Functions
 ********************************/
static void GPIO_ConfigInterrupt(GPIO_Handle_t *const pGPIOPin)
{
	uint8_t ConfValue;
	uint8_t RegIndex;
	uint8_t ConfPos;

	AFIO_EnableClock();

	/* Unmask pin interrupt line */
	BIT_SET(EXTI->IMR, pGPIOPin->Number);

	/* Configure Trigger */
	if(pGPIOPin->Mode == GPIO_ITR_FALLING)
	{
		BIT_SET(EXTI->FTSR, pGPIOPin->Number);
		BIT_CLR(EXTI->RTSR, pGPIOPin->Number);
	}
	else if(pGPIOPin->Mode == GPIO_ITR_RISING)
	{
		BIT_SET(EXTI->RTSR, pGPIOPin->Number);
		BIT_CLR(EXTI->FTSR, pGPIOPin->Number);
	}
	else
	{
		BIT_SET(EXTI->RTSR, pGPIOPin->Number);
		BIT_SET(EXTI->FTSR, pGPIOPin->Number);
	}

	/* Get AFIO_EXTICR index (1 to 4) */
	RegIndex =  pGPIOPin->Number / NUM_EXTICR_REG_PINS;

	/* Get config value start bit position */
	ConfPos = (pGPIOPin->Number % NUM_EXTICR_REG_PINS) * EXTICR_PIN_DATA_LEN;

	/* Select Pin Port to control the Interrupt Line */
	ConfValue = GET_GPIO_PORT_INDEX(pGPIOPin->pGPIOx);

	AFIO->EXTICR[RegIndex] &= (uint32_t) ~(0x0FU << ConfPos);
	AFIO->EXTICR[RegIndex] |= (uint32_t) (ConfValue << ConfPos);

}
