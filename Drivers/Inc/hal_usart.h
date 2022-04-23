/*
 * hal_usart.h
 *
 *  Created on: 23 de abr de 2022
 *      Author: matheus
 */

#ifndef INC_HAL_USART_H_
#define INC_HAL_USART_H_

#include <stdint.h>
#include "stm32f103xx.h"

/* Expressions */
#define IS_USART_REG_VALID(USARTx) 	((USARTx == USART1) || \
									 (USARTx == USART2) || \
									 (USARTx == USART3) || \
									 (USARTx == UART4) || \
								 	 (USARTx == UART5) ? 1U : 0U)


typedef struct
{
	uint32_t BaudRate;
	uint8_t DataLen;
	uint8_t Parity;
	uint8_t StopBits;
} USART_DevConfig_t;

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_DevConfig_t Config;
} USART_Handle_t;


void USART_Init(USART_Handle_t *const pUSARTHandle);
void USART_Reset(USART_RegDef_t *const pUSARTx);

void USART_EnableClock(USART_RegDef_t *const pUSARTx);
void USART_DisableClock(USART_RegDef_t *const pUSARTx);


#endif /* INC_HAL_USART_H_ */
