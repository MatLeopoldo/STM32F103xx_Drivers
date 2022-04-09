/*
 * hal_nvic.h
 *
 *  Created on: 22 de jan de 2022
 *      Author: matheus
 */

#ifndef INC_HAL_NVIC_H_
#define INC_HAL_NVIC_H_

#include <stdint.h>
#include "stm32f103xx.h"

#define NUM_ITR_ISER_ICER	(32U)
#define NUM_ITR_IPR			(4U)
#define IPR_ITR_LEN			(8U)


void NVIC_IRQEnable(uint8_t IntrNum);
void NVIC_IRQDisable(uint8_t IntrNum);

void NVIC_IRQChangePriority(uint8_t IntrNum, uint8_t PriorVal);

#endif /* INC_HAL_NVIC_H_ */
