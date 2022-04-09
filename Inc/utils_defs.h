/*
 * utils_defs.h
 *
 *  Created on: 15 de jan de 2022
 *      Author: matheus
 */

#ifndef UTILS_DEFS_H_
#define UTILS_DEFS_H_

#include <stdint.h>
#include <stddef.h>

#define SUCCESS (0)
#define FAILURE (1)

#define LOW		(0)
#define HIGH	(1)
#define DISABLE (LOW)
#define ENABLE 	(HIGH)

#define BIT_SET(reg, n) 	(reg |= (uint32_t) (1U << n))
#define BIT_CLR(reg, n) 	(reg &= ~((uint32_t)1U << n))
#define BIT_TST(reg, n) 	((uint8_t)(reg >> n) & 0x01U)
#define BIT_TOGGLE(reg, n) 	(reg ^= (uint32_t) (1U << n))

/* General Asserts Expressions */
#define IS_NOT_NULL(pPointer) ((pPointer != NULL) ? 1 : 0)

/* General Convertions */
#define CONVERT_HZ_TO_MHZ(x) ((uint8_t)(x / 1000000U))

#endif /* UTILS_DEFS_H_ */
