/*
 * hal_gpio.h
 *
 *  Created on: 15 de jan de 2022
 *      Author: matheus
 */

#ifndef INC_HAL_GPIO_H_
#define INC_HAL_GPIO_H_

#include "stm32f103xx.h"

/* Expressions */
#define IS_GPIO_PORT_VALID(GPIOx)  ((GPIOx == GPIOA) || \
								   (GPIOx == GPIOB) || \
								   (GPIOx == GPIOC) || \
								   (GPIOx == GPIOD) || \
								   (GPIOx == GPIOE) || \
								   (GPIOx == GPIOF) || \
								   (GPIOx == GPIOG) ? 1 : 0)

#define GET_GPIO_PORT_INDEX(GPIOx) ((GPIOx == GPIOA)? 0 : \
                                    (GPIOx == GPIOB)? 1 : \
                                    (GPIOx == GPIOC)? 2 : \
                                    (GPIOx == GPIOD)? 3 : \
                                    (GPIOx == GPIOE)? 4 : \
									(GPIOx == GPIOF)? 5 : 6 )

#define IS_GPIO_PIN_VALID(PinNumber) (PinNumber <= 15 ? 1 : 0)


#define NUM_CR_REG_PINS (8U)
#define CR_PIN_DATA_LEN (4U)

#define NUM_EXTICR_REG_PINS (4U)
#define EXTICR_PIN_DATA_LEN (4U)

#define GPIO_MODE_OUTPUT_PP		(0x00U)
#define GPIO_MODE_OUTPUT_OD		(0x04U)
#define AFIO_MODE_OUTPUT_PP		(0x08U)
#define AFIO_MODE_OUTPUT_OD		(0x0CU)
#define GPIO_MODE_INPUT_AN		(0x00U)
#define GPIO_MODE_INPUT_FL		(0x04U)
#define GPIO_MODE_INPUT_PULL	(0x08U)


typedef enum
{
	GPIO_PIN_00 = 0,
	GPIO_PIN_01,
	GPIO_PIN_02,
	GPIO_PIN_03,
	GPIO_PIN_04,
	GPIO_PIN_05,
	GPIO_PIN_06,
	GPIO_PIN_07,
	GPIO_PIN_08,
	GPIO_PIN_09,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
} GPIO_PinNum_enum;

typedef enum
{
	GPIO_OUT_PUSH_PULL = 0,
	GPIO_OUT_OPEN_DRAIN,
	AFIO_OUT_PUSH_PULL,
	AFIO_OUT_OPEN_DRAIN,
	GPIO_IN_ANALOG,
	GPIO_IN_FLOATING,
	GPIO_ITR_FALLING,
	GPIO_ITR_RISING,
	GPIO_ITR_FALL_RIS
} GPIO_PinMode_enum;

typedef enum
{
	GPIO_SPEED_MEDIUM = 1,
	GPIO_SPEED_LOW,
	GPIO_SPEED_HIGH,
} GPIO_OutSpeed_enum;

typedef enum
{
	GPIO_PULL_DISABLED = 0,
	GPIO_PULL_DOWN,
	GPIO_PULL_UP
} GPIO_PullMode_enum;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	uint8_t Number;
	uint8_t Mode;
	uint8_t Speed;
	uint8_t PullMode;
} GPIO_Handle_t;


void GPIO_InitPin(GPIO_Handle_t *const pGPIOHandle);

void GPIO_ResetReg(GPIO_RegDef_t *const pGPIOx);
void AFIO_ResetReg(void);

void GPIO_EnableClock(GPIO_RegDef_t *const pGPIOx);
void GPIO_DisableClock(GPIO_RegDef_t *const pGPIOx);
void AFIO_EnableClock(void);
void AFIO_DisableClock(void);

void GPIO_Write(GPIO_RegDef_t *const pGPIOx, uint8_t Pin, uint8_t Value);
void GPIO_Toggle(GPIO_RegDef_t *const pGPIOx, uint8_t Pin);

uint8_t GPIO_Read(GPIO_RegDef_t *const pGPIOx, uint8_t Pin);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_HAL_GPIO_H_ */
