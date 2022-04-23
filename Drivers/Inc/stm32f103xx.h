/*
 * stm32f103xx.h
 *
 *  Created on: 12 de jan de 2022
 *      Author: matheus
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>
#include <assert.h>

/**************************
 *  ARM CORTEX-M3
 **************************/
#define NVIC_ISER0				((uint32_t *)0XE000E100U)
#define NVIC_ICER0				((uint32_t *)0XE000E180U)
#define NVIC_IPR0				((uint32_t *)0XE000E400U)

/**************************
 *  Vector Table Values
 **************************/
#define IRQ_NO_EXTI0 		(6U)
#define IRQ_NO_EXTI1 		(7U)
#define IRQ_NO_EXTI2 		(8U)
#define IRQ_NO_EXTI3 		(9U)
#define IRQ_NO_EXTI4 		(10U)
#define IRQ_NO_EXTI9_5 		(23U)
#define IRQ_NO_I2C1_EV		(31U)
#define IRQ_NO_I2C1_ER		(32U)
#define IRQ_NO_I2C2_EV		(33U)
#define IRQ_NO_I2C2_ER		(34U)
#define IRQ_NO_SPI1			(35U)
#define IRQ_NO_SPI2			(36U)
#define IRQ_NO_EXTI15_10	(40U)
#define IRQ_NO_SPI3			(51U)

/**************************
 *  AHB Registers Adresses
 **************************/
#define AHB_BASE_ADDRESS		(0x40018000U)

#define RCC_BASE_ADDRESS		(AHB_BASE_ADDRESS + 0x00009000U)


/**************************
 *  APB1 Registers Adresses
 **************************/
#define APB1_BASE_ADDRESS 		(0x40000000U)

#define SPI2_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00003800U)
#define SPI3_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00003C00U)
#define USART2_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00004400U)
#define USART3_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00004800U)
#define UART4_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00004C00U)
#define UART5_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00005000U)
#define I2C1_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00005400U)
#define I2C2_BASE_ADDRESS		(APB1_BASE_ADDRESS + 0x00005800U)


/**************************
 *  APB2 Registers Adresses
 **************************/
#define APB2_BASE_ADDRESS 		(0x40010000U)

#define AFIO_BASE_ADDRESS		(APB2_BASE_ADDRESS)
#define EXTI_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00000400U)
#define GPIOA_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00000800U)
#define GPIOB_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00000C00U)
#define GPIOC_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00001000U)
#define GPIOD_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00001400U)
#define GPIOE_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00001800U)
#define GPIOF_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00001C00U)
#define GPIOG_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00002000U)
#define SPI1_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00003000U)
#define USART1_BASE_ADDRESS		(APB2_BASE_ADDRESS + 0x00003800U)


/**************************
 *  Registers Bit Position
 **************************/

/* RCC_APB1RSTR */
#define RCC_APB1RSTR_SPI2_RST	(14U)
#define RCC_APB1RSTR_SPI3_RST	(15U)
#define RCC_APB1RSTR_USART2_RST (17U)
#define RCC_APB1RSTR_USART3_RST (18U)
#define RCC_APB1RSTR_UART4_RST 	(19U)
#define RCC_APB1RSTR_UART5_RST 	(20U)
#define RCC_APB1RSTR_I2C1_RST	(21U)
#define RCC_APB1RSTR_I2C2_RST	(22U)

/* RCC_APB1ENR */
#define RCC_APB1ENR_SPI2_EN		(14U)
#define RCC_APB1ENR_SPI3_EN		(15U)
#define RCC_APB1ENR_USART2_EN 	(17U)
#define RCC_APB1ENR_USART3_EN 	(18U)
#define RCC_APB1ENR_UART4_EN 	(19U)
#define RCC_APB1ENR_UART5_EN 	(20U)
#define RCC_APB1ENR_I2C1_EN		(21U)
#define RCC_APB1ENR_I2C2_EN		(22U)

/* RCC_APB2RSTR */
#define RCC_APB2RSTR_AFIO_RST	(0U)
#define RCC_APB2RSTR_IOPA_RST	(2U)
#define RCC_APB2RSTR_IOPB_RST	(3U)
#define RCC_APB2RSTR_IOPC_RST	(4U)
#define RCC_APB2RSTR_IOPD_RST	(5U)
#define RCC_APB2RSTR_IOPE_RST	(6U)
#define RCC_APB2RSTR_IOPF_RST	(7U)
#define RCC_APB2RSTR_IOPG_RST	(8U)
#define RCC_APB2RSTR_SPI1_RST	(12U)
#define RCC_APB2RSTR_USART1_RST (14U)

/* RCC_APB2ENR */
#define RCC_APB2ENR_AFIO_EN		(0U)
#define RCC_APB2ENR_IOPA_EN		(2U)
#define RCC_APB2ENR_IOPB_EN		(3U)
#define RCC_APB2ENR_IOPC_EN		(4U)
#define RCC_APB2ENR_IOPD_EN		(5U)
#define RCC_APB2ENR_IOPE_EN		(6U)
#define RCC_APB2ENR_IOPF_EN		(7U)
#define RCC_APB2ENR_IOPG_EN		(8U)
#define RCC_APB2ENR_SPI1_EN		(12U)
#define RCC_APB2ENR_USART1_EN 	(14U)

/* SPI_CR1 */
#define SPI_CR1_CPHA			(0U)
#define SPI_CR1_CPOL			(1U)
#define SPI_CR1_MSTR			(2U)
#define SPI_CR1_BR				(3U)
#define SPI_CR1_SPE				(6U)
#define SPI_CR1_LSB_FIRST		(7U)
#define SPI_CR1_SSI				(8U)
#define SPI_CR1_SSM				(9U)
#define SPI_CR1_RX_ONLY			(10U)
#define SPI_CR1_DFF				(11U)
#define SPI_CR1_CRC_NEXT		(12U)
#define SPI_CR1_CRC_EN			(13U)
#define SPI_CR1_BIDI_OE			(14U)
#define SPI_CR1_BIDI_MODE		(15U)

/* SPI_CR2 */
#define SPI_CR2_RXDMAEN			(0U)
#define SPI_CR2_TXDMAEN			(1U)
#define SPI_CR2_SSOE			(2U)
#define SPI_CR2_ERRIE			(5U)
#define SPI_CR2_RXNEIE			(6U)
#define SPI_CR2_TXEIE			(7U)

/* SPI_SR */
#define SPI_SR_RXNE				(0U)
#define SPI_SR_TXE				(1U)
#define SPI_SR_CHSIDE			(2U)
#define SPI_SR_UDR				(3U)
#define SPI_SR_CRC_ERR			(4U)
#define SPI_SR_MODF				(5U)
#define SPI_SR_OVR				(6U)
#define SPI_SR_BSY				(7U)

/* I2C_CR1 */
#define I2C_CR1_PE				(0U)
#define I2C_CR1_SMBUS			(1U)
#define I2C_CR1_SMBTYPE			(3U)
#define I2C_CR1_ENARP			(4U)
#define I2C_CR1_ENPEC			(5U)
#define I2C_CR1_ENGC			(6U)
#define I2C_CR1_NOSTRETCH		(7U)
#define I2C_CR1_START			(8U)
#define I2C_CR1_STOP			(9U)
#define I2C_CR1_ACK				(10U)
#define I2C_CR1_POS				(11U)
#define I2C_CR1_PEC				(12U)
#define I2C_CR1_ALERT			(13U)
#define I2C_CR1_SWRST			(15U)

/* I2C_CR2 */
#define I2C_CR2_FREQ			(0U)
#define I2C_CR2_ITERREN			(8U)
#define I2C_CR2_ITEVTEN			(9U)
#define I2C_CR2_ITBUFEN			(10U)
#define I2C_CR2_DMAEN			(11U)
#define I2C_CR2_LAST			(12U)

/* I2C_SR1 */
#define I2C_SR1_SB				(0U)
#define I2C_SR1_ADDR			(1U)
#define I2C_SR1_BTF				(2U)
#define I2C_SR1_ADD10			(3U)
#define I2C_SR1_STOPF			(4U)
#define I2C_SR1_RXNE			(6U)
#define I2C_SR1_TXE				(7U)
#define I2C_SR1_BERR			(8U)
#define I2C_SR1_ARLO			(9U)
#define I2C_SR1_AF				(10U)
#define I2C_SR1_OVR				(11U)
#define I2C_SR1_PEC_ERR			(12U)
#define I2C_SR1_TIMEOUT			(14U)
#define I2C_SR1_SMB_ALERT		(15U)

/* I2C_SR2 */
#define I2C_SR2_MSL				(0U)
#define I2C_SR2_BUSY			(1U)
#define I2C_SR2_TRA				(2U)
#define I2C_SR2_GEN_CALL		(4U)
#define I2C_SR2_SMBDE_FAULT		(5U)
#define I2C_SR2_SMB_HOST		(6U)
#define I2C_SR2_DUALF			(7U)
#define I2C_SR2_PEC				(8U)

/* I2C_OAR1 */
#define I2C_OAR1_ADD0			(0U)
#define I2C_OAR1_ADD1			(1U)
#define I2C_OAR1_ADDMODE		(15U)

/* I2C_CCR */
#define I2C_CCR_CCR				(0U)
#define I2C_CCR_DUTY			(14U)
#define I2C_CCR_FS				(15U)

/* RCC_CFGR */
#define RCC_CFGR_SW				(0U)
#define RCC_CFGR_SWS			(2U)
#define RCC_CFGR_HRPE			(4U)
#define RCC_CFGR_PRE1			(8U)
#define RCC_CFGR_PRE2			(11U)
#define RCC_CFGR_ADCPRE			(14U)
#define RCC_CFGR_PLL_SRC		(16U)
#define RCC_CFGR_PLL_XTPRE		(17U)
#define RCC_CFGR_PLL_MUL		(18U)
#define RCC_CFGR_USB_PRE		(22U)
#define RCC_CFGR_MCO			(24U)

/* USART_SR */
#define USART_SR_PE				(0U)
#define USART_SR_FE				(1U)
#define USART_SR_NE				(2U)
#define USART_SR_ORE			(3U)
#define USART_SR_IDLE			(4U)
#define USART_SR_RXNE			(5U)
#define USART_SR_TC				(6U)
#define USART_SR_TXE			(7U)
#define USART_SR_LBD			(8U)
#define USART_SR_CTS			(9U)

/* USART_CR1 */
#define USART_CR1_SBK			(0U)
#define USART_CR1_RWU			(1U)
#define USART_CR1_RE			(2U)
#define USART_CR1_TE			(3U)
#define USART_CR1_IDLEIE		(4U)
#define USART_CR1_RXNEIE		(5U)
#define USART_CR1_TCIE			(6U)
#define USART_CR1_TXEIE			(7U)
#define USART_CR1_PEIE			(8U)
#define USART_CR1_PS			(9U)
#define USART_CR1_PCE			(10U)
#define USART_CR1_WAKE			(11U)
#define USART_CR1_M				(12U)
#define USART_CR1_UE			(13U)

/* USART_CR2 */
#define USART_CR2_ADD			(0U)
#define USART_CR2_LBDL			(5U)
#define USART_CR2_LBDIE			(6U)
#define USART_CR2_LBLC			(8U)
#define USART_CR2_CPHA			(9U)
#define USART_CR2_CPOL			(10U)
#define USART_CR2_CLKEN			(11U)
#define USART_CR2_STOP			(12U)
#define USART_CR2_LINEN			(14U)

/* USART_CR1 */
#define USART_CR3_EIE			(0U)
#define USART_CR3_IREN			(1U)
#define USART_CR3_IRLP			(2U)
#define USART_CR3_HDSEL			(3U)
#define USART_CR3_NACK			(4U)
#define USART_CR3_SCEN			(5U)
#define USART_CR3_DMAR			(6U)
#define USART_CR3_DMAT			(7U)
#define USART_CR3_RTSE			(8U)
#define USART_CR3_CTSE			(9U)
#define USART_CR3_CTSIE			(10U)

/**************************
 *  Registers Structures
 **************************/

/* Reset and Clock Cotrol (RCC) */
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
} RCC_RegDef_t;


 /* General Purpose Input/Output (GPIO) */
typedef struct
{
	volatile uint32_t CR[2]; 	/* GPIO_CRL (0) or GPIO_CRH (1) */
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
} GPIO_RegDef_t;

/* Alternative Function Input/Output (AFIO) */
typedef struct
{
	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR[4]; /* EXTICR1 to EXTICR4*/
	volatile uint32_t RESERVED;
	volatile uint32_t MAPR2;
} AFIO_RegDef_t;

/* External Interrupt (EXTI) */
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegDef_t;

/* Serial Peripheral Interface (SPI) */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_RegDef_t;

/* Inter-Intergrated Circuit Interface (I2C)*/
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
} I2C_RegDef_t;

/* Universal Synchronous Asynchronous Receiver Transmitter (USART) */
typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_RegDef_t;

/**************************
 *  Registers Defs
 **************************/
#define RCC						((RCC_RegDef_t *) RCC_BASE_ADDRESS)

#define GPIOA					((GPIO_RegDef_t *) GPIOA_BASE_ADDRESS)
#define GPIOB					((GPIO_RegDef_t *) GPIOB_BASE_ADDRESS)
#define GPIOC					((GPIO_RegDef_t *) GPIOC_BASE_ADDRESS)
#define GPIOD					((GPIO_RegDef_t *) GPIOD_BASE_ADDRESS)
#define GPIOE					((GPIO_RegDef_t *) GPIOE_BASE_ADDRESS)
#define GPIOF					((GPIO_RegDef_t *) GPIOF_BASE_ADDRESS)
#define GPIOG					((GPIO_RegDef_t *) GPIOG_BASE_ADDRESS)

#define EXTI					((EXTI_RegDef_t *) EXTI_BASE_ADDRESS)
#define AFIO					((AFIO_RegDef_t *) AFIO_BASE_ADDRESS)

#define SPI1					((SPI_RegDef_t *) SPI1_BASE_ADDRESS)
#define SPI2					((SPI_RegDef_t *) SPI2_BASE_ADDRESS)
#define SPI3					((SPI_RegDef_t *) SPI3_BASE_ADDRESS)

#define I2C1					((I2C_RegDef_t *) I2C1_BASE_ADDRESS)
#define I2C2					((I2C_RegDef_t *) I2C2_BASE_ADDRESS)

#define USART1					((USART_RegDef_t *) USART1_BASE_ADDRESS)
#define USART2					((USART_RegDef_t *) USART2_BASE_ADDRESS)
#define USART3					((USART_RegDef_t *) USART3_BASE_ADDRESS)
#define UART4					((USART_RegDef_t *) UART4_BASE_ADDRESS)
#define UART5					((USART_RegDef_t *) UART5_BASE_ADDRESS)

#endif /* INC_STM32F103XX_H_ */
