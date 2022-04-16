/*
 * STM32f407xx.h
 *
 *  Created on: Mar 13, 2022
 *      Author: atarek
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

#define NO_BITS_IN_BYTE			8

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						(0x08000000U)
#define SRAM1_BASEADDR						(0x20000000U)
#define SRAM2_BASEADDR						(0x2001C000U)
#define ROM_BASEADDR						(0x1FFF0000U)
#define SRAM 								(SRAM1_BASEADDR)

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR						(0x40000000U)
#define APB1PERIPH_BASEADDR					(PERIPH_BASEADDR)
#define APB2PERIPH_BASEADDR					(0x40010000U)
#define AHB1PERIPH_BASEADDR					(0x40020000U)
#define AHB2PERIPH_BASEADDR					(0x50000000U)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define ADC_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)
#define ADC1_BASEADDR						(ADC_BASEADDR + 0x0000)
#define ADC2_BASEADDR						(ADC_BASEADDR + 0x0100)
#define ADC3_BASEADDR						(ADC_BASEADDR + 0x0200)
#define ADC_COM_REG_BASEADDR				(ADC_BASEADDR + 0x0300)

/**********************************peripheral register definition structures **********************************/

/*
 * peripheral register definition structure for GPIOs
 */

typedef struct
{
	__vo uint32_t MODER;						/*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;						/*!< GPIO port output type register,     			Address offset: 0x04      */
	__vo uint32_t OSPEEDR;						/*!< GPIO port output speed register,     			Address offset: 0x08      */
	__vo uint32_t PUPDR;						/*!< GPIO port pull-up/pull-down register,   		Address offset: 0x0C      */
	__vo uint32_t IDR;							/*!< GPIO port input data register,     			Address offset: 0x10      */
	__vo uint32_t ODR;							/*!< GPIO port output data register,     			Address offset: 0x14      */
	__vo uint32_t BSRR;							/*!< GPIO port bit set/reset register,     			Address offset: 0x18      */
	__vo uint32_t LCKR;							/*!< GPIO port configuration lock register,    		Address offset: 0x1C      */
	__vo uint32_t AFR[2];						/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */



/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t  SPI_CR1;		/* SPI control register 1							Address offset: 0x00 */
	__vo uint32_t  SPI_CR2;		/* SPI control register 2							Address offset: 0x04 */
	__vo uint32_t  SPI_SR;		/* SPI status register								Address offset: 0x08 */
	__vo uint32_t  SPI_DR;		/* SPI data register								Address offset: 0x0C */
	__vo uint32_t  SPI_CRCPR;	/* SPI CRC polynomial register						Address offset: 0x10 */
	__vo uint32_t  SPI_RXCRCR;	/* SPI RX CRC register								Address offset: 0x14 */
	__vo uint32_t  SPI_TXCRCR;	/* SPI TX CRC register								Address offset: 0x18 */
	__vo uint32_t  SPI_I2SCFGR;	/* SPI_I2S configuration register					Address offset: 0x1C */
	__vo uint32_t  SPI_I2SPR;	/* SPI_I2S prescaler register						Address offset: 0x20 */

} SPI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */



/*
 * peripheral register definition structure for I2C
 */


/*
 * peripheral register definition structure for USART
 */

/*
 * peripheral register definition structure for ADC
 */

typedef struct
{
	__vo uint32_t  ADC_SR;		/* ADC status register								Address offset: 0x00 */
	__vo uint32_t  ADC_CR1;		/* ADC control register 1							Address offset: 0x04 */
	__vo uint32_t  ADC_CR2;		/* ADC control register 2							Address offset: 0x08 */
	__vo uint32_t  ADC_SMPR1;	/* ADC sample time register 1						Address offset: 0x0C */
	__vo uint32_t  ADC_SMPR2;	/* ADC sample time register 2						Address offset: 0x10 */
	__vo uint32_t  ADC_JOFR1;	/* ADC injected channel data offset register 1		Address offset: 0x14 */
	__vo uint32_t  ADC_JOFR2;	/* ADC injected channel data offset register 2		Address offset: 0x18 */
	__vo uint32_t  ADC_JOFR3;	/* ADC injected channel data offset register 3		Address offset: 0x1C */
	__vo uint32_t  ADC_JOFR4;	/* ADC injected channel data offset register 4		Address offset: 0x20 */
	__vo uint32_t  ADC_HTR;		/* ADC watchdog higher threshold register			Address offset: 0x24 */
	__vo uint32_t  ADC_LTR;		/* ADC watchdog lower threshold register			Address offset: 0x28 */
	__vo uint32_t  ADC_SQR1;	/* ADC regular sequence register 1					Address offset: 0x2C */
	__vo uint32_t  ADC_SQR2;	/* ADC regular sequence register 2					Address offset: 0x30 */
	__vo uint32_t  ADC_SQR3;	/* ADC regular sequence register 3					Address offset: 0x34 */
	__vo uint32_t  ADC_JSQR;	/* ADC injected sequence register					Address offset: 0x38 */
	__vo uint32_t  ADC_JDR1;	/* ADC injected data register 1						Address offset: 0x3C */
	__vo uint32_t  ADC_JDR2;	/* ADC injected data register 2						Address offset: 0x40 */
	__vo uint32_t  ADC_JDR3;	/* ADC injected data register 3						Address offset: 0x44 */
	__vo uint32_t  ADC_JDR4;	/* ADC injected data register 4						Address offset: 0x48 */
	__vo uint32_t  ADC_DR;		/* ADC regular data register						Address offset: 0x4C */
} ADC_RegDef_t;


typedef struct
{
	__vo uint32_t  ADC_CSR;		/* ADC Common status register						Address offset: 0x0 */
	__vo uint32_t  ADC_CCR;		/* ADC common control register						Address offset: 0x04 */
	__vo uint32_t  ADC_CDR;		/* ADC common regular data register 				Address offset: 0x08 */

} ADC_Comm_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define ADC1  				((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2  				((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3  				((ADC_RegDef_t*)ADC3_BASEADDR)
#define ADC_COMM  			((ADC_Comm_RegDef_t*)ADC_COM_REG_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for ADCx peripherals
 */
#define ADC1_PCLK_EN() (RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN() (RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN() (RCC->APB2ENR |= (1 << 10))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)




//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET			RESET
#define FLAG_SET 			SET



/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0				/* Clock phase */
#define SPI_CR1_CPOL      				 1				/* Clock polarity */
#define SPI_CR1_MSTR     				 2				/* Master selection */
#define SPI_CR1_BR   					 3				/* Baud rate control */
#define SPI_CR1_SPE     				 6				/* SPI enable */
#define SPI_CR1_LSBFIRST   			 	 7				/* Frame format */
#define SPI_CR1_SSI     				 8				/* Internal slave select */
#define SPI_CR1_SSM      				 9				/* Software slave management */
#define SPI_CR1_RXONLY      		 	10				/* Receive only */
#define SPI_CR1_DFF     			 	11				/* Data frame format */
#define SPI_CR1_CRCNEXT   			 	12				/* CRC transfer next */
#define SPI_CR1_CRCEN   			 	13				/* Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE     			 	14				/* Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE      			15				/* Bidirectional data mode enable */

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0				/* Rx buffer DMA enable */
#define SPI_CR2_TXDMAEN				 	1				/* Tx buffer DMA enable */
#define SPI_CR2_SSOE				 	2				/* SS output enable */
#define SPI_CR2_FRF						4				/* Frame format */
#define SPI_CR2_ERRIE					5				/* Error interrupt enable */
#define SPI_CR2_RXNEIE				 	6				/* RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE					7				/* Tx buffer empty interrupt enable
 */


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0				/* Receive buffer not empty */
#define SPI_SR_TXE				 		1				/* Transmit buffer empty */
#define SPI_SR_CHSIDE				 	2				/* Channel side */
#define SPI_SR_UDR					 	3				/* Underrun flag */
#define SPI_SR_CRCERR				 	4				/* CRC error flag */
#define SPI_SR_MODF					 	5				/* Mode fault */
#define SPI_SR_OVR					 	6				/* Overrun flag */
#define SPI_SR_BSY					 	7				/* Busy flag */
#define SPI_SR_FRE					 	8				/* Frame format error */

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */


/*
 * Bit position definitions I2C_CR2
 */


/*
 * Bit position definitions I2C_OAR1
 */


/*
 * Bit position definitions I2C_SR1
 */


/*
 * Bit position definitions I2C_SR2
 */


/*
 * Bit position definitions I2C_CCR
 */


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */




/*
 * Bit position definitions USART_CR2
 */



/*
 * Bit position definitions USART_CR3
 */


/*
 * Bit position definitions USART_SR
 */

/******************************************************************************************
 *Bit position definitions of ADC peripheral
 ******************************************************************************************/

/*
 * Bit position definitions ADC_SR
 */
#define ADC_SR_AWD						0				/*  */
#define ADC_SR_EOC						1				/*  */
#define ADC_SR_JEOC						2				/*  */
#define ADC_SR_JSTRT					3				/*  */
#define ADC_SR_STRT						4				/*  */
#define ADC_SR_OVR						5				/*  */

/*
 * Bit position definitions ADC_CR1
 */
#define ADC_CR1_SCAN     				 8				/* Scan Mode Enable / Disable */

/*
 * Bit position definitions ADC_CR2
 */
#define ADC_CR2_ADON     				 0				/* A/D Converter ON / OFF */
#define ADC_CR2_CONT     				 1				/* A/D Converter Mode Single / Continuous */
#define ADC_CR2_SWSTART     			 30				/* Start conversion of regular channels */

/*
 * Bit position definitions ADC_SMPR1
 */

/*
* Bit position definitions ADC_SQR1
*/
#define ADC_SQR_COV_13     				 0				/* 13th conversion in regular sequence */
#define ADC_SQR_COV_14     				 5				/* 14th conversion in regular sequence */
#define ADC_SQR_COV_15     				 10				/* 15th conversion in regular sequence */
#define ADC_SQR_COV_16     				 15				/* 16th conversion in regular sequence */
#define ADC_SQR_SEQ_LEN    				 20				/* Regular channel sequence length */

/*
* Bit position definitions ADC_SQR2
*/
#define ADC_SQR_COV_7     				 0				/* 7th conversion in regular sequence */
#define ADC_SQR_COV_8     				 5				/* 8th conversion in regular sequence */
#define ADC_SQR_COV_9     				 10				/* 9th conversion in regular sequence */
#define ADC_SQR_COV_10     				 15				/* 10th conversion in regular sequence */
#define ADC_SQR_COV_11    				 20				/* 11th conversion in regular sequence */
#define ADC_SQR_COV_12    				 25				/* 12th conversion in regular sequence */

/*
* Bit position definitions ADC_SQR3
*/
#define ADC_SQR_COV_1     				 0				/* 1th conversion in regular sequence */
#define ADC_SQR_COV_2     				 5				/* 2th conversion in regular sequence */
#define ADC_SQR_COV_3     				 10				/* 3th conversion in regular sequence */
#define ADC_SQR_COV_4     				 15				/* 4th conversion in regular sequence */
#define ADC_SQR_COV_5    				 20				/* 5th conversion in regular sequence */
#define ADC_SQR_COV_6    				 25				/* 6th conversion in regular sequence */


#endif /* STM32F407XX_H_ */
