/*
 * stm32f407xx.h
 *
 *  Created on: Jun 2, 2024
 *      Author: Karan Patel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stddef.h>
#include<stdint.h>



#define __vo volatile
#define __weak __attribute__((weak))

// ARM cortex Mx processor NVIC ISERx register addresses.

#define NVIC_ISER0       ((__vo uint32_t*) 0XE000E100)
#define NVIC_ISER1       ((__vo uint32_t*) 0XE000E104)
#define NVIC_ISER2       ((__vo uint32_t*) 0XE000E108)
#define NVIC_ISER3       ((__vo uint32_t*) 0XE000E10C)

// ARM cortex Mx processor NVIC ICERx register addresses.

#define NVIC_ICER0       ((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1       ((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2       ((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3       ((__vo uint32_t*) 0XE000E18C)


// ARM cortex Mx processor priority register address calculation.
#define  NVIC_PR_BASE_ADDR ((__vo uint32_t*)0XE000E400)

#define NO_PR_BITS_IMPLEMENTED  4
// Base address of flash and SRAM memories.

#define FLASH_BASEADDR           0x08000000U  // base address for flash memory
#define SRAM1_BASEADDR           0x20000000U  // base address for SRAM1
#define SRAM2_BASEADDR           0X2001C000U  // base address for SRAM2
#define ROM_BASEADDR             0X1FFF0000U  // base address for ROM
#define SRAM                     SRAM1_BASEADDR



// Base address of ahbx and apbx bus peripheral devices.


#define PERIPH_BASE           0X40000000U
#define APB1PERIPH_BASEADDR       PERIPH_BASE
#define APB2PERIPH_BASEADDR       0X40010000U
#define AHB1PERIPH_BASEADDR       0X40020000U
#define AHB2PERIPH_BASEADDR       0X50000000U



//Base address of peripherals which are hanging on AHB1 bus.



#define GPIOA_BASEADDR             (AHB1PERIPH_BASEADDR+0X0000)
#define GPIOB_BASEADDR             (AHB1PERIPH_BASEADDR+0X0400)
#define GPIOC_BASEADDR             (AHB1PERIPH_BASEADDR+0X0800)
#define GPIOD_BASEADDR             (AHB1PERIPH_BASEADDR+0X0C00)
#define GPIOE_BASEADDR             (AHB1PERIPH_BASEADDR+0X1000)
#define GPIOF_BASEADDR             (AHB1PERIPH_BASEADDR+0X1400)
#define GPIOG_BASEADDR             (AHB1PERIPH_BASEADDR+0X1800)
#define GPIOH_BASEADDR             (AHB1PERIPH_BASEADDR+0X1C00)
#define GPIOI_BASEADDR             (AHB1PERIPH_BASEADDR+0X2000)
#define GPIOJ_BASEADDR             (AHB1PERIPH_BASEADDR+0X2400)
#define GPIOK_BASEADDR             (AHB1PERIPH_BASEADDR+0X2800)
#define RCC_BASEADDR               (AHB1PERIPH_BASEADDR+0X3800)

//Base address of peripherals which are hanging on AHB1 bus.


#define I2C1_BASEADDR                (APB1PERIPH_BASEADDR+0X5400)
#define I2C2_BASEADDR                (APB1PERIPH_BASEADDR+0X5800)
#define I2C3_BASEADDR                (APB1PERIPH_BASEADDR+0X5C00)
#define SPI2_BASEADDR                (APB1PERIPH_BASEADDR+0X3800)
#define SPI3_BASEADDR                (APB1PERIPH_BASEADDR+0X3C00)
#define USART2_BASEADDR              (APB1PERIPH_BASEADDR+0X4400)
#define USART3_BASEADDR              (APB1PERIPH_BASEADDR+0X4800)
#define UART4_BASEADDR               (APB1PERIPH_BASEADDR+0X4C00)
#define UART5_BASEADDR               (APB1PERIPH_BASEADDR+0X5000)


//Base address of peripherals which are hanging on APB2 bus.


#define EXTI_BASEADDR                  (APB2PERIPH_BASEADDR+0X3C00)
#define SPI1_BASEADDR                  (APB2PERIPH_BASEADDR+0X3000)
#define USART1_BASEADDR                (APB2PERIPH_BASEADDR+0X1000)
#define USART6_BASEADDR                (APB2PERIPH_BASEADDR+0X1400)
#define SYSCFG_BASEADDR                (APB2PERIPH_BASEADDR+0X3800)



#define  ADC1_BASEADDR                 (APB2PERIPH_BASEADDR+0X2000)
#define  ADC2_BASEADDR                 (APB2PERIPH_BASEADDR+0x2100)
#define  ADC3_BASEADDR                 (APB2PERIPH_BASEADDR+0x2200)


#define ADC1                           ((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2                           ((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3                           ((ADC_RegDef_t*)ADC3_BASEADDR)

// Peripherals register defination structures

typedef struct{

	__vo uint32_t MODER;  //GPIO port mode register.
	__vo uint32_t OTYPER; //GPIO port output type register
	__vo uint32_t OSPEEDR; //GPIO port output speed register
	__vo uint32_t PUPDR;  //GPIO port pull-up/pull-down register
	__vo uint32_t IDR;    //GPIO port input data register
	__vo uint32_t ODR;    //GPIO port output data register
	__vo uint32_t BSRR;   //GPIO port bit set/reset register
	__vo uint32_t LCKR;   //GPIO port configuration lock register
	__vo uint32_t AFR[2]; // AFR[0]:GPIO alternate function low register,AFR[1]:GPIO alternate function high register

}GPIO_RegDef_t;


typedef struct{

	__vo uint32_t SR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMPR1;
	__vo uint32_t SMPR2;
	__vo uint32_t JOFR1;
	__vo uint32_t JOFR2;
	__vo uint32_t JOFR3;
	__vo uint32_t JOFR4;
	__vo uint32_t HTR;
	__vo uint32_t LTR;
	__vo uint32_t SQR1;
	__vo uint32_t SQR2;
	__vo uint32_t SQR3;
	__vo uint32_t JSQR;
	__vo uint32_t JDR1;
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t DR;

}ADC_RegDef_t;

typedef struct
{
  __vo uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  __vo uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  __vo uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


typedef struct{

	__vo uint32_t  CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t      RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t      RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t      RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKFGR2;

}RCC_RegDef_t;



typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;



typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	 uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	 uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR ;
}I2C_RegDef_t;


typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


#define GPIOA      ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB      ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF      ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG      ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH      ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI      ((GPIO_RegDef_t*)GPIOI_BASEADDR)



#define TIM1_BASEADDR        (APB2PERIPH_BASEADDR+0X0000)
#define TIM2_BASEADDR        (APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR        (APB1PERIPH_BASEADDR+0X0400)
#define TIM4_BASEADDR        (APB1PERIPH_BASEADDR+0X0800)
#define TIM5_BASEADDR        (APB1PERIPH_BASEADDR+0X0C00)
#define TIM6_BASEADDR        (APB1PERIPH_BASEADDR+0X1000)
#define TIM7_BASEADDR        (APB1PERIPH_BASEADDR+0X1400)
#define TIM8_BASEADDR        (APB2PERIPH_BASEADDR+0X0400)
#define TIM9_BASEADDR        (APB2PERIPH_BASEADDR+0X4000)
#define TIM10_BASEADDR       (APB2PERIPH_BASEADDR+0X4400)
#define TIM11_BASEADDR       (APB2PERIPH_BASEADDR+0X4800)
#define TIM12_BASEADDR       (APB1PERIPH_BASEADDR+0X1800)
#define TIM13_BASEADDR       (APB1PERIPH_BASEADDR+0X1C00)
#define TIM14_BASEADDR       (APB1PERIPH_BASEADDR+0X2000)

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1[2];
	__vo uint32_t CCMR2[2];
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t DCR;
	__vo uint32_t DMAR;
	__vo uint32_t OR;

}TIMER_RegDef_t;





#define TIM1    ((TIMER_RegDef_t*)TIM1_BASEADDR)
#define TIM2    ((TIMER_RegDef_t*)TIM2_BASEADDR)
#define TIM3    ((TIMER_RegDef_t*)TIM3_BASEADDR)
#define TIM4    ((TIMER_RegDef_t*)TIM4_BASEADDR)
#define TIM5    ((TIMER_RegDef_t*)TIM5_BASEADDR)
#define TIM6    ((TIMER_RegDef_t*)TIM6_BASEADDR)
#define TIM7    ((TIMER_RegDef_t*)TIM7_BASEADDR)
#define TIM8    ((TIMER_RegDef_t*)TIM8_BASEADDR)
#define TIM9    ((TIMER_RegDef_t*)TIM9_BASEADDR)
#define TIM10   ((TIMER_RegDef_t*)TIM10_BASEADDR)
#define TIM11   ((TIMER_RegDef_t*)TIM11_BASEADDR)
#define TIM12   ((TIMER_RegDef_t*)TIM12_BASEADDR)
#define TIM13   ((TIMER_RegDef_t*)TIM13_BASEADDR)
#define TIM14   ((TIMER_RegDef_t*)TIM14_BASEADDR)



#define ADC_CCR_ADCPRE



#define RCC        ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI       ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG     ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*)SPI3_BASEADDR)



#define I2C1            ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2            ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3            ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1          ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2          ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3          ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4           ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5           ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6          ((USART_RegDef_t*)USART6_BASEADDR)
//Clock enable macros for GPIOX peripherals.

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0));
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1));
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2));
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3));
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4));
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |=(1<<5));
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |=(1<<6));
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |=(1<<7));
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |=(1<<8));


// Clock enable macros for I2Cx peripherals.

#define I2C1_PCLK_EN()  (RCC->APB1ENR |=(1<<21));
#define I2C2_PCLK_EN()  (RCC->APB1ENR |=(1<<22));
#define I2C3_PCLK_EN()  (RCC->APB1ENR |=(1<<23));
//Clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN()  (RCC->APB2ENR |=(1<<12));
#define SPI2_PCLK_EN()  (RCC->APB1ENR |=(1<<14));
#define SPI3_PCLK_EN()  (RCC->APB1ENR |=(1<<15));




#define ADC1_PCLK_EN()  (RCC->APB2ENR |=(1<<8));
#define ADC2_PCLK_EN()  (RCC->APB1ENR |=(1<<9));
#define ADC3_PCLK_EN()  (RCC->APB1ENR |=(1<<10));


#define ADC1_PCLK_DI()  (RCC->APB2ENR &=~(1<<8));
#define ADC2_PCLK_DI()  (RCC->APB1ENR &=~(1<<9));
#define ADC3_PCLK_DI()  (RCC->APB1ENR &=~(1<<10));


//Clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()  (RCC->APB2ENR |=(1<<4));
#define USART2_PCLK_EN()  (RCC->APB1ENR |=(1<<17));
#define USART3_PCLK_EN()  (RCC->APB1ENR |=(1<<18));
#define USART6_PCLK_EN()  (RCC->APB2ENR |=(1<<5));
#define UART4_PCLK_EN()   (RCC->APB1ENR |=(1<<19));
#define UART5_PCLK_EN()   (RCC->APB1ENR |=(1<<20));
// Clock enable macros for SYSCFG peripherals.
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |=(1<<14));

//Clock disable macros for GPIOx peripherals.
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(1<<0));
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~(1<<1));
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(1<<2));
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(1<<3));
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(1<<4));
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &=~(1<<5));
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &=~(1<<6));
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &=~(1<<7));
#define GPIOI_PCLK_DI()    (RCC->AHB1ENR &=~(1<<8));


// Clock disable macros for I2Cx peripherals.

#define I2C1_PCLK_DI()  (RCC->APB1ENR &=~(1<<21));
#define I2C2_PCLK_DI()  (RCC->APB1ENR &=~(1<<22));
#define I2C3_PCLK_DI()  (RCC->APB1ENR &=~(1<<23));
//Clock enable macros for SPIx peripherals

#define SPI1_PCLK_DI()  (RCC->APB2ENR &=~(1<<12));
#define SPI2_PCLK_DI()  (RCC->APB1ENR &=~(1<<14));
#define SPI3_PCLK_DI()  (RCC->APB1ENR &=~(1<<15));

//Clock enable macros for USARTx peripherals
#define USART1_PCLK_DI()  (RCC->APB2ENR &=~(1<<4));
#define USART2_PCLK_DI()  (RCC->APB1ENR &=~(1<<17));
#define USART3_PCLK_DI()  (RCC->APB1ENR &=~(1<<18));
#define USART6_PCLK_DI()  (RCC->APB2ENR &=~(1<<5));
#define UART4_PCLK_DI()   (RCC->APB1ENR &=~(1<<19));
#define UART5_PCLK_DI()   (RCC->APB1ENR &=~(1<<20));

// Clock enable macros for SYSCFG peripherals.
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &=~(1<<14));

// Macros to reset GPIOX peripherals.
#define GPIOA_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<0)); (RCC->AHB1RSTR&=~(1<<0)); } while(0)
#define GPIOB_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<1)); (RCC->AHB1RSTR&=~(1<<1)); } while(0)
#define GPIOC_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<2)); (RCC->AHB1RSTR&=~(1<<2)); } while(0)
#define GPIOD_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<3)); (RCC->AHB1RSTR&=~(1<<3)); } while(0)
#define GPIOE_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<4)); (RCC->AHB1RSTR&=~(1<<4)); } while(0)
#define GPIOF_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<5)); (RCC->AHB1RSTR&=~(1<<5)); } while(0)
#define GPIOG_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<6)); (RCC->AHB1RSTR&=~(1<<6)); } while(0)
#define GPIOH_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<7)); (RCC->AHB1RSTR&=~(1<<7)); } while(0)
#define GPIOI_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<8)); (RCC->AHB1RSTR&=~(1<<8)); } while(0)


// Resetting the registers.
#define SPI1_REG_RESET()   do {(RCC->APB2RSTR|=(1<<12)); (RCC->APB2RSTR&=~(1<<12)); } while(0)
#define SPI2_REG_RESET()   do {(RCC->APB2RSTR|=(1<<14)); (RCC->APB2RSTR&=~(1<<14)); } while(0)
#define SPI3_REG_RESET()   do {(RCC->APB2RSTR|=(1<<15)); (RCC->APB2RSTR&=~(1<<15)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
		(x == GPIOB)?1:\
		(x == GPIOC)?2:\
		(x == GPIOD)?3:\
        (x == GPIOE)?4:\
        (x == GPIOF)?5:\
        (x == GPIOG)?6:\
        (x == GPIOH)?7: \
        (x == GPIOI)?8:0)



#define ENABLE   1
#define DISABLE  0
#define SET      ENABLE
#define RESET    DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET RESET

//Bit position definationss of CR1 peripheral.

#define SPI_CR1_CPHA         0
#define SPI_CR1_CPOL         1
#define SPI_CR1_MSTR         2
#define SPI_CR1_BR           3
#define SPI_CR1_SPE          6
#define SPI_CR1_LSBFIRST     7
#define SPI_CR1_SSI          8
#define SPI_CR1_SSM          9
#define SPI_CR1_RXONLY       10
#define SPI_CR1_DFF          11
#define SPI_CR1_CRCNEXT      12
#define SPI_CR1_CRCEN        13
#define SPI_CR1_BIDIOE       14
#define SPI_CR1_BIDIMODE     15

//Bit position definationss of CR2 peripheral.
#define SPI_CR2_RXDMAEN      0
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_SSOE         2
#define SPI_CR2_FRF          4
#define SPI_CR2_ERRIE        5
#define SPI_CR2_RXNEIE       6
#define SPI_CR2_TXEIE        7


// Bit position defination of SPI_SR

#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8


//bit position defination of i2c_cr1

#define I2C_CR1_PE               0
#define I2C_CR1_NOSTRETCH        7
#define I2C_CR1_START            8
#define I2C_CR1_STOP             9
#define I2C_CR1_ACK              10
#define I2C_CR1_SWRST            15


//bit position defination of i2c_cr2

#define I2C_CR2_FREQ               0
#define I2C_CR2_ITERREN            8
#define I2C_CR2_ITEVTEN            9
#define I2C_CR2_ITBUFEN            10

//bit position defination of i2c_oar1

#define I2C_OAR_ADD0               0
#define I2C_OAR_ADD71              1
#define I2C_OAR_ADD98              8
#define I2C_OAR_ADDMODE            15


//bit position defination of i2c_SR1

#define I2C_SR1_SB                    0
#define I2C_SR1_ADDR                  1
#define I2C_SR1_BTF                   2
#define I2C_SR1_ADD10                 3
#define I2C_SR1_STOPF                 4
#define I2C_SR1_RXNE                  6
#define I2C_SR1_TXE                   7
#define I2C_SR1_BERR                  8
#define I2C_SR1_ARLO                  9
#define I2C_SR1_AF                    10
#define I2C_SR1_OVR                   11
#define I2C_SR1_TIMEOUT               14


//bit position defination of i2c_SR2

#define I2C_SR2_MSL                   0
#define I2C_SR2_BUSY                  1
#define I2C_SR2_TRA                   2
#define I2C_SR2_GENCALL               4
#define I2C_SR2_DUALF                 7

//bit position defination of i2c_CCR
#define I2C_CCR_CCR                   0
#define I2C_CCR_DUTY                  14
#define I2C_CCR_FS                    15
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

//IRQ(interrupt request) number of stm32f407x MCU.

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

//macros for all the possible priority levels.

#define NVIC_IRQ_PRI0   0
#define NVIC_IRQ_PRI1   1
#define NVIC_IRQ_PRI2   2
#define NVIC_IRQ_PRI3   3
#define NVIC_IRQ_PRI4   4
#define NVIC_IRQ_PRI5   5
#define NVIC_IRQ_PRI6   6
#define NVIC_IRQ_PRI7   7
#define NVIC_IRQ_PRI8   8
#define NVIC_IRQ_PRI9   9
#define NVIC_IRQ_PRI10  10
#define NVIC_IRQ_PRI11  11
#define NVIC_IRQ_PRI12  12
#define NVIC_IRQ_PRI13  13
#define NVIC_IRQ_PRI14  14
#define NVIC_IRQ_PRI15  15



// ADC related

#define ADC_CR2_ALIGN   11
#define ADC_CR2_CONT    2

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include"stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_ADC_driver.h"
#endif /* INC_STM32F407XX_H_ */


