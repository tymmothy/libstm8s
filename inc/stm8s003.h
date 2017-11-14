/******************************************************************************
 * @file:    stm8s003.h
 * @purpose: Header File for STM8S003/STM8S103 Microcontrollers.
 * @version: V1.0
 * @author:  Tymm Zerr
 * @date:    1. November 2014
 *
 * Generally this should be included indirectly via stm8s.h
 * (this will be included when STM8S003 or STM8S103 is defined)
 *
 ******************************************************************************
 * @section License License
 * Licensed under a Simplified BSD License:
 *
 * Copyright (c) 2014-2017, Tymm Zerr
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * TYMM ZERR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Tymm Zerr.
 *****************************************************************************/

#ifndef STM8S003_H_
#define STM8S003_H_

#include <stdint.h>

#include "stm8s.h"


/** @addtogroup IRQn
  * @{
  */

/***** STM8S003/103 Exceptions & Interrupt Numbers ***************************/

typedef enum {
    RESET_IRQn = -2,
    TRAP_IRQn = -1,
    TLI_IRQn = 0,
    AWU_IRQn,
    CLK_IRQn,
    EXTI0_IRQn,
    EXT1_IRQn,
    EXT2_IRQn,
    EXT3_IRQn,
    EXT4_IRQn,
    SPI_IRQn = 10,
    TIM1_UPD_OVF_UF_TRIG_BRK_IRQn,
    TIM1_CC_IRQn,
    TIM2_UPD_OVF_UF_TRIG_BRK_IRQn,
    TIM2_CC_IRQn,
    UART1_TX_IRQn = 17,
    UART1_RX_IRQn,
    I2C_IRQn,
    ADC1_IRQn = 22,
    TIM4_IRQn,
    FLASH_IRQn,
} IRQn_Type;

/**
  * @}
  */

/** @addtogroup Peripheral_Base_Addresses
  * @{
  */

/***** STM8S003/103 Peripheral Base Addresses ********************************/

#define GPIOA_BASE      (0x5000)
#define GPIOB_BASE      (0x5005)
#define GPIOC_BASE      (0x500a)
#define GPIOD_BASE      (0x500f)
#define GPIOE_BASE      (0x5014)
#define GPIOF_BASE      (0x5019)
#define FLASH_BASE      (0x505a)
#define EXTI_BASE       (0x50a0)
#define RST_BASE        (0x50b3)
#define CLK_BASE        (0x50c0)
#define WWDG_BASE       (0x50d1)
#define IWDG_BASE       (0x50e0)
#define AWU_BASE        (0x50f0)
#define BEEP_BASE       (0x50f3)
#define SPI1_BASE       (0x5200)
#define I2C1_BASE       (0x5210)
#define UART1_BASE      (0x5230)
#define TIM1_BASE       (0x5250)
#define TIM2_BASE       (0x5300)
#define TIM4_BASE       (0x5340)
#define ADC1_BASE       (0x53e0)
#define CPUREG_BASE     (0x7f00)
#define GCR_BASE        (0x7f60)
#define ITC_BASE        (0x7f70)
#define SWIM_BASE       (0x7f80)
#define DM_BASE         (0x7f90)

/**
  * @}
  */

/** @addtogroup Peripherals
  * @{
  */

/***** STM8S003/103 Peripherals **********************************************/

#define GPIOA           ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB           ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC           ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD           ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE           ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF           ((GPIO_TypeDef *)GPIOF_BASE)
#define FLASH           ((FLASH_TypeDef *)FLASH_BASE)
#define EXTI            ((EXTI_TypeDef *)EXTI_BASE)
#define RST             ((RST_TypeDef *)RST_BASE)
#define CLK             ((CLK_TypeDef *)CLK_BASE)
#define WWDG            ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG            ((IWDG_TypeDef *)IWDG_BASE)
#define AWU             ((AWU_TypeDef *)AWU_BASE)
#define BEEP            ((BEEP_TypeDef *)BEEP_BASE)
#define SPI1            ((SPI_TypeDef *)SPI_BASE)
#define I2C1            ((I2C_TypeDef *)I2C_BASE)
#define UART1           ((UART_TypeDef *)UART_BASE)
#define TIM1            ((TIM1_TypeDef *)TIM1_BASE)
#define TIM2            ((TIM2_TypeDef *)TIM2_BASE)
#define TIM4            ((TIM4_TypeDef *)TIM4_BASE)
#define ADC1            ((ADC1_TypeDef *)ADC1_BASE)
#define CPUREG          ((CPUREG_TypeDef *)CPUREG_BASE)
#define GCR             ((GCR_TypeDef *)GCR_BASE)
#define ITC             ((ITC_TypeDef *)ITC_BASE)

/**
  * @}
  */

#endif /* #ifndef STM8S003_H_ ... */
