/******************************************************************************
 * @file:    stm8s.h
 * @purpose: Header File for STM8S CPUs
 * @version: V1.0
 * @author:  Tymm Zerr
 * @date:    1. September 2015
 ******************************************************************************
 * @section License License
 * Licensed under a Simplified BSD License:
 *
 * Copyright (c) 2015, Tymm Zerr
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

#ifndef STM8S_H_
#define STM8S_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>


/* CPU Definitions ----------------------------------------------------------*/

#ifndef __STM8__
# define __STM8__
#endif

#ifndef __STM8S__
# define __STM8S__
#endif


/* Compiler Configuration Definitions ---------------------------------------*/

#if defined(SDCC)
/* Small Device C Compiler ( http://sdcc.sourceforge.net ) */
# define _SDCC_
#endif

/**
  * Definitions for IO register access
  */
#define __O     volatile
#define __I     volatile const
#define __IO    volatile


/* System Clock Frequencies -------------------------------------------------*/

#define HSI_FREQ   (16000000UL)    /*!< High Speed Internal Oscillator */
#define LSI_FREQ   (128000UL)      /*!< Low Speed Internal Oscillator */


/* Peripheral Registers -----------------------------------------------------*/

/** @brief GPIO Port Register Layout
  */
typedef struct {
    __IO uint8_t ODR;           /*!< 0x00: Output Data Register              */
    __IO uint8_t IDR;           /*!< 0x01: Input Data Register               */
    __IO uint8_t DDR;           /*!< 0x02: Data Direction Register           */
    __IO uint8_t CR1;           /*!< 0x03: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x04: Control Register 1                */
} GPIO_TypeDef;

/** @brief FLASH Peripheral Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
    __IO uint8_t NCR2;          /*!< 0x02: Complementary Control Reg. 2      */
    __IO uint8_t FPR;           /*!< 0x03: Protection Register               */
    __IO uint8_t NFPR;          /*!< 0x04: Complementary Protection Reg.     */
    __IO uint8_t IAPSR;         /*!< 0x05: Flash Status Register             */
    __IO uint8_t Reserved_0x06;
    __IO uint8_t Reserved_0x07;
    __IO uint8_t PUKR;          /*!< 0x08: Program Memory Unprotect Key Reg. */
    __IO uint8_t Reserved_0x09;
    __IO uint8_t DUKR;          /*!< 0x0a: Data EEPROM Unprotection Key Reg. */
} FLASH_TypeDef;

/** @brief Interrupt Controller Register Map
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
} EXTI_TypeDef;

/** @brief RESET Peripheral Register Map
  */
typedef struct {
    __IO uint8_t SR;            /*!< 0x00: Status Register                   */
} RST_TypeDef;

/** @brief Clock Control Register Map
  */
typedef struct {
    __IO uint8_t ICKR;          /*!< 0x00: Internal Clock Register           */
    __IO uint8_t ECKR;          /*!< 0x01: External Clock Register           */
    __IO uint8_t Reserved_0x02;
    __IO uint8_t CMSR;          /*!< 0x03: Clock Master Status Register      */
    __IO uint8_t SWR;           /*!< 0x04: Clock Master Switch Register      */
    __IO uint8_t SWCR;          /*!< 0x05: Switch Control Register           */
    __IO uint8_t CKDIVR;        /*!< 0x06: Clock Divide Register             */
    __IO uint8_t PCKENR1;       /*!< 0x07: Peripheral Clock Gating Reg. 1    */
    __IO uint8_t CSSR;          /*!< 0x08: Clock Security System Register    */
    __IO uint8_t CCOR;          /*!< 0x09: Configurable Clock Output Reg.    */
    __IO uint8_t PCKENR2;       /*!< 0x0a: Peripheral Clock Gating Reg. 2    */
    __IO uint8_t Reserved_0x0b;
    __IO uint8_t HSITRIMR;      /*!< 0x0c: HSI Trimmer Register              */
    __IO uint8_t SWIMCCR;       /*!< 0x0d: SWIM Clock Control Register       */
} CLK_TypeDef;

/** @brief Auto_Wakeup Register Layout
  */
typedef struct {
    __IO uint8_t CSR;           /*!< 0x00: Control / Status Register         */
    __IO uint8_t APR;           /*!< 0x01: Async Prescaler Register          */
    __IO uint8_t TBR;           /*!< 0x02: Timebase Selection Register       */
} AWU_TypeDef;

/** @brief Beeper Register Layout
  */
typedef struct {
    __IO uint8_t CSR;           /*!< 0x00: Control / Status Register         */
} BEEP_TypeDef;

/** @brief SPI Peripheral Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
    __IO uint8_t ICR;           /*!< 0x02: Interrupt Control Register        */
    __IO uint8_t SR;            /*!< 0x03: Status Register                   */
    __IO uint8_t DR;            /*!< 0x04: Data Register                     */
    __IO uint8_t CRCPR;         /*!< 0x05: CRC Polynomial Register           */
    __IO uint8_t RXCRC;         /*!< 0x06: RX CRC Register                   */
    __IO uint8_t TXCRC;         /*!< 0x07: TX CRC Register                   */
} SPI_TypeDef;

/** @brief I2C Peripheral Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
    __IO uint8_t FREQR;         /*!< 0x02: Frequency Register                */
    __IO uint8_t OARL;          /*!< 0x03: Own Address Register LSB          */
    __IO uint8_t OARH;          /*!< 0x04: Own Address Register MSB          */
         uint8_t Reserved_0x05;
    __IO uint8_t DR;            /*!< 0x06: Data Register                     */
    __IO uint8_t SR1;           /*!< 0x07: Status Register 1                 */
    __IO uint8_t SR2;           /*!< 0x08: Status Register 2                 */
    __IO uint8_t SR3;           /*!< 0x09: Status Register 3                 */
    __IO uint8_t ITR;           /*!< 0x0a: Interrupt Register                */
    __IO uint8_t CCRL;          /*!< 0x0b: Clock Control Register Low        */
    __IO uint8_t CCRH;          /*!< 0x0c: Clock Control Register High       */
    __IO uint8_t TRISER;        /*!< 0x0d: t_RISE Register                   */
} I2C_TypeDef;

/** @brief UART register layout
  */
typedef struct {
    __IO uint8_t SR;            /*!< 0x00: Status Register                   */
    __IO uint8_t DR;            /*!< 0x01: Data Register                     */
    __IO uint8_t BRR1;          /*!< 0x02: Baud Rate Register 1              */
    __IO uint8_t BRR2;          /*!< 0x03: Baud Rate Register 2              */
    __IO uint8_t CR1;           /*!< 0x04: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x05: Control Register 2                */
    __IO uint8_t CR3;           /*!< 0x06: Control Register 3                */
    __IO uint8_t CR4;           /*!< 0x07: Control Register 4                */
    __IO uint8_t CR5;           /*!< 0x08: Control Register 5                */
    __IO uint8_t CR6;           /*!< 0x09: Control Register 6                */
    __IO uint8_t GTR;           /*!< 0x0a: Guard Time Register               */
    __IO uint8_t PSCR;          /*!< 0x0b: Prescaler Register                */
} UART_TypeDef;

/** @brief Advanced 16-bit Timer Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
    __IO uint8_t SMCR;          /*!< 0x02: Slave Mode Control Register       */
    __IO uint8_t ETR;           /*!< 0x03: External Trigger Register         */
    __IO uint8_t IER;           /*!< 0x04: Interrupt Enable Register         */
    __IO uint8_t SR1;           /*!< 0x05: Status Register 1                 */
    __IO uint8_t SR2;           /*!< 0x06: Status Register 2                 */
    __IO uint8_t EGR;           /*!< 0x07: Event Generation Register         */
    __IO uint8_t CCMR1;         /*!< 0x08: Capture / Compare Mode Register 1 */
    __IO uint8_t CCMR2;         /*!< 0x09: Capture / Compare Mode Register 2 */
    __IO uint8_t CCMR3;         /*!< 0x0a: Capture / Compare Mode Register 3 */
    __IO uint8_t CCMR4;         /*!< 0x0b: Capture / Compare Mode Register 4 */
    __IO uint8_t CCER1;         /*!< 0x0c: Capture / Compare Enable Reg. 1   */
    __IO uint8_t CCER2;         /*!< 0x0d: Capture / Compare Enable Reg. 2   */
    __IO uint8_t CNTRH;         /*!< 0x0e: Counter High Register             */
    __IO uint8_t CNTRL;         /*!< 0x0f: Counter Low Register              */
    __IO uint8_t PSCRH;         /*!< 0x10: Prescaler High Register           */
    __IO uint8_t PSCRL;         /*!< 0x11: Prescaler Low Register            */
    __IO uint8_t ARRH;          /*!< 0x12: Auto Reload High Register         */
    __IO uint8_t ARRL;          /*!< 0x13: Auto Reload Low Register          */
    __IO uint8_t RCR;           /*!< 0x14: Repetition Counter Register       */
    __IO uint8_t CCR1H;         /*!< 0x15: Capture / Compare Register 1 High */
    __IO uint8_t CCR1L;         /*!< 0x16: Capture / Compare Register 1 Low  */
    __IO uint8_t CCR2H;         /*!< 0x17: Capture / Compare Register 2 High */
    __IO uint8_t CCR2L;         /*!< 0x18: Capture / Compare Register 2 Low  */
    __IO uint8_t CCR3H;         /*!< 0x19: Capture / Compare Register 3 High */
    __IO uint8_t CCR3L;         /*!< 0x1a: Capture / Compare Register 3 Low  */
    __IO uint8_t CCR4H;         /*!< 0x1b: Capture / Compare Register 4 High */
    __IO uint8_t CCR4L;         /*!< 0x1c: Capture / Compare Register 4 Low  */
    __IO uint8_t BKR;           /*!< 0x1d: Break Register                    */
    __IO uint8_t DTR;           /*!< 0x1e: Deadtime Register                 */
    __IO uint8_t OISR;          /*!< 0x1f: Output Idle State Register        */
} TIM1_TypeDef;

/** @brief General 16-bit Timer 2 Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
#ifdef TIM2_TYPE2
    uint8_t Reserved_0x01;
    uint8_t Reserved_0x02;
#endif
    __IO uint8_t IER;           /*!< 0x01/0x03: Interrupt Enable Register    */
    __IO uint8_t SR1;           /*!< 0x02/0x04: Status Register 1            */
    __IO uint8_t SR2;           /*!< 0x03/0x05: Status Register 2            */
    __IO uint8_t EGR;           /*!< 0x04/0x06: Event Generation Register    */
    __IO uint8_t CCMR1;         /*!< 0x05/0x07: Capt / Compare Mode Reg. 1   */
    __IO uint8_t CCMR2;         /*!< 0x06/0x08: Capt / Compare Mode Reg. 2   */
    __IO uint8_t CCMR3;         /*!< 0x07/0x09: Capt / Compare Mode Reg. 3   */
    __IO uint8_t CCER1;         /*!< 0x08/0x0a: Capt / Compare Enable Reg. 1 */
    __IO uint8_t CCER2;         /*!< 0x09/0x0b: Capt / Compare Enable Reg. 2 */
    __IO uint8_t CNTRH;         /*!< 0x0a/0x0c: Counter High Register        */
    __IO uint8_t CNTRL;         /*!< 0x0b/0x0d: Counter Low Register         */
    __IO uint8_t PSCR;          /*!< 0x0c/0x0e: Prescaler Register           */
    __IO uint8_t ARRH;          /*!< 0x0d/0x0f: Auto Reload High Register    */
    __IO uint8_t ARRL;          /*!< 0x0e/0x10: Auto Reload Low Register     */
    __IO uint8_t CCR1H;         /*!< 0x0f/0x11: Capt / Compare Reg. 1 High   */
    __IO uint8_t CCR1L;         /*!< 0x10/0x12: Capt / Compare Reg. 1 Low    */
    __IO uint8_t CCR2H;         /*!< 0x11/0x13: Capt / Compare Reg. 2 High   */
    __IO uint8_t CCR2L;         /*!< 0x12/0x14: Capt / Compare Reg. 2 Low    */
    __IO uint8_t CCR3H;         /*!< 0x13/0x15: Capt / Compare Reg. 3 High   */
    __IO uint8_t CCR3L;         /*!< 0x14/0x16: Capt / Compare Reg. 3 Low    */
} TIM2_TypeDef;

/** @brief 8-bit Timer 4 Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
#ifdef TIM4_TYPE2
    uint8_t Reserved_0x01;
    uint8_t Reserved_0x02;
#endif
    __IO uint8_t IER;           /*!< 0x01/0x03: Interrupt Enable Register    */
    __IO uint8_t SR;            /*!< 0x02/0x04: Status Register              */
    __IO uint8_t EGR;           /*!< 0x03/0x05: Event Generation Register    */
    __IO uint8_t CNTR;          /*!< 0x04/0x06: Counter Register             */
    __IO uint8_t PSCR;          /*!< 0x05/0x07: Prescaler Register           */
    __IO uint8_t ARR ;          /*!< 0x06/0x08: Auto Reload Register         */
} TIM4_TypeDef;

/** @brief 8-bit Timer 6 Register Layout
  */
typedef struct {
    __IO uint8_t CR1;           /*!< 0x00: Control Register 1                */
    __IO uint8_t CR2;           /*!< 0x01: Control Register 2                */
    __IO uint8_t SMCR;          /*!< 0x02: Slave Mode Control Register       */
    __IO uint8_t IER;           /*!< 0x03: Interrupt Enable Register         */
    __IO uint8_t SR;            /*!< 0x04: Status Register                   */
    __IO uint8_t EGR;           /*!< 0x05: Event Generation Register         */
    __IO uint8_t CNTR;          /*!< 0x06: Counter Register                  */
    __IO uint8_t PSCR;          /*!< 0x07: Prescaler Register                */
    __IO uint8_t ARR ;          /*!< 0x08: Auto Reload Register              */
} TIM4_TypeDef;

/** @brief ADC Channel Buffer Register Layout
  */
typedef struct {
    __IO uint8_t H;             /*!< Channel Data Buffer High */
    __IO uint8_t L;             /*!< Channel Data Buffer Low */
} ADC_DataBuffer_TypeDef;

/** @brief ADC1 Register Layout
  */
typedef struct {
    union {
        struct {
            __IO uint8_t DB0RH; /*!< Channel 0 Data Buffer High */
            __IO uint8_t DB0RL; /*!< Channel 0 Data Buffer Low */
            __IO uint8_t DB1RH; /*!< Channel 1 Data Buffer High */
            __IO uint8_t DB1RL; /*!< Channel 1 Data Buffer Low */
            __IO uint8_t DB2RH; /*!< Channel 2 Data Buffer High */
            __IO uint8_t DB2RL; /*!< Channel 2 Data Buffer Low */
            __IO uint8_t DB3RH; /*!< Channel 3 Data Buffer High */
            __IO uint8_t DB3RL; /*!< Channel 3 Data Buffer Low */
            __IO uint8_t DB4RH; /*!< Channel 4 Data Buffer High */
            __IO uint8_t DB4RL; /*!< Channel 4 Data Buffer Low */
            __IO uint8_t DB5RH; /*!< Channel 5 Data Buffer High */
            __IO uint8_t DB5RL; /*!< Channel 5 Data Buffer Low */
            __IO uint8_t DB6RH; /*!< Channel 6 Data Buffer High */
            __IO uint8_t DB6RL; /*!< Channel 6 Data Buffer Low */
            __IO uint8_t DB7RH; /*!< Channel 7 Data Buffer High */
            __IO uint8_t DB7RL; /*!< Channel 7 Data Buffer Low */
            __IO uint8_t DB8RH; /*!< Channel 8 Data Buffer High */
            __IO uint8_t DB8RL; /*!< Channel 8 Data Buffer Low */
            __IO uint8_t DB9RH; /*!< Channel 9 Data Buffer High */
            __IO uint8_t DB9RL; /*!< Channel 9 Data Buffer Low */
        };

        ADC_DataBuffer_TypeDef DBR[9];
    };

    uint8_t Reserved_0x14_0x1f[14];

    __IO uint8_t CSR;           /*!< 0x20: Control/Status Register           */
    __IO uint8_t CR1;           /*!< 0x21: Configuration Register 1          */
    __IO uint8_t CR2;           /*!< 0x22: Configuration Register 2          */
    __IO uint8_t CR3;           /*!< 0x23: Configuration Register 3          */
    __IO uint8_t DRH;           /*!< 0x24: Data Register High                */
    __IO uint8_t DRL;           /*!< 0x25: Data Register Low                 */
    __IO uint8_t TDRH;          /*!< 0x26: Schmitt Trigger Disable Reg. High */
    __IO uint8_t TDRL;          /*!< 0x27: Schmitt Trigger Disable Reg. Low  */
    __IO uint8_t HTRH;          /*!< 0x28: High Threshold Register High      */
    __IO uint8_t HTRL;          /*!< 0x29: High Threshold Register Low       */
    __IO uint8_t LTRH;          /*!< 0x2a: Low Threshold Register High       */
    __IO uint8_t LTRL;          /*!< 0x2b: High Threshold Register Low       */
    __IO uint8_t AWSRH;         /*!< 0x2c: Watchdog Status Register High     */
    __IO uint8_t AWSRL;         /*!< 0x2d: Watchdog Status Register Low      */
    __IO uint8_t AWCRH;         /*!< 0x2e: Watchdog Control Register High    */
    __IO uint8_t AWCRL;         /*!< 0x2f: Watchdog Status Register Low      */
} ADC1_TypeDef;

/** @brief ADC2 Register Layout
  */
typedef struct {
    __IO uint8_t CSR;           /*!< 0x00: Control/Status Register           */
    __IO uint8_t CR1;           /*!< 0x01: Configuration Register 1          */
    __IO uint8_t CR2;           /*!< 0x02: Configuration Register 2          */
    __IO uint8_t CR3;           /*!< 0x03: Configuration Register 3          */
    __IO uint8_t DRH;           /*!< 0x04: Data Register High                */
    __IO uint8_t DRL;           /*!< 0x05: Data Register Low                 */
    __IO uint8_t TDRH;          /*!< 0x06: Schmitt Trigger Disable Reg. High */
    __IO uint8_t TDRL;          /*!< 0x07: Schmitt Trigger Disable Reg Low   */
} ADC2_TypeDef;

/** @brief IWDG Register Layout
  */
typedef struct {
    __IO uint8_t KR;            /*!< 0x00: Key Register                      */
    __IO uint8_t PR;            /*!< 0x01: Prescaler Register                */
    __IO uint8_t RR;            /*!< 0x02: Reload Register                   */
} IWDG_TypeDef;

/** @brief WWDG Register Layout
  */
typedef struct {
    __IO uint8_t CR;            /*!< 0x00: Control Register                  */
    __IO uint8_t WR;            /*!< 0x01: Window Register                   */
} WWDG_TypeDef;

/** @brief CPU Register Layout
  */
typedef struct {
    __IO uint8_t A;             /*!< 0x00: Accumulator                       */
    __IO uint8_t PCE;           /*!< 0x01: Program Counter Extended          */
    __IO uint8_t PCH;           /*!< 0x02: Program Counter High              */
    __IO uint8_t PCL;           /*!< 0x03: Program Counter Low               */
    __IO uint8_t XH;            /*!< 0x04: X Index Register High             */
    __IO uint8_t XL;            /*!< 0x05: X Index Register Low              */
    __IO uint8_t YH;            /*!< 0x06: Y Index Register High             */
    __IO uint8_t YL;            /*!< 0x07: Y Index Register Low              */
    __IO uint8_t SPH;           /*!< 0x08: Stack Pointer High                */
    __IO uint8_t SPL;           /*!< 0x09: Stack Pointer Low                 */
    __IO uint8_t CCR;           /*!< 0x0a: Condition Code Register           */
} CPUREG_TypeDef;

/** @brief CPU Configuration Register Layout
  */
typedef struct {
    uint8_t GCR;                /*!< 0x00: Global Configuration Register     */
} CFG_TypeDef;

/** @brief Interrupt Software Priority Register Layout
  */
typedef struct {
    uint8_t SPR1;               /*!< 0x00: Interrupt SW Priority Register 1  */
    uint8_t SPR2;               /*!< 0x01: Interrupt SW Priority Register 2  */
    uint8_t SPR3;               /*!< 0x02: Interrupt SW Priority Register 3  */
    uint8_t SPR4;               /*!< 0x03: Interrupt SW Priority Register 4  */
    uint8_t SPR5;               /*!< 0x04: Interrupt SW Priority Register 5  */
    uint8_t SPR6;               /*!< 0x05: Interrupt SW Priority Register 6  */
    uint8_t SPR7;               /*!< 0x06: Interrupt SW Priority Register 7  */
    uint8_t SPR8;               /*!< 0x07: Interrupt SW Priority Register 8  */
} ITC_TypeDef;


/* Peripheral Register Bit Definitions  -------------------------------------*/

/** @defgroup GPIO_Register_Bit_Definitions GPIO Register Bit Definitions
  * @{
  */

/* GPIO Output Data Register Bit Definitions */
#define GPIO_ODR_7              (1 << 7)  /*!< GPIO Output Data Bit 7 */
#define GPIO_ODR_6              (1 << 6)  /*!< GPIO Output Data Bit 6 */
#define GPIO_ODR_5              (1 << 5)  /*!< GPIO Output Data Bit 5 */
#define GPIO_ODR_4              (1 << 4)  /*!< GPIO Output Data Bit 4 */
#define GPIO_ODR_3              (1 << 3)  /*!< GPIO Output Data Bit 3 */
#define GPIO_ODR_2              (1 << 2)  /*!< GPIO Output Data Bit 2 */
#define GPIO_ODR_1              (1 << 1)  /*!< GPIO Output Data Bit 1 */
#define GPIO_ODR_0              (1 << 0)  /*!< GPIO Output Data Bit 0 */

/* GPIO Input Data Register Bit Definitions */
#define GPIO_IDR_7              (1 << 7)  /*!< GPIO Input Data Bit 7 */
#define GPIO_IDR_6              (1 << 6)  /*!< GPIO Input Data Bit 6 */
#define GPIO_IDR_5              (1 << 5)  /*!< GPIO Input Data Bit 5 */
#define GPIO_IDR_4              (1 << 4)  /*!< GPIO Input Data Bit 4 */
#define GPIO_IDR_3              (1 << 3)  /*!< GPIO Input Data Bit 3 */
#define GPIO_IDR_2              (1 << 2)  /*!< GPIO Input Data Bit 2 */
#define GPIO_IDR_1              (1 << 1)  /*!< GPIO Input Data Bit 1 */
#define GPIO_IDR_0              (1 << 0)  /*!< GPIO Input Data Bit 0 */

/* GPIO Data Direction Register Bit Definitions */
#define GPIO_DDR_7              (1 << 7)  /*!< Data Dir / Bit 7 (0=in,1=out) */
#define GPIO_DDR_6              (1 << 6)  /*!< Data Dir / Bit 6 (0=in,1=out) */
#define GPIO_DDR_5              (1 << 5)  /*!< Data Dir / Bit 5 (0=in,1=out) */
#define GPIO_DDR_4              (1 << 4)  /*!< Data Dir / Bit 4 (0=in,1=out) */
#define GPIO_DDR_3              (1 << 3)  /*!< Data Dir / Bit 3 (0=in,1=out) */
#define GPIO_DDR_2              (1 << 2)  /*!< Data Dir / Bit 2 (0=in,1=out) */
#define GPIO_DDR_1              (1 << 1)  /*!< Data Dir / Bit 1 (0=in,1=out) */
#define GPIO_DDR_0              (1 << 0)  /*!< Data Dir / Bit 0 (0=in,1=out) */

/* GPIO Control Register 1 Bit Definitions */
#define GPIO_CR1_C17            (1 << 7)  /*!< GPIO Pin Ctrl 1 / Data Bit 7 */
#define GPIO_CR1_C16            (1 << 6)  /*!< GPIO Pin Ctrl 1 / Data Bit 6 */
#define GPIO_CR1_C15            (1 << 5)  /*!< GPIO Pin Ctrl 1 / Data Bit 5 */
#define GPIO_CR1_C14            (1 << 4)  /*!< GPIO Pin Ctrl 1 / Data Bit 4 */
#define GPIO_CR1_C13            (1 << 3)  /*!< GPIO Pin Ctrl 1 / Data Bit 3 */
#define GPIO_CR1_C12            (1 << 2)  /*!< GPIO Pin Ctrl 1 / Data Bit 2 */
#define GPIO_CR1_C11            (1 << 1)  /*!< GPIO Pin Ctrl 1 / Data Bit 1 */
#define GPIO_CR1_C10            (1 << 0)  /*!< GPIO Pin Ctrl 1 / Data Bit 0 */

/* GPIO Control Register 2 Bit Definitions */
#define GPIO_CR2_C27            (1 << 7)  /*!< GPIO Pin Ctrl 2 / Data Bit 7 */
#define GPIO_CR2_C26            (1 << 6)  /*!< GPIO Pin Ctrl 2 / Data Bit 6 */
#define GPIO_CR2_C25            (1 << 5)  /*!< GPIO Pin Ctrl 2 / Data Bit 5 */
#define GPIO_CR2_C24            (1 << 4)  /*!< GPIO Pin Ctrl 2 / Data Bit 4 */
#define GPIO_CR2_C23            (1 << 3)  /*!< GPIO Pin Ctrl 2 / Data Bit 3 */
#define GPIO_CR2_C22            (1 << 2)  /*!< GPIO Pin Ctrl 2 / Data Bit 2 */
#define GPIO_CR2_C21            (1 << 1)  /*!< GPIO Pin Ctrl 2 / Data Bit 1 */
#define GPIO_CR2_C20            (1 << 0)  /*!< GPIO Pin Ctrl 2 / Data Bit 0 */

/**
  * @}
  */

/** @defgroup FLASH_Register_Bit_Definitions FLASH Register Bit Definitions
  * @{
  */

#define FLASH_CR1_HALT          (1 << 3)  /*!< Power-Down in Halt Mode Enable */
#define FLASH_CR1_AHALT         (1 << 2)  /*!< Power-Down in Act Halt Mode En */
#define FLASH_CR1_IE            (1 << 1)  /*!< FLASH Interrupt Enable */
#define FLASH_CR1_FIX           (1 << 0)  /*!< Fixed Byte Prog Time Enable */

#define FLASH_CR2_OPT           (1 << 7)  /*!< Write to Option Bytes Enable */
#define FLASH_CR2_WPRG          (1 << 6)  /*!< Word Programming Enable */
#define FLASH_CR2_ERASE         (1 << 5)  /*!< Block Erasing Enable */
#define FLASH_CR2_FPRG          (1 << 4)  /*!< Fast Block Programming Enable */
#define FLASH_CR2_PRG           (1 << 0)  /*!< Standard Block Prog Enable */

#define FLASH_NCR2_OPT          (1 << 7)  /*!< Write to Option Bytes Disable */
#define FLASH_NCR2_WPRG         (1 << 6)  /*!< Word Programming Disable */
#define FLASH_NCR2_ERASE        (1 << 5)  /*!< Block Erasing Disable */
#define FLASH_NCR2_FPRG         (1 << 4)  /*!< Fast Block Programming Disable */
#define FLASH_NCR2_PRG          (1 << 0)  /*!< Standard Block Prog Disable */

#define FLASH_IAPSR_HVOFF       (1 << 6)  /*!< End of High Voltage Flag */
#define FLASH_IAPSR_DUL         (1 << 3)  /*!< Data EEPROM Unlocked Flag */
#define FLASH_IAPSR_EOP         (1 << 2)  /*!< End of Programming Flag */
#define FLASH_IAPSR_PUL         (1 << 1)  /*!< Flash Pgm Memory Unlocked Flag */
#define FLASH_IAPSR_WR_PG_DIS   (1 << 0)  /*!< Write Attempt / Prot Page Flag */

/**
  * @}
  */

/** @defgroup EXTI_Register_Bit_Definitions Ext Int Register Bit Definitions
  * @{
  */

/* EXTI CR1 Bit Definitions */
#define EXTI_CR1_PDIS1          (1 << 7)  /*!< Port D Interrupt Sense Bit 1 */
#define EXTI_CR1_PDIS0          (1 << 6)  /*!< Port D Interrupt Sense Bit 0 */
#define EXTI_CR1_PCIS1          (1 << 5)  /*!< Port C Interrupt Sense Bit 1 */
#define EXTI_CR1_PCIS0          (1 << 4)  /*!< Port C Interrupt Sense Bit 0 */
#define EXTI_CR1_PBIS1          (1 << 3)  /*!< Port B Interrupt Sense Bit 1 */
#define EXTI_CR1_PBIS0          (1 << 2)  /*!< Port B Interrupt Sense Bit 0 */
#define EXTI_CR1_PAIS1          (1 << 1)  /*!< Port A Interrupt Sense Bit 1 */
#define EXTI_CR1_PAIS0          (1 << 0)  /*!< Port A Interrupt Sense Bit 0 */

#define EXTI_CR1_Shift_Port_D       (6)
#define EXTI_CR1_Mask_Port_D        (0x03 << 6)

#define EXTI_CR1_Shift_Port_C       (4)
#define EXTI_CR1_Mask_Port_C        (0x03 << 4)

#define EXTI_CR1_Shift_Port_B       (2)
#define EXTI_CR1_Mask_Port_B        (0x03 << 2)

#define EXTI_CR1_Shift_Port_A       (0)
#define EXTI_CR1_Mask_Port_A        (0x03)

/* EXTI CR2 Bit Definitions */
#define EXTI_CR2_TLIS               (1 << 2)  /*!< Top Lvl Int Sens
                                                   (0=fall edge, 1=rise edge) */
#define EXTI_CR2_PEIS1              (1 << 1)  /*!< Port E Int Sense Bit 1 */
#define EXTI_CR2_PEIS0              (1 << 0)  /*!< Port E Int Sense Bit 0 */

#define EXTI_CR2_Shift_Port_E       (0)
#define EXTI_CR2_Mask_Port_E        (0x03)

#define EXTI_Sense_Low_Falling      (0)
#define EXTI_Sense_Rising_Edge      (1)
#define EXTI_Sense_Falling_Edge     (2)
#define EXTI_Sense_Both_Edges       (3)

/**
  * @}
  */

/** @defgroup RESET_Register_Bit_Definitions RESET Register Bit Definitions
  * @{
  */

/* RST SR Bit Definitions */
#define RST_SR_EMCF             (1 << 4)  /*!< EMC Reset Flag */
#define RST_SR_SWIMF            (1 << 3)  /*!< SWIM Reset Flag */
#define RST_SR_ILLOPF           (1 << 2)  /*!< Illegal Operation Reset Flag */
#define RST_SR_IWDGF            (1 << 1)  /*!< Independent Wdg Reset Flag */
#define RST_SR_WWDGF            (1 << 0)  /*!< Windowed Watchdog Reset Flag */

/**
  * @}
  */

/** @defgroup CLK_Register_Bit_Definitions Clock Ctrl Register Bit Definitions
  * @{
  */

/* Clock Control Internal Clock Register Bits */
#define CLK_ICKR_REGAH          (1 << 5)  /*!<*/
#define CLK_ICKR_LSIRDY         (1 << 4)  /*!< LSI Ready Flag */
#define CLK_ICKR_LSIEN          (1 << 3)  /*!< LSI Enable */
#define CLK_ICKR_FHWU           (1 << 2)
#define CLK_ICKR_HSIRDY         (1 << 1)  /*!< HSI Ready Flag */
#define CLK_ICKR_HSIEN          (1 << 0)  /*!< HSI Enable */

/* Clock Control External Clock Register Bits */
#define CLK_ECKR_HSERDY         (1 << 1)  /*!< HSE Ready Flag */
#define CLK_ECKR_HSEEN          (1 << 0)  /*!< HSE Enable */

/* Clock Control Master Status Register Bits */
#define CLK_SWCR_SWIF           (1 << 3)  /*!< Clock Switch Interrupt Flag */
#define CLK_SWCR_SWIEN          (1 << 2)  /*!< Clock Switch Interrupt Enable */
#define CLK_SWCR_SWEN           (1 << 1)  /*!< Clock Switch Enable Flag */
#define CLK_SWCR_SWBSY          (1 << 0)  /*!< Clock Switch Busy Flag */

/* Clock Control Divide Register Bits */
#define CLK_CKDIVR_HSIDIV1      (1 << 4)  /*!< HSI Clock Divider Bit 1 */
#define CLK_CKDIVR_HSIDIV0      (1 << 3)  /*!< HSI Clock Divider Bit 0 */
#define CLK_CKDIVR_CPUDIV2      (1 << 2)  /*!< CPU Clock Divider Bit 2 */
#define CLK_CKDIVR_CPUDIV1      (1 << 1)  /*!< HSI Clock Divider Bit 1 */
#define CLK_CKDIVR_CPUDIV0      (1 << 0)  /*!< HSI Clock Divider Bit 0 */

#define CLK_CKDIVR_HSIDIV_Shift (3)
#define CLK_CKDIVR_HSIDIV_Mask  (0x03 << 3)

#define CLK_CKDIVR_CPUDIV_Shift (0)
#define CLK_CKDIVR_CPUDIV_Mask  (0x07)

/* Clock Control Security System Register Bits */
#define CLK_CSSR_CSSD           (1 << 3)  /*!< Clock Sec Disturbance Det Flag */
#define CLK_CSSR_CSSDIE         (1 << 3)  /*!< Clock Sec Disturbance Int En */
#define CLK_CSSR_AUX            (1 << 3)  /*!< Auxiliary Oscillator Connected */
#define CLK_CSSR_CSSEN          (1 << 3)  /*!< CSS Enable */

/* Clock Control Configurable Clock Output Register Bits */
#define CLK_CCOR_CCOBSY         (1 << 6)  /*!< Config Clk Output Operating Fl */
#define CLK_CCOR_CCORDY         (1 << 5)  /*!< Config Clk Output Ready */
#define CLK_CCOR_CCOSEL3        (1 << 4)  /*!< Config Clk Output Select Bit 3 */
#define CLK_CCOR_CCOSEL2        (1 << 3)  /*!< Config Clk Output Select Bit 2 */
#define CLK_CCOR_CCOSEL1        (1 << 2)  /*!< Config Clk Output Select Bit 1 */
#define CLK_CCOR_CCOSEL0        (1 << 1)  /*!< Config Clk Output Select Bit 0 */
#define CLK_CCOR_CCOEN          (1 << 0)  /*!< Config Clk Output Enable */

#define CLK_CCOR_CCOSEL_Shift   (1)
#define CLK_CCOR_CCOSEL_Mask    (0x0f << 1)

/* Clock Control HSI Trimmer Register Bits */
#define CLK_HSITRIMR_Shift      (0)
#define CLK_HSITRIMR_Mask       (0x07)

/* Clock Control SWIM Clock Control Register Bits */
#define CLK_SWIMCCR_SWIMCLK     (1 << 0)

/* Clock Source Values */

#define CLK_SOURCE_HSI          (0xe1)
#define CLK_SOURCE_LSI          (0xd2)
#define CLK_SOURCE_HSE          (0xb4)

/**
  * @}
  */

/** @defgroup AWU_Register_Bit_Definitions Auto_Wakeup Register Bit Definitions
  * @{
  */

/* Auto-Wakeup Control / Status Register Bit Definitions */
#define AWU_CSR_AWUF            (1 << 5)  /*!< Auto-Wakeup Flag */
#define AWU_CSR_AWUEN           (1 << 4)  /*!< Auto-Wakeup Enable */
#define AWU_CSR_MSR             (1 << 0)  /*!< Measurement Enable */

/* Auto-Wakeup Timebase Selection Register Bit Definitions */
#define AWU_TBR_AWUTB3          (1 << 3)  /*!< Auto-Wkup Timebase Sel Bit 3 */
#define AWU_TBR_AWUTB2          (1 << 2)  /*!< Auto-Wkup Timebase Sel Bit 2 */
#define AWU_TBR_AWUTB1          (1 << 1)  /*!< Auto-Wkup Timebase Sel Bit 1 */
#define AWU_TBR_AWUTB0          (1 << 0)  /*!< Auto-Wkup Timebase Sel Bit 0 */

#define AWU_TBR_Timebase_Shift  (0)
#define AWU_TBR_Timebase_Mask   (0x0f)

/**
  * @}
  */

/** @defgroup BEEP_Register_Bit_Definitions Beeper Register Bit Definitions
  * @{
  */

/* Beeper Control / Status Register Bit Definitions */
#define BEEP_CSR_SEL1           (1 << 7)  /*!< Beep Selection Bit 1 */
#define BEEP_CSR_SEL0           (1 << 6)  /*!< Beep Selection Bit 0 */
#define BEEP_CSR_EN             (1 << 5)  /*!< Beeper Enable */

/* Beep Prescaler Divider Bits */
#define BEEP_CSR_DIV4           (1 << 4)  /*!< Beeper Divider Bit 4 */
#define BEEP_CSR_DIV3           (1 << 3)  /*!< Beeper Divider Bit 3 */
#define BEEP_CSR_DIV2           (1 << 2)  /*!< Beeper Divider Bit 2 */
#define BEEP_CSR_DIV1           (1 << 1)  /*!< Beeper Divider Bit 1 */
#define BEEP_CSR_DIV0           (1 << 0)  /*!< Beeper Divider Bit 0 */

#define BEEP_Select_Shift       (6)
#define BEEP_Select_Mask        (0x03 << 6)

#define BEEP_Divider_Shift      (0)
#define BEEP_Divider_Mask       (0x1f)

/**
  * @}
  */

/** @defgroup SPI_Register_Bit_Definitions SPI Register Bit Definitions
  * @{
  */

/* CR1 Bit Definitions */
#define SPI_CR1_LSBFIRST        (1 << 7)  /*!< Frm Fmt (0=MSB 1st, 1=LSB 1st) */
#define SPI_CR1_SPE             (1 << 6)  /*!< SPI Peripheral Enable */
#define SPI_CR1_BR2             (1 << 5)  /*!< Baud Rate Control Bit 2 */
#define SPI_CR1_BR1             (1 << 4)  /*!< Baud Rate Control Bit 1 */
#define SPI_CR1_BR0             (1 << 3)  /*!< Baud Rate Control Bit 0 */
#define SPI_CR1_MSTR            (1 << 2)  /*!< Mstr Sel (0=Slave, 1=Master) */
#define SPI_CR1_CPOL            (1 << 1)  /*!< Clk Pol (0=SCK idl 0, 1=idl 1) */
#define SPI_CR1_CPHA            (1 << 0)  /*!< Clk Phs (0=cap 1st clk, 1=2nd) */

/* Shift & Bitmask for SPI Baud setting in CR1
 * Baud rate = (fMaster/2^(n+1))
 */
#define SPI_CR1_Baud_Shift      (3)
#define SPI_CR1_Baud_Mask       (0x07 << 3)

/* CR2 Bit Definitions */
#define SPI_CR2_BDM             (1 << 7)  /*!< Bidirectional Data Mode Enable
                                               0 : 2 line unidir
                                               1 : 1 line bidir */
#define SPI_CR2_BDOE            (1 << 6)  /*!< I/O Enable in Bidirectional Mode
                                               0 : input
                                               1 : output */
#define SPI_CR2_CRCEN           (1 << 5)  /*!< HW CRC Calculation Enable */
#define SPI_CR2_CRCNEXT         (1 << 4)  /*!< Transmit CRC Next */
#define SPI_CR2_RXONLY          (1 << 2)  /*!< Rcv Only
                                               0 : Full duplex
                                               1 : Output disabled*/
#define SPI_CR2_SSM             (1 << 1)  /*!< SW Slave Management Enable */
#define SPI_CR2_SSI             (1 << 0)  /*!< SW Slave Sel/SSM Mode
                                               0 : Slave
                                               1 : Master */

/* ICR Bit Definitions */
#define SPI_ICR_TXIE            (1 << 7)  /*!< Tx Buffer Empty Int Enable */
#define SPI_ICR_RXIE            (1 << 6)  /*!< Rx Buffer Not Empty Int Enable */
#define SPI_ICR_ERRIE           (1 << 5)  /*!< Error Interrupt Enable */
#define SPI_ICR_WKIE            (1 << 4)  /*!< Wakeup Interrupt Enable */

/* SR Bit Definitions */
#define SPI_SR_BSY              (1 << 7)  /*!< Busy Flag */
#define SPI_SR_OVR              (1 << 6)  /*!< Overrun Flag */
#define SPI_SR_MODF             (1 << 5)  /*!< Mode Fault Flag */
#define SPI_SR_CRCERR           (1 << 4)  /*!< CRC Error Flag */
#define SPI_SR_WKUP             (1 << 3)  /*!< Wakeup Flag */
#define SPI_SR_TXE              (1 << 1)  /*!< Tx Buffer Empty Flag */
#define SPI_SR_RXNE             (1 << 0)  /*!< Rx Buffer Not Empty Flag */

/**
  * @}
  */

/** @defgroup I2C_Register_Bit_Definitions I2C Register Bit Definitions
  * @{
  */

/* I2C CR1 Bit Definitions */
#define I2C_CR1_NOSTRETCH       (1 << 7)  /*!< Disable Clock Stretching */
#define I2C_CR1_ENGC            (1 << 6)  /*!< Enable General Call */
#define I2C_CR1_PE              (1 << 0)  /*!< I2C Peripheral Enable */

/* I2C CR2 Bit Definitions */
#define I2C_CR2_SWRST           (1 << 7)  /*!< Software Reset */
#define I2C_CR2_POS             (1 << 3)  /*!< ACK Pos (0=curr B, 1=next B) */
#define I2C_CR2_ACK             (1 << 2)  /*!< Acknowledge Enable */
#define I2C_CR2_STOP            (1 << 1)  /*!< Stop Generation */
#define I2C_CR2_START           (1 << 0)  /*!< Start Generation */

/* I2C FREQ bit definitions */
#define I2C_FREQ_FREQUENCY_Shift (0)
#define I2C_FREQ_FREQUENCY_Mask  (0x3f)

/* I2C OARL bit definitions */
#define I2C_OARL_7BITADDR_Shift (1)
#define I2C_OARL_7BITADDR_Mask  (0xfe)

/* I2C OARH bit definitions */
#define I2C_OARH_ADDMODE        (1 << 7)  /*!< Addr Mode (0=7bit, 1=10bit) */
#define I2C_OARH_ADDCONF        (1 << 6)  /*!< Address Mode Configuration */
#define I2C_OARH_ADD9           (1 << 2)  /*!< 10 bit address bit 9 */
#define I2C_OARH_ADD8           (1 << 1)  /*!< 10 bit address bit 8 */

/* Shift and Mask for top 2 bits of 10-bit address */
#define I2C_OARH_10BITADDR_B8_9_Shift (1)
#define I2C_OARH_10BITADDR_B8_9_Mask  (0x03 << 1)

/* I2C SR1 bit definitions */
#define I2C_SR1_TXE             (1 << 7)  /*!< Data Register TX Empty */
#define I2C_SR1_RXNE            (1 << 6)  /*!< Data Register RX Not Empty */
#define I2C_SR1_STOPF           (1 << 4)  /*!< Stop Detected */
#define I2C_SR1_ADD10           (1 << 3)  /*!< 10-bit Header Sent (mstr mode) */
#define I2C_SR1_BTF             (1 << 2)  /*!< Byte Transfer Finished */
#define I2C_SR1_ADDR            (1 << 1)  /*!< Addr Sent (mm) / Matched (sm) */
#define I2C_SR1_SB              (1 << 0)  /*!< Start Bit Generated (mm) */

/* I2C SR2 bit definitions */
#define I2C_SR2_WUFH            (1 << 5)  /*!< Wakeup from Halt */
#define I2C_SR2_OVR             (1 << 3)  /*!< Overrun / Underrun */
#define I2C_SR2_AF              (1 << 2)  /*!< Acknowledge Failure */
#define I2C_SR2_ARLO            (1 << 1)  /*!< Arbitration Lost (master mode) */
#define I2C_SR2_BERR            (1 << 0)  /*!< Bus Error */

/* I2C SR3 bit definitions */
#define I2C_SR3_GENCALL         (1 << 4)  /*!< General Call Hdr Received (sm) */
#define I2C_SR3_TRA             (1 << 2)  /*!< Transmitter / Receiver */
#define I2C_SR3_BUSY            (1 << 1)  /*!< Busy Flag */
#define I2C_SR3_MSL             (1 << 0)  /*!< Mstr/Slv Mode (0=slv, 1=mstr) */

/* I2C ITR bit definitions */
#define I2C_ITR_ITBUFEN         (1 << 2)  /*!< Buffer Interrupt Enable */
#define I2C_ITR_ITEVTEN         (1 << 1)  /*!< Event Interrupt Enable */
#define I2C_ITR_ITERREN         (1 << 0)  /*!< Error Interrupt Enable */

/* I2C CCRL bit definitions */
#define I2C_CCRL_CLOCK_Shift    (0)
#define I2C_CCRL_CLOCK_Mask     (0xff)

/* I2C CCRH bit definitions */
#define I2C_CCRH_FS             (1 << 7)  /*!< Mstr Mode Sel (0=std, 1=fast) */
#define I2C_CCRH_DUTY           (1 << 6)  /*!< Fast Md Duty l/h (0=2, 1=16/9) */

#define I2C_CCRH_CLOCK_Shift    (0)
#define I2C_CCRH_CLOCK_Mask     (0x0f)

/* I2C TRISER bit definitions */
#define I2C_TRISER_TRISE_Shift  (0)
#define I2C_TRISER_TRISE_Mask   (0x3f)

/**
  * @}
  */

/** @defgroup UART_Register_Bit_Definitions UART Register Bit Definitions
  * @{
  */

/* UART Status Register Bit Definitions  */
#define UART_SR_TXE             (1 << 7)  /*!< Transmit Data Register Empty */
#define UART_SR_TXC             (1 << 6)  /*!< Transmission Complete */
#define UART_SR_RXNE            (1 << 5)  /*!< Read Data Register Not Empty */
#define UART_SR_IDLE            (1 << 4)  /*!< IDLE Line Detected */
#define UART_SR_OR              (1 << 3)  /*!< Overrun Error */
#define UART_SR_NF              (1 << 2)  /*!< Noise Flag */
#define UART_SR_FE              (1 << 1)  /*!< Framing Error */
#define UART_SR_PE              (1 << 0)  /*!< Parity Error */

/* UART Control Register 1 Bit Definitions */
#define UART_CR1_R8             (1 << 7)  /*!< Receive Databit 8 */
#define UART_CR1_T8             (1 << 6)  /*!< Transmit Databit 8 */
#define UART_CR1_UARTD          (1 << 5)  /*!< Uart Disable */
#define UART_CR1_M              (1 << 4)  /*!< Word Length */
#define UART_CR1_WAKE           (1 << 3)  /*!< Wake Meth (0=idle, 1=addr mark */
#define UART_CR1_PCEN           (1 << 2)  /*!< Parity Control Enable */
#define UART_CR1_PS             (1 << 1)  /*!< Parity Select (0=even, 1=odd) */
#define UART_CR1_PIEN           (1 << 0)  /*!< Parity Interrupt Enable */

#define UART_CR1_Parity_Shift   (1)
#define UART_CR1_Parity_Mask    (0x03 << 1)

#define UART_CR1_Parity_Even    (UART_CR1_PCEN)
#define UART_CR1_Parity_Odd     (UART_CR1_PCEN | UART_CR1_PS)
#define UART_CR1_Parity_None    (0)

/* UART Control Register 2 Bit Definitions */
#define UART_CR2_TIEN           (1 << 7)  /*!< Transmitter Interrupt Enable */
#define UART_CR2_TCIEN          (1 << 6)  /*!< Transmission Complete Int En */
#define UART_CR2_RIEN           (1 << 5)  /*!< Receiver Interrupt Enable */
#define UART_CR2_ILIEN          (1 << 4)  /*!< IDLE Line Interrupt Enable */
#define UART_CR2_TEN            (1 << 3)  /*!< Transmitter Enable */
#define UART_CR2_REN            (1 << 2)  /*!< Receiver Enable */
#define UART_CR2_RWU            (1 << 1)  /*!< Receiver Wakeup */
#define UART_CR2_SBK            (1 << 0)  /*!< Send Break */

/* UART Control Register 3 Bit Definitions */
#define UART_CR3_LINEN          (1 << 6)  /*!< LIN Mode Enable */
#define UART_CR3_STOP1          (1 << 5)  /*!< Stop Bits Control Bit 1 */
#define UART_CR3_STOP0          (1 << 4)  /*!< Stop Bits Control Bit 0 */
#define UART_CR3_CLKEN          (1 << 3)  /*!< SCLK Pin Enable */
#define UART_CR3_CPOL           (1 << 2)  /*!< SCLK Polarity */
#define UART_CR3_CPHA           (1 << 1)  /*!< SCLK Phase */
#define UART_CR3_LBCL           (1 << 0)  /*!< Last Bit Clock PUlse */


/* Number of bit shifts for UART CR3 STOP bit control bits */
#define UART_CR3_STOP_Shift (4)
/* Mask for UART CR3 STOP bit control bits */
#define UART_CR3_STOP_Mask  (0x03 << 4)

/* CR3 STOP selection for 1 stop bit */
#define UART_CR3_STOPBITS_1   (0)
/* CR3 STOP selection for 2 stop bits */
#define UART_CR3_STOPBITS_2   (2 << UART_CR3_STOP_Shift)
/* CR3 STOP selection for 1.5 stop bits */
#define UART_CR3_STOPBITS_1_5 (3 << UART_CR3_STOP_Shift)

/* UART Control Register 4 Bit Definitions */
#define UART_CR4_LBDIEN         (1 << 6)  /*!< LIN BRK Detect Int Enable */
#define UART_CR4_LBDL           (1 << 5)  /*!< LIN BRK Det. Len (0=10b 1=11b) */
#define UART_CR4_LBDF           (1 << 4)  /*!< LIN BRK Detection Flag */
#define UART_CR4_ADD3           (1 << 3)  /*!< UART Node Address Bit 3 */
#define UART_CR4_ADD2           (1 << 2)  /*!< UART Node Address Bit 2 */
#define UART_CR4_ADD1           (1 << 1)  /*!< UART Node Address Bit 1 */
#define UART_CR4_ADD0           (1 << 0)  /*!< UART Node Address Bit 0 */

/* Number of bit shifts for UART CR4 ADD bits */
#define UART_CR4_ADD_Shift  (0)
/* Mask for UART CR4 ADD bits */
#define UART_CR4_ADD_Mask   (0x0f)

/* UART Control Register 5 Bit Definitions */
#define UART_CR5_SCEN           (1 << 5)  /*!< Smartcard Mode Enable */
#define UART_CR5_NACK           (1 << 4)  /*!< Smartcard NACK Enable */
#define UART_CR5_HDSEL          (1 << 3)  /*!< Half-Duplex Select */
#define UART_CR5_IRLP           (1 << 2)  /*!< IrDA Low Power */
#define UART_CR5_IREN           (1 << 1)  /*!< IrDA Mode Enable */

/* UART Control Register 6 Bit Definitions */
#define UART_CR6_LDUM           (1 << 7)  /*!< LIN Divider Update Method */
#define UART_CR6_LSLV           (1 << 5)  /*!< LIN Slave Enable */
#define UART_CR6_LASE           (1 << 4)  /*!< LIN Auto-resync Enable */
#define UART_CR6_LHDIEN         (1 << 2)  /*!< LIN Header Detect Int. Enable */
#define UART_CR6_LHDF           (1 << 1)  /*!< LIN Header Detection Flag */
#define UART_CR6_LSF            (1 << 0)  /*!< LIN Sync Field */

/**
  * @}
  */

/** @defgroup TIM_Register_Bit_Definitions TIM Register Bit Definitions
  * @{
  */

/* TIM CR1 Bit Definitions */
#define TIM_CR1_ARPE            (1 << 7)  /*!< Auto-Reload Preload Enable */
#define TIM_CR1_OPM             (1 << 3)  /*!< One Pulse Mode Enable */
#define TIM_CR1_URS             (1 << 2)  /*!< Update Request Source */
#define TIM_CR1_UDIS            (1 << 1)  /*!< Update Disable */
#define TIM_CR1_CEN             (1 << 0)  /*!< Counter Enable */

/* TIM CR2 Bit Definitions (TIM5) */
#define TIM_CR2_MMS2            (1 << 6)  /*!< Master Mode Selection Bit 2 */
#define TIM_CR2_MMS1            (1 << 5)  /*!< Master Mode Selection Bit 1 */
#define TIM_CR2_MMS0            (1 << 4)  /*!< Master Mode Selection Bit 0 */

/* Shift and Mask for Master Mode Selection Bits */
#define TIM_CR2_MMS_Shift       (4)
#define TIM_CR2_MMS_Mask        (0x07 << 4)

/* TIM SMCR Bit Definitions (TIM5) */
#define TIM_CR2_MSM             (1 << 7)  /*!< Master/Slave Mode */
#define TIM_CR2_TS2             (1 << 6)  /*!< Trigger Selection Bit 2 */
#define TIM_CR2_TS1             (1 << 5)  /*!< Trigger Selection Bit 1 */
#define TIM_CR2_TS0             (1 << 4)  /*!< Trigger Selection Bit 0 */
#define TIM_CR2_SMS2            (1 << 2)  /*!< Clock/Trig/Slv Mode Sel Bit 2 */
#define TIM_CR2_SMS1            (1 << 1)  /*!< Clock/Trig/Slv Mode Sel Bit 1 */
#define TIM_CR2_SMS0            (1 << 0)  /*!< Clock/Trig/Slv Mode Sel Bit 0 */

/* TIM IER Bit Definitions  */
#define TIM_IER_TIE             (1 << 6)  /*!< Trigger Interrupt Enable */
#define TIM_IER_CC3IE           (1 << 3)  /*!< Capt/Cmp 3 Interrupt Enable */
#define TIM_IER_CC2IE           (1 << 2)  /*!< Capt/Cmp 2 Interrupt Enable */
#define TIM_IER_CC1IE           (1 << 1)  /*!< Capt/Cmp 1 Interrupt Enable */
#define TIM_IER_UIE             (1 << 0)  /*!< Update Interrupt Enable */

/* TIM SR1 Bit Definitions  */
#define TIM_SR1_TIF             (1 << 6)  /*!< Trigger Interrupt Flag */
#define TIM_SR1_CC3IF           (1 << 3)  /*!< Capt/Compare 3 Interrupt Flag */
#define TIM_SR1_CC2IF           (1 << 2)  /*!< Capt/Compare 2 Interrupt Flag */
#define TIM_SR1_CC1IF           (1 << 1)  /*!< Capt/Compare 1 Interrupt Flag */
#define TIM_SR1_UIF             (1 << 0)  /*!< Update Interrupt Flag */

/* TIM SR2 Bit Definitions  */
#define TIM_SR2_CC3OF           (1 << 3)  /*!< Capt/Cmp 3 Overcapture Flag */
#define TIM_SR2_CC2OF           (1 << 2)  /*!< Capt/Cmp 2 Overcapture Flag */
#define TIM_SR2_CC1OF           (1 << 1)  /*!< Capt/Cmp 1 Overcapture Flag */

/* TIM EGR Bit Definitions  */
#define TIM_EGR_TG              (1 << 6)  /*!< Trigger Generation */
#define TIM_EGR_CC3G            (1 << 3)  /*!< Capt/Compare 3 Generation */
#define TIM_EGR_CC2G            (1 << 2)  /*!< Capt/Compare 2 Generation */
#define TIM_EGR_CC1G            (1 << 1)  /*!< Capt/Compare 1 Generation */
#define TIM_EGR_UG              (1 << 0)  /*!< Update Generation */

/* TIM CCMR1 Bit Definitions */
#define TIM_CCMR1_OC1M2         (1 << 6)  /*!< Output Cmp 1 Mode Bit 2 */
#define TIM_CCMR1_OC1M1         (1 << 5)  /*!< Output Cmp 1 Mode Bit 1 */
#define TIM_CCMR1_OC1M0         (1 << 4)  /*!< Output Cmp 1 Mode Bit 0 */
#define TIM_CCMR1_OC1PE         (1 << 3)  /*!< Output Cmp 1 Preload Enable */
#define TIM_CCMR1_CC1S1         (1 << 1)  /*!< Output Cmp 1 Selection Bit 1 */
#define TIM_CCMR1_CC1S0         (1 << 0)  /*!< Output Cmp 1 Selection Bit 0 */

#define TIM_CCMR1_OC1M_Shift    (4)
#define TIM_CCMR1_OC1M_Mask     (0x07 << 4)

#define TIM_CCMR1_CC1S_Shift    (0)
#define TIM_CCMR1_CC1S_Mask     (0x03)

#define TIM_CCMR1_IC1F3         (1 << 7)  /*!< Input Capture 1 Filter Bit 3 */
#define TIM_CCMR1_IC1F2         (1 << 6)  /*!< Input Capture 1 Filter Bit 2 */
#define TIM_CCMR1_IC1F1         (1 << 5)  /*!< Input Capture 1 Filter Bit 1 */
#define TIM_CCMR1_IC1F0         (1 << 4)  /*!< Input Capture 1 Filter Bit 0 */

#define TIM_CCMR1_IC1PSC1       (1 << 3)  /*!< Input Capt 1 Prescaler Bit 1 */
#define TIM_CCMR1_IC1PSC0       (1 << 2)  /*!< Input Capt 1 Prescaler Bit 0 */

#define TIM_CCMR1_CC1S1         (1 << 1)  /*!< Capt / Cmp 1 Selection Bit 1 */
#define TIM_CCMR1_CC1S0         (1 << 0)  /*!< Capt / Cmp 1 Selection Bit 0 */

#define TIM_CCMR1_IC1F_Shift    (4)
#define TIM_CCMR1_IC1F_Mask     (0x0f << 4)

#define TIM_CCMR1_IC1PSC_Shift  (2)
#define TIM_CCMR1_IC1PSC_Mask   (0x03 << 2)

#define TIM_CCMR1_CC1S_Shift    (0)
#define TIM_CCMR1_CC1S_Mask     (0x03)

/* TIM CCMR2 Bit Definitions */
#define TIM_CCMR2_OC1M2         (1 << 6)  /*!< Output Compare 1 Mode Bit 2 */
#define TIM_CCMR2_OC1M1         (1 << 5)  /*!< Output Compare 1 Mode Bit 1 */
#define TIM_CCMR2_OC1M0         (1 << 4)  /*!< Output Compare 1 Mode Bit 0 */
#define TIM_CCMR2_OC1PE         (1 << 3)  /*!< Output Compare 1 Preload Ena. */
#define TIM_CCMR2_CC1S1         (1 << 1)  /*!< Output Compare 1 Select Bit 1 */
#define TIM_CCMR2_CC1S0         (1 << 0)  /*!< Output Compare 1 Select Bit 0 */

#define TIM_CCMR2_OC1M_Shift    (4)
#define TIM_CCMR2_OC1M_Mask     (0x07 << 4)

#define TIM_CCMR2_CC1S_Shift    (0)
#define TIM_CCMR2_CC1S_Mask     (0x03)

#define TIM_CCMR2_IC1F3         (1 << 7)  /*!< Input Capture 1 Filter Bit 3 */
#define TIM_CCMR2_IC1F2         (1 << 6)  /*!< Input Capture 1 Filter Bit 2 */
#define TIM_CCMR2_IC1F1         (1 << 5)  /*!< Input Capture 1 Filter Bit 1 */
#define TIM_CCMR2_IC1F0         (1 << 4)  /*!< Input Capture 1 Filter Bit 0 */

#define TIM_CCMR2_IC1PSC1       (1 << 3)  /*!< Input Capt 1 Prescaler Bit 1 */
#define TIM_CCMR2_IC1PSC0       (1 << 2)  /*!< Input Capt 1 Prescaler Bit 0 */

#define TIM_CCMR2_CC1S1         (1 << 1)  /*!< Capt / Cmp 1 Selection Bit 1 */
#define TIM_CCMR2_CC1S0         (1 << 0)  /*!< Capt / Cmp 1 Selection Bit 0 */

#define TIM_CCMR2_IC1F_Shift    (4)
#define TIM_CCMR2_IC1F_Mask     (0x0f << 4)

#define TIM_CCMR2_IC1PSC_Shift  (2)
#define TIM_CCMR2_IC1PSC_Mask   (0x03 << 2)

#define TIM_CCMR2_CC1S_Shift    (0)
#define TIM_CCMR2_CC1S_Mask     (0x03)

/* TIM CCMR3 Bit Definitions */
#define TIM_CCMR3_OC1M2         (1 << 6)  /*!< Output Compare 1 Mode Bit 2 */
#define TIM_CCMR3_OC1M1         (1 << 5)  /*!< Output Compare 1 Mode Bit 1 */
#define TIM_CCMR3_OC1M0         (1 << 4)  /*!< Output Compare 1 Mode Bit 0 */
#define TIM_CCMR3_OC1PE         (1 << 3)  /*!< Output Cmp 1 Preload Enable */
#define TIM_CCMR3_CC1S1         (1 << 1)  /*!< Output Cmp 1 Selection Bit 1 */
#define TIM_CCMR3_CC1S0         (1 << 0)  /*!< Output Cmp 1 Selection Bit 0 */

#define TIM_CCMR3_OC1M_Shift    (4)
#define TIM_CCMR3_OC1M_Mask     (0x07 << 4)

#define TIM_CCMR3_CC1S_Shift    (0)
#define TIM_CCMR3_CC1S_Mask     (0x03)

#define TIM_CCMR3_IC1F3         (1 << 7)  /*!< Input Capture 1 Filter Bit 3 */
#define TIM_CCMR3_IC1F2         (1 << 6)  /*!< Input Capture 1 Filter Bit 2 */
#define TIM_CCMR3_IC1F1         (1 << 5)  /*!< Input Capture 1 Filter Bit 1 */
#define TIM_CCMR3_IC1F0         (1 << 4)  /*!< Input Capture 1 Filter Bit 0 */

#define TIM_CCMR3_IC1PSC1       (1 << 3)  /*!< Input Capt 1 Prescaler Bit 1 */
#define TIM_CCMR3_IC1PSC0       (1 << 2)  /*!< Input Capt 1 Prescaler Bit 0 */

#define TIM_CCMR3_CC1S1         (1 << 1)  /*!< Capt / Cmp 1 Selection Bit 1 */
#define TIM_CCMR3_CC1S0         (1 << 0)  /*!< Capt / Cmp 1 Selection Bit 0 */

#define TIM_CCMR3_IC1F_Shift    (4)
#define TIM_CCMR3_IC1F_Mask     (0x0f << 4)

#define TIM_CCMR3_IC1PSC_Shift  (2)
#define TIM_CCMR3_IC1PSC_Mask   (0x03 << 2)

#define TIM_CCMR3_CC1S_Shift    (0)
#define TIM_CCMR3_CC1S_Mask     (0x03)

/**
  * @}
  */

/** @defgroup ADC_Register_Bit_Definitions ADC Register Bit Definitions
  * @{
  */

/* ADC CSR Bit Definitions */
#define ADC_CSR_EOC            (1 << 7)  /*!< End of Conversion Flag */
#define ADC_CSR_AWD            (1 << 6)  /*!< Analog Watchdog Flag */
#define ADC_CSR_EOCIE          (1 << 5)  /*!< EOC Interrupt Enable */
#define ADC_CSR_AWDIE          (1 << 4)  /*!< AWD Interrupt Enable */
#define ADC_CSR_CH3            (1 << 3)  /*!< Channel Selection Bit 3 */
#define ADC_CSR_CH2            (1 << 2)  /*!< Channel Selection Bit 2 */
#define ADC_CSR_CH1            (1 << 1)  /*!< Channel Selection Bit 1 */
#define ADC_CSR_CH0            (1 << 0)  /*!< Channel Selection Bit 0 */

/* Shift and Mask for Channel Selection */
#define ADC_CSR_CHANNEL_Shift  (0)
#define ADC_CSR_CHANNEL_Mask   (0x0f)

/* ADC CR1 Bit Definitions */
#define ADC_CR1_SPSEL2         (1 << 6)  /*!< Prescaler Select Bit 2 */
#define ADC_CR1_SPSEL1         (1 << 5)  /*!< Prescaler Select Bit 1 */
#define ADC_CR1_SPSEL0         (1 << 4)  /*!< Prescaler Select Bit 0 */
#define ADC_CR1_CONT           (1 << 1)  /*!< Continuous Conversion  */
#define ADC_CR1_ADON           (1 << 0)  /*!< ADC Enable  */

/* Shift and Mask for Prescaler Selection */
#define ADC_CR1_SPSEL_Shift    (4)
#define ADC_CR1_SPSEL_Mask     (0x07 << 4)

/* ADC CR2 Bit Definitions */
#define ADC_CR2_EXTTRIG        (1 << 6)  /*!< External Trigger Enable */
#define ADC_CR2_EXTSEL1        (1 << 5)  /*!< External Event Select Bit 1 */
#define ADC_CR2_EXTSEL0        (1 << 4)  /*!< External Event Select Bit 0 */
#define ADC_CR2_ALIGN          (1 << 3)  /*!< Data Align (0=Left, 1=Right) */
#define ADC_CR2_SCAN           (1 << 1)  /*!< Scan Mode Enable */

/* ADC CR3 Bit Definitions */
#define ADC_CR3_DBUF           (1 << 7)  /*!< Data Buffer Enable */
#define ADC_CR3_OVR            (1 << 6)  /*!< Overrun Flag */

/* ADC TDRH Bit Definitions */
#define ADC_TDRH_TD15          (1 << 7)  /*!< Ch15 Schmitt Trigger Disable */
#define ADC_TDRH_TD14          (1 << 6)  /*!< Ch14 Schmitt Trigger Disable */
#define ADC_TDRH_TD13          (1 << 5)  /*!< Ch13 Schmitt Trigger Disable */
#define ADC_TDRH_TD12          (1 << 4)  /*!< Ch12 Schmitt Trigger Disable */
#define ADC_TDRH_TD11          (1 << 3)  /*!< Ch11 Schmitt Trigger Disable */
#define ADC_TDRH_TD10          (1 << 2)  /*!< Ch10 Schmitt Trigger Disable */
#define ADC_TDRH_TD9           (1 << 1)  /*!< Ch9 Schmitt Trigger Disable  */
#define ADC_TDRH_TD8           (1 << 1)  /*!< Ch8 Schmitt Trigger Disable  */

/* ADC TDRL Bit Definitions */
#define ADC_TDRL_TD7           (1 << 7)  /*!< Ch7 Schmitt Trigger Disable */
#define ADC_TDRL_TD6           (1 << 6)  /*!< Ch6 Schmitt Trigger Disable */
#define ADC_TDRL_TD5           (1 << 5)  /*!< Ch5 Schmitt Trigger Disable */
#define ADC_TDRL_TD4           (1 << 4)  /*!< Ch4 Schmitt Trigger Disable */
#define ADC_TDRL_TD3           (1 << 3)  /*!< Ch3 Schmitt Trigger Disable */
#define ADC_TDRL_TD2           (1 << 2)  /*!< Ch2 Schmitt Trigger Disable */
#define ADC_TDRL_TD1           (1 << 1)  /*!< Ch1 Schmitt Trigger Disable */
#define ADC_TDRL_TD0           (1 << 1)  /*!< Ch0 Schmitt Trigger Disable */

/**
  * @}
  */

/** @defgroup IWDG_Register_Bit_Definitions IWDG Register Bit Definitions
  * @{
  */

/* IWDG Key Register Definitions */
#define IWDG_KR_KEY_ENABLE     (0xcc)     /*!< Key to enable the IWDG         */
#define IWDG_KR_KEY_REFRESH    (0xaa)     /*!< Key to refresh the IWDG        */
#define IWDG_KR_KEY_ACCESS     (0x55)     /*!< Key to Enable access to PR/RLR */


/* IWDG Prescaler Register Bit Definitions */
#define IWDG_PR_Shift          (0)
#define IWDG_PR_MASK           (0x07)

#define IWDG_PR_DIV4           (0x00)
#define IWDG_PR_DIV8           (0x01)
#define IWDG_PR_DIV16          (0x02)
#define IWDG_PR_DIV32          (0x03)
#define IWDG_PR_DIV64          (0x04)
#define IWDG_PR_DIV128         (0x05)
#define IWDG_PR_DIV256         (0x06)

/**
  * @}
  */

/** @defgroup WWDG_Register_Bit_Definitions WWDG Register Bit Definitions
  * @{
  */

/* WWDG Control Register Definitions */
#define WWDG_CR_WDGA           (1 << 7)   /*!< WWDG Activation Bit */
#define WWDG_CR_T_Shift        (0)        /*!< Shift for 7 bit counter value */
#define WWDG_KR_T_Mask         (0x7f)     /*!< Mask for 7 bit counter value */

/**
  * @}
  */

/** @defgroup CFG_Register_Bit_Definitions Config Register Bit Definitions
  * @{
  */

/* CFG Register Bit Definitions */
#define CFG_GCR_AL              (1 << 0)  /*!< Activn Lvl (0=norm,1=irq-only) */
#define CFG_GCR_SWD             (1 << 1)  /*!< SWIM Disable */

/**
  * @}
  */

/** @defgroup ASM_Definitions Special Assembly Definitions
  * @{
  */

#if defined(_SDCC_)
# define __rim()                {__asm__("rim\n");}  /*!< Enable interrupts  */
# define __sim()                {__asm__("sim\n");}  /*!< Disable interrupts */
# define __nop()                {__asm__("nop\n");}  /*!< No Operation       */
# define __trap()               {__asm__("trap\n");} /*!< Trap Instruction   */
# define __wfi()                {__asm__("wfi\n");}  /*!< Wait for Interrupt */
# define __halt()               {__asm__("halt\n");} /*!< Halt */
#endif

#define enableInterrupts()      __rim()
#define disableInterrupts()     __sim()

/**
  * @}
  */

/* MCU-Specific Includes */
#if defined(STM8S003) || defined(STM8S103)
#include "stm8s003.h"
#endif


#endif /* #ifndef STM8S_H_ ... */
