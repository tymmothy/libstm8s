/******************************************************************************
 * @file:    stm8_adc.h
 * @purpose: Header File for ADC peripheral on STM8 microcontrollers
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

#ifndef STM8S_ADC_H_
#define STM8S_ADC_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "stm8s.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Static Inline Functions --------------------------------------------------*/


static inline int adc_conversion_complete(ADC_TypeDef *adc)
{
    return (adc->CSR & ADC_CSR_EOC) ? 1:0;
}

static inline int adc_analog_watchdog_triggered(ADC_TypeDef *adc)
{
    return (adc->CSR & ADC_CSR_AWD);
}

static inline void adc_conversion_complete_irq_enable(ADC_TypeDef *adc)
{
    adc->CSR |= ADC_CSR_EOCIE;
}

static inline void adc_conversion_complete_irq_disable(ADC_TypeDef *adc)
{
    adc->CSR &= ~ADC_CSR_EOCIE;
}

static inline int adc_conversion_complete_irq_is_enabled(ADC_TypeDef *adc)
{
    return (adc->CSR & ADC_CSR_EOCIE);
}

static inline void adc_analog_watchdog_irq_enable(ADC_TypeDef *adc)
{
    adc->CSR |= ADC_CSR_AWDIE;
}

static inline void adc_analog_watchdog_irq_disable(ADC_TypeDef *adc)
{
    adc->CSR &= ~ADC_CSR_AWDIE;
}

static inline int adc_analog_watchdog_irq_is_enabled(ADC_TypeDef *adc)
{
    return (adc->CSR & ADC_CSR_AWDIE);
}

static inline void adc_set_channel(ADC_TypeDef *adc, uint8_t channel)
{
    adc->CSR = (adc->CSR & ~ADC_CSR_CHANNEL_Mask)
             | (channel & ADC_CSR_CHANNEL_Mask);
}

static inline uint8_t adc_get_channel(ADC_TypeDef *adc)
{
    return adc->CSR & ADC_CSR_CHANNEL_Mask;
}

static inline void adc_set_prescaler(ADC_TypeDef *adc, uint8_t prescaler)
{
    adc->CR1 = (adc->CR1 & ~ADC_CR1_SPSEL_Mask)
             | (prescaler & ADC_CR1_SPSEL_Mask);
}

static inline uint8_t adc_get_prescaler(ADC_TypeDef *adc)
{
    return adc->CR1 & ADC_CR1_SPSEL_Mask;
}

static inline void adc_enable_continuous_conversion(ADC_TypeDef *adc)
{
    adc->CR1 |= ADC_CR1_CONT;
}

static inline void adc_disable_continuous_conversion(ADC_TypeDef *adc)
{
    adc->CR1 &= ~ADC_CR1_CONT;
}

static inline int adc_continuous_conversion_is_enabled(ADC_TypeDef *adc)
{
    return (adc->CR1 & ADC_CR1_CONT) ? 1:0;
}

static inline void adc_enable(ADC_TypeDef *adc)
{
    adc->CR1 |= ADC_CR1_ADON;
}

static inline void adc_disable(ADC_TypeDef *adc)
{
    adc->CR1 &= ~ADC_CR1_ADON;
}

static inline int adc_is_enabled(ADC_TypeDef *adc)
{
    return (adc->CR1 & ADC_CR1_ADON) ? 1:0;
}

static inline void adc_enable_external_trigger(ADC_TypeDef *adc)
{
    adc->CR2 |= ADC_CR2_EXTTRIG;
}

static inline void adc_disable_external_trigger(ADC_TypeDef *adc)
{
    adc->CR2 &= ~ADC_CR2_EXTTRIG;
}

static inline int adc_external_trigger_is_enabled(ADC_TypeDef *adc)
{
    return (adc->CR2 & ADC_CR2_EXTTRIG) ? 1:0;
}

static inline uint16_t adc_read(ADC_TypeDef *adc)
{
    uint16_t rdg;


    rdg = (adc->DRH) << 8;
    rdg |= adc->DRL;

    return rdg;
}

static inline void adc_enable_schmitt_triggers(ADC_TypeDef *adc, uint16_t mask)
{
    adc->TDRH |= (mask >> 8);
    adc->TDRL |= (mask & 0xff);
}

static inline void adc_disable_schmitt_triggers(ADC_TypeDef *adc, uint16_t mask)
{
    adc->TDRH &= ~(mask >> 8);
    adc->TDRL &= ~(mask & 0xff);
}

static inline uint16_t adc_enabled_schmitt_triggers(ADC_TypeDef *adc)
{
    uint16_t trig;


    trig = (adc->TDRH) << 8;
    trig |= adc->TDRH;

    return trig;
}


#ifdef __cplusplus
};
#endif

#endif /* #ifndef STM8S_ADC_H_ ... */

