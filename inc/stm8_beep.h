/******************************************************************************
 * @file:    stm8_beep.h
 * @purpose: Header File for beeper peripheral on STM8 microcontrollers
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
 * TIMOTHY TWILLMAN OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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

#ifndef STM8S_BEEP_H_
#define STM8S_BEEP_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "stm8s.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Defines ------------------------------------------------------------------*/

#define BEEP_TONE_LOW      (0)
#define BEEP_TONE_MED      (1)
#define BEEP_TONE_HIGH     (2)


/* Static Inline Functions --------------------------------------------------*/

/** @brief Set the beeper tone type
  * @param  beep     Beeper peripheral
  * @param  tone     Tone type.  One of:
  *                 BEEP_TONE_LOW, BEEP_TONE_MED, BEEP_TONE_HIGH
  */
static inline void beep_set_tone(Beep_TypeDef *beep, uint8_t tone)
{
    assert(tone <= BEEP_TONE_HIGH);

    beep->CSR = (beep->CSR & ~BEEP_Select_Mask) | (tone << BEEP_Select_Shift);
}

/** @brief Get the currently set beeper tone type
  * @param  beep      Beeper peripheral
  * @return BEEP_TONE_LOW, BEEP_TONE_MED, BEEP_TONE_HIGH, BEEP_TONE_HIGH2
  */
static inline uint8_t beep_get_tone(Beep_TypeDef *beep)
{
    uint8_t ret = (beep->CSR & BEEP_Select_Mask) >> BEEP_Select_Shift;


    if (ret > BEEP_TONE_HIGH) {
        ret = BEEP_TONE_HIGH;
    }

    return ret;
}

/** @brief Enable the beeper
  * @param  beep      Beeper peripheral
  */
static inline void beep_enable(Beep_TypeDef *beep)
{
    beep->CSR |= BEEP_CSR_EN;
}

/** @brief Disable the beeper
  * @param  beep      Beeper peripheral
  */
static inline void beep_disable(Beep_TypeDef *beep)
{
    beep->CSR &= ~BEEP_CSR_EN;
}

/** @brief Test whether the beeper is enabled
  * @param  beep      Beeper peripheral
  * @return 1 if the beeper is enabled, 0 if not.
  */
static inline int beep_is_enabled(Beep_TypeDef *beep)
{
    return (beep->CSR & BEEP_CSR_EN) ? 1:0;
}

/** @brief Set the beeper prescaler
  * @param  beep      Beeper peripheral
  * @param  prescaler New beeper prescaler value (2-32).
  */
static inline void beep_set_prescaler(Beep_TypeDef *beep, uint8_t prescaler)
{
    assert(pre >= 2);
    assert (pre <= 32);

    pre -= 2;
    beep->CSR = (beep->CSR & ~BEEP_Divider_Mask) | (div & BEEP_Divider_Mask);
}

/** @brief Get the beeper prescaler
  * @param  beep      Beeper peripheral
  * @return Current beeper prescaler value (2-32), or 33 if not set.
  */
static inline uint8_t beep_get_prescaler(Beep_TypeDef *beep)
{
    return (beep->CSR & BEEP_Divider_Mask) + 2;
}



#ifdef __cplusplus
};
#endif

#endif /* #ifndef STM8S_SPI_H_ ... */
