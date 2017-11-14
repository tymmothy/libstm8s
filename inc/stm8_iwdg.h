/******************************************************************************
 * @file:    stm8_tim16.h
 * @purpose: Header File for 16 bit timer peripheral on STM8 microcontrollers
 * @version: V1.0
 * @author:  Tymm Zerr
 * @date:    1. September 2015
 ******************************************************************************
 * @section License License
 * Licensed under a Simplified BSD License:
 *
 * Copyright (c) 2017, Tymm Zerr
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
 * TIMOTHY ZERR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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

#ifndef STM8S_IWDG_H_
#define STM8S_IWDG_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "stm8s.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enums --------------------------------------------------------------------*/

typedef enum {
    IWDG_PRESCALER_4   = 0,
    IWDG_PRESCALER_8   = 1,
    IWDG_PRESCALER_16  = 2,
    IWDG_PRESCALER_32  = 3,
    IWDG_PRESCALER_64  = 4,
    IWDG_PRESCALER_128 = 5,
    IWDG_PRESCALER_256 = 6,
} iwdg_prescaler_e;


/* Static Inline Functions --------------------------------------------------*/

/**
 * @brief Enable the independent watchdog
 *
 * @param iwdg      Pointer to the IWDG peripheral structure
 *
 * @note After enabling, the independent watchdog can only be disabled by reset
 */
void iwdg_enable(IWDG_TypeDef *iwdg)
{
    iwdg->KR = IWDG_KR_KEY_ENABLE;
}

/**
 * @brief Refresh the independent watchdog
 *
 * @param iwdg      Pointer to the IWDG peripheral structure
 *
 * @details
 * After the independent watchdog is enabled, it must be refreshed before the
 * timer runs out, or the MCU will reset.
 */
void iwdg_refresh(IWDG_TypeDef *iwdg)
{
    iwdg->KR = IWDG_KR_KEY_REFRESH;
}

/**
 * @brief Set the independent watchdog's reload value
 *
 * @param iwdg       Pointer to the IWDG peripheral structure
 * @param reload_val Value to reload into the IWDG's counter when refreshed
 */
void iwdg_set_reload(IWDG_TypeDef *iwdg, uint8_t reload_val)
{
    iwdg->KR = IWDG_KR_KEY_ACCESS;
    iwdg->RR = reload_val;
}

/**
 * @brief Get the independent watchdog's reload value
 *
 * @param iwdg       Pointer to the IWDG peripheral structure
 *
 * @return Value in the iwdg's reload register
 */
uint8_t iwdg_get_reload(IWDG_TypeDef *iwdg) {
    return iwdg->RR;
}

/**
 * @brief Set the independent watchdog's prescaler
 *
 * @param iwdg       Pointer to the IWDG peripheral structure
 * @param pre        Prescaler setting (iwdg_prescaler_e)
 */
void iwdg_set_prescaler(IWDG_TypeDef *iwdg, iwdg_prescaler_e pre)
{
    iwdg->KR = IWDG_KR_KEY_ACCESS;
    iwdg->PR = pre;
}

/**
 * @brief Get the independent watchdog's prescaler value
 *
 * @param iwdg       Pointer to the IWDG peripheral structure
 *
 * @return Value in the iwdg's prescaler register (iwdg_prescaler_e)
 */
iwdg_prescaler_e iwdg_get_prescaler(IWDG_TypeDef *iwdg) {
    return iwdg->PR;
}

#ifdef __cplusplus
};
#endif

#endif /* #ifndef STM8S_IWDG_H_ ... */
