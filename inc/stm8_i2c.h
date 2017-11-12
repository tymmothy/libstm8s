/******************************************************************************
 * @file:    stm8_i2c.h
 * @purpose: Header File for I2C peripheral on STM8 microcontrollers
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

#ifndef STM8S_I2C_H_
#define STM8S_I2C_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "stm8s.h"


#ifdef __cplusplus
extern "C" {
#endif


/* Defines ------------------------------------------------------------------*/

/* Static Inline Functions --------------------------------------------------*/

/**
 * @brief Enable clock stretching on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_enable_clock_stretching(I2C_TypeDef *i2c)
{
    i2c->CR1 &= ~I2C_CR1_NOSTRETCH;
}

/**
 * @brief Disable clock stretching on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_disable_clock_stretching(I2C_TypeDef *i2c)
{
    i2c->CR1 |= I2C_CR1_NOSTRETCH;
}

/**
 * @brief Test whether clock stretching is enabled on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 *
 * @return 1 if enabled, 0 otherwise
 */
static inline int i2c_clock_stretching_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR1 & I2C_CR1_NOSTRETCH) ? 0:1;
}

/**
 * @brief Enable reception of general call messages on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_enable_general_call(I2C_TypeDef *i2c)
{
    i2c->CR1 |= I2C_CR1_ENGC;
}

/**
 * @brief Disable reception of general call messages on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_disable_general_call(I2C_TypeDef *i2c)
{
    i2c->CR1 &= ~I2C_CR1_ENGC;
}

/**
 * @brief Test whether general call reception is enabled on an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 *
 * @return 1 if enabled, 0 otherwise
 */
static inline int i2c_general_call_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR1 & I2C_CR1_ENGC) ? 1:0;
}

/**
 * @brief Enable an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_enable(I2C_TypeDef *i2c)
{
    i2c->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Disable an I2C peripheral
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_disable(I2C_TypeDef *i2c)
{
    i2c->CR1 &= ~I2C_CR1_PI;
}

/**
 * @brief Test whether an I2C peripheral is enabled
 *
 * @param i2c  Pointer to the I2C peripheral structure
 *
 * @return 1 if enabled, 0 otherwise
 */
static inline int i2c_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR1 & I2C_CR1_PE) ? 1:0;
}

/**
 * @brief Set an I2C peripheral into reset state
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_enable_reset(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_SWRST;
}

/**
 * @brief Remove an I2C peripheral from reset state
 *
 * @param i2c  Pointer to the I2C peripheral structure
 */
static inline void i2c_disable_reset(I2C_TypeDef *i2c)
{
    i2c->CR2 &= ~I2C_CR2_SWRST;
}

/**
 * @brief Test whether an I2C peripheral is currently in reset state
 *
 * @param i2c  Pointer to the I2C peripheral structure
 *
 * @return 1 if in reset, 0 otherwise
 */
static inline int i2c_reset_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR2 & I2C_CR2_SWRST) ? 1:0;
}

static inline void i2c_enable_pos(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_POS;
}

static inline void i2c_disable_pos(I2C_TypeDef *i2c)
{
    i2c->CR2 &= ~I2C_CR2_POS;
}

static inline int i2c_pos_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR2 & I2C_CR2_POS) ? 1:0;
}

static inline void i2c_enable_ack(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_ACK;
}

static inline void i2c_disable_ack(I2C_TypeDef *i2c)
{
    i2c->CR2 &= ~I2C_CR2_ACK;
}

static inline int i2c_ack_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR2 & I2C_CR2_ACK) ? 1:0;
}

static inline void i2c_enable_stop(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_STOP;
}

static inline void i2c_disable_stop(I2C_TypeDef *i2c)
{
    i2c->CR2 &= ~I2C_CR2_STOP;
}

static inline int i2c_stop_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR2 & I2C_CR2_STOP) ? 1:0;
}

static inline void i2c_enable_start(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_START;
}

static inline void i2c_disable_start(I2C_TypeDef *i2c)
{
    i2c->CR2 &= ~I2C_CR2_START;
}

static inline int i2c_start_is_enabled(const I2C_TypeDef *i2c)
{
    return (i2c->CR2 & I2C_CR2_START) ? 1:0;
}

/**
 * @brief Set the frequency, in MHz, 
static inline void i2c_set_frequency(I2C_TypeDef *i2c, uint8_t freq_mhz)
{
    assert(freq_mhz > 0);
    assert(freq_mhz <= 24);
    
    i2c->FREQ = freq_mhz;
}

static inline uint8_t i2c_get_frequency(const I2C_TypeDef *i2c)
{
    return i2c->FREQ;
}

/**
 * @brief Set the address the I2C peripheral will answer to as 7-bit value
 *
 * @param i2c  Pointer to the I2C peripheral structure
 * @param addr New 7-bit I2C address
 */
static inline void i2c_set_addr7(I2C_TypeDef *i2c, uint8_t addr)
{
    assert(addr < (1 << 7));
    
    i2c->OARL = (addr << 1);    
    i2c->OARL = I2C_OARH_ADDCONF;
}

/**
 * @brief Get the address of the I2C peripheral when in 7-bit address mode
 *
 * @param i2c  Pointer to the I2C peripheral structure
 *
 * @return 7-bit I2C address
 */
static inline uint8_t i2c_get_addr7(const I2C_TypeDef *i2c)
{
    return (i2c->OARL >> 1);
}

/**
 * @brief Set the address the I2C peripheral will answer to as 10-bit value
 *
 * @param i2c  Pointer to the I2C peripheral structure
 * @param addr New 10-bit I2C address
 */
static inline void i2c_set_addr10(I2C_TypeDef *i2c, uint16_t addr)
{
    assert(addr < (1 << 10));
    
    i2c->OARL = (addr & 0x7f);
    i2c->OARH = ((addr >> 7) & 0x03) | I2C_OARH_ADDMODE | I2C_OARH_ADDCONF;
}

static inline uint16_t i2c_get_addr10(const I2C_TypeDef *i2c)
{
    return ((i2c->OARH & I2C_OARH_10BITADDR_B8_9_Mask) << 7) | (i2c->OARL);
}

/**
 * @brief Load the next byte to send into an I2C peripheral
 *
 * @param i2c    Pointer to the I2C peripheral structure
 * @param data   Data byte to send
 */
static inline void i2c_send(I2C_TypeDef *i2c, uint8_t data)
{
    i2c->DR = data;
}

/**
 * @brief Retrieve the next byte that was received by an I2C peripheral
 *
 * @param i2c    Pointer to the I2C peripheral structure
 *
 * @return Data byte received by the I2C peripheral
 */
static inline uint8_t i2c_recv(I2C_TypeDef *i2c)
{
    return i2c->DR;
}


#ifdef __cplusplus
};
#endif

#endif /* #ifndef STM8S_SPI_H_ */
