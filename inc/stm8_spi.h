#ifndef STM8S_SPI_H_
#define STM8S_SPI_H_

/* Includes -----------------------------------------------------------------*/

#include <stdint.h>

#include "stm8s.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Defines ------------------------------------------------------------------*/

#define SPI_FORMAT_LSBFIRST             (SPI_CR1_LSBFIRST)
#define SPI_FORMAT_MSBFIRST             (0)
#define SPI_ROLE_MASTER                 (SPI_CR1_MSTR)
#define SPI_ROLE_SLAVE                  (0)

#define SPI_BAUDDIV_2                   (0 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_4                   (1 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_8                   (2 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_16                  (3 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_32                  (4 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_64                  (5 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_128                 (6 << SPI_CR1_Baud_Shift)
#define SPI_BAUDDIV_256                 (7 << SPI_CR1_Baud_Shift)


/* Static Inline Functions --------------------------------------------------*/

static inline void spi_configure(SPI_TypeDef *spi, uint8_t role)
{
    SPI->CR1 = role;
    SPI->CR2 = 0;
}

static inline void spi_set_mode(SPI_TypeDef *spi, uint8_t mode)
{
    switch(mode) {
        case 0:
            SPI->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CHA);
            break;
        case 1:
            SPI->CR1 = (SPI->CR1 & ~(SPI_CR1_CPOL)) | SPI_CR1_CPHA;
            break;
        case 2:
            SPI->CR1 = (SPI->CR1 & ~(SPI_CR1_CPHA)) | SPI_CR1_CPOL;
            break;
        case 3:
            SPI->CR1 |= (SPI_CR1_CPHA | SPI_CR1_CPOL);
            break;
        default:
            assert(0);
    }
}

static inline int spi_get_mode(SPI_TypeDef *spi, uint8_t mode)
{
    int mode;

    mode = (SPI->CR1 & SPI_CR1_CPHA) ? 1:0;
    mode |= (SPI->CR1 & SPI_CR1_COL) ? 2:0;
    
    return mode;
}

static inline void spi_set_divider(SPI_TypeDef *spi, uint8_t div)
{
    assert(div <= 7);

    SPI->CR1 = (SPI->CR1 & ~SPI_CR1_Baud_Mask) | (div << SPI_CR1_Baud_Shift);
}

static inline int spi_get_divider(SPI_TypeDef *spi, uint8_t div)
{
    return (SPI->CR1 & SPI_CR1_Baud_Mask) >> SPI_CR1_Baud_Shift;
}

/** @brief Enable SPI peripheral.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_enable(SPI_TypeDef *spi)
{
    spi->CR1 |= SPI_CR1_SPEN;
}

/** @brief Disable SPI peripheral.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_disable_crc(SPI_TypeDef *spi)
{
    spi->CR1 &= ~SPI_CR1_SPEN;
}

/** @brief Test whether SPI peripheral is enabled.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if SPI peripheral is enabled, 0 otherwise.
  */
static inline int spi_is_enabled(SPI_TypeDef *spi)
{
    return (spi->CR1 & SPI_CR1_SPEN) ? 1:0;
}

/** @brief Send a byte via SPI peripheral.
  * @param spi    Pointer to SPI peripheral
  * @param b      Byte to send
  */
static inline void spi_send(SPI_TypeDef *spi, uint8_t b)
{
    spi->DR = b;
}

/** @brief Read a received byte from SPI peripheral.
  * @param spi    Pointer to SPI peripheral
  * @return Byte read from SPI peripheral.
  */
static inline uint8_t spi_send(SPI_TypeDef *spi, uint8_t b)
{
    return spi->DR;
}

/** @brief Enable SPI Receive Only mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_enable_rxonly(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_RXONLY;
}

/** @brief Disable SPI Receive Only mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_disable_rxonly(SPI_TypeDef *spi)
{
    spi->CR2 &= ~SPI_CR2_RXONLY;
}

/** @brief Test whether SPI Receive Only mode is enabled.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if Receive Only mode is enabled, 0 otherwise.
  */
static inline int spi_rxonly_is_enabled(SPI_TypeDef *spi)
{
    return (spi->CR2 & SPI_CR2_RXONLY) ? 1:0;
}

/** @brief Enable SPI hardware CRC calculation.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_enable_crc(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_CRCEN;
}

/** @brief Disable SPI hardware CRC calculation.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_disable_crc(SPI_TypeDef *spi)
{
    spi->CR2 &= ~SPI_CR2_CRCEN;
}

/** @brief Test whether SPI hardware CRC calculation is enabled.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if CRC calculation is enabled, 0 otherwise.
  */
static inline int spi_crc_is_enabled(SPI_TypeDef *spi)
{
    return (spi->CR2 & SPI_CR2_CRCEN) ? 1:0;
}

/** @brief Enable SPI bidirectional mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_enable_bidirectional_mode(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_BDM;
}

/** @brief Disable SPI bidirectional mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_disable_bidirectional_mode(SPI_TypeDef *spi)
{
    spi->CR2 &= ~SPI_CR2_BDM;
}

/** @brief Test whether SPI bidirectional mode is enabled.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if bidirectional mode is enabled, 0 otherwise.
  */
static inline int spi_bidirectional_mode_is_enabled(SPI_TypeDef *spi)
{
    return (spi->CR2 & SPI_CR2_BDM) ? 1:0;
}

/** @brief Set SPI bidirectional mode to input.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_bidirectional_mode_input(SPI_TypeDef *spi)
{
    spi->CR2 &= ~SPI_CR2_BDOE;
}

/** @brief Set SPI bidirectional mode to output.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_bidirectional_mode_output(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_BDOE;
}

/** @brief Test whether SPI bidirectional mode is set to output.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if bidirectional mode is set to output, 0 if input.
  */
static inline int spi_bidirectional_mode_is_output(SPI_TypeDef *spi)
{
    return (spi->CR2 & SPI_CR2_BDOE) ? 1:0;
}

/** @brief Enable SPI software slave mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_enable_software_slave_mode(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_SSM;
}

/** @brief Disable SPI software slave mode.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_disable_software_slave_mode(SPI_TypeDef *spi)
{
    spi->CR2 &= ~SPI_CR2_SSM;
}

/** @brief Test whether SPI software slave mode is enabled.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if software slave mode is enabled, 0 otherwise.
  */
static inline int spi_software_slave_mode_is_enabled(SPI_TypeDef *spi)
{
    return (spi->CR2 & SPI_CR2_SSM) ? 1:0;
}

/** @brief Request SPI send hardware CRC calculation byte.
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_send_crc(SPI_TypeDef *spi)
{
    spi->CR2 |= SPI_CR2_CRCNEXT;
}

/** @brief Enable SPI Tx Empty IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_tx_empty_irq_enable(SPI_TypeDef *spi)
{
    spi->ICR |= SPI_ICR_TXIE;
}

/** @brief Disable SPI Tx Empty IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_tx_empty_irq_disable(SPI_TypeDef *spi)
{
    spi->ICR &= ~SPI_ICR_TXIE;
}

/** @brief Test whether SPI Tx Empty IRQ is enabled
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the IRQ is enabled, 0 if disabled.
  */
static inline int spi_tx_empty_irq_is_enabled(SPI_TypeDef *spi)
{
    (spi->ICR & SPI_ICR_TXIE) ? 1:0;
}

/** @brief Enable SPI Rx Data Available IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_rx_available_irq_enable(SPI_TypeDef *spi)
{
    spi->ICR |= SPI_ICR_RXIE;
}

/** @brief Disable SPI Rx Data Available IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_rx_available_irq_disable(SPI_TypeDef *spi)
{
    spi->ICR &= ~SPI_ICR_RXIE;
}

/** @brief Test whether SPI Rx Data Available IRQ is enabled
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the IRQ is enabled, 0 if disabled.
  */
static inline int spi_rx_available_irq_is_enabled(SPI_TypeDef *spi)
{
    (spi->ICR & SPI_ICR_RXIE) ? 1:0;
}

/** @brief Enable SPI Error IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_error_irq_enable(SPI_TypeDef *spi)
{
    spi->ICR |= SPI_ICR_ERRIE;
}

/** @brief Disable SPI Error IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_error_irq_disable(SPI_TypeDef *spi)
{
    spi->ICR &= ~SPI_ICR_ERRIE;
}

/** @brief Test whether SPI Error IRQ is enabled
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the IRQ is enabled, 0 if disabled.
  */
static inline int spi_error_irq_is_enabled(SPI_TypeDef *spi)
{
    (spi->ICR & SPI_ICR_ERRIE) ? 1:0;
}

/** @brief Enable SPI Wakeup IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_wakeup_irq_enable(SPI_TypeDef *spi)
{
    spi->ICR |= SPI_ICR_WKIE;
}

/** @brief Disable SPI Wakeup IRQ
  * @param spi    Pointer to SPI peripheral
  */
static inline void spi_wakeup_irq_disable(SPI_TypeDef *spi)
{
    spi->ICR &= ~SPI_ICR_WKIE;
}

/** @brief Test whether SPI Wakeup IRQ is enabled
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the IRQ is enabled, 0 if disabled.
  */
static inline int spi_wakeup_irq_is_enabled(SPI_TypeDef *spi)
{
    (spi->ICR & SPI_ICR_WKIE) ? 1:0;
}

/** @brief Test whether an SPI peripheral is currently transferring data.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the peripheral is currently transferring data, 0 otherwise.
  */
static inline int spi_busy(SPI_TypeDef *spi)
{
    return (spi->SR & SPI_SR_BSY) ? 1:0;
}

/** @brief Test whether an SPI peripheral has a pending error.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the peripheral has a pending error, 0 otherwise.
  */
static inline int spi_error(SPI_TypeDef *spi)
{
    return (spi->SR & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR));
}

/** @brief Test whether an SPI peripheral's Tx buffer is empty.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the peripheral's Tx buffer is empty, 0 otherwise.
  */
static inline int spi_tx_empty(SPI_TypeDef *spi)
{
    return (spi->SR & SPI_SR_TXE) ? 1:0;
}

/** @brief Test whether an SPI peripheral has received data available.
  * @param spi    Pointer to SPI peripheral
  * @return 1 if the peripheral has received data available, 0 otherwise.
  */
static inline int spi_rx_available(SPI_TypeDef *spi)
{
    return (spi->SR & SPI_SR_RXNE) ? 1:0;
}

#ifdef __cplusplus
};
#endif

#endif /* #ifndef STM8S_SPI_H_ ... */
