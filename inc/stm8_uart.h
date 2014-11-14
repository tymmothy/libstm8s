#ifndef STM8_UART_H_
#define STM8_UART_H_

static inline void uart_set_brr(UART_TypeDef *uart, long div)
{
    uart->BRR2 = (div & 0xff00) >> 8;
    uart->BRR1 = (div & 0xff);
}

static inline long uart_get_brr(UART_TypeDef *uart)
{
    div = uart->BRR2 << 8;
    div |= uart->BRR1;

    return div;
}

static int uart_get_rx_databit8(UART_TypeDef *uart)
{
    return (uart->CR1 & UART_CR1_D8) ? 1:0;
}

static void uart_set_tx_databit8(UART_TypeDef *uart, int b8)
{
    if (b8) {
        uart->CR1 |= UART_CR1_R8;
    } else {
        uart->CR1 &= ~UART_CR1_R8;
    }
}

static void uart_disable(UART_TypeDef *uart)
{
    uart->CR1 |= UART_CR1_UARTD;
}

static void uart_enable(UART_TypeDef *uart)
{
    uart->CR1 &= ~UART_CR1_UARTD;
}

static int uart_is_disabled(UART_TypeDef *uart)
{
    return (uart->CR1 & UART_CR1_UARTD) ? 1:0;
}

static void uart_set_wordlen_8(UART_TypeDef *uart)
{
    uart->CR1 &= ~UART_CR1_M;
}

static void uart_set_wordlen_9(UART_TypeDef *uart)
{
    uart->CR1 |= UART_CR1_M;
}

static void uart_get_wordlen(UART_TypeDef *uart)
{
    if (uart->CR1 & UART_CR1_M) {
        return 9;
    }

    return 8;
}

static void uart_set_parity_even(UART_TypeDef *uart)
{
    uart->CR1 &= ~UART_CR1_PS;
}

static void uart_set_parity_odd(UART_TypeDef *uart)
{
    uart->CR1 |= UART_CR1_PS;
}

static int uart_parity_is_odd(UART_TypeDef *uart)
{
    return (uart->CR1 & UART_CR1_PS) ? 1:0;
}

static void uart_enable_parity(UART_TypeDef *uart)
{
    uart->CR1 |= UART_CR1_PCEN;
}

static void uart_disable_parity(UART_TypeDef *uart)
{
    uart->CR1 &= ~UART_CR1_PCEN;
}

static int uart_parity_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR1 & UART_CR1_PCEN) ? 1:0;
}

static inline void uart_enable_tx_interrupt(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_TIEN;
}

static inline void uart_disable_tx_interrupt(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_TIEN;
}

static inline int uart_tx_interrupt_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_TIEN) ? 1:0;
}

static inline void uart_enable_tx_complete_interrupt(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_TCIEN;
}

static inline void uart_disable_tx_complete_interrupt(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_TCIEN;
}

static inline int uart_tx_complete_interrupt_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_TCIEN) ? 1:0;
}

static inline void uart_enable_rx(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_REN;
}

static inline void uart_disable_rx(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_REN;
}

static inline void uart_enable_rx_interrupt(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_RIEN;
}

static inline void uart_disable_rx_interrupt(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_RIEN;
}

static inline void uart_enable_tx(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_TEN;
}

static inline void uart_disable_tx(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_TEN;
}

static inline int uart_tx_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_TEN) ? 1:0;
}

static inline int uart_rx_interrupt_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_RIEN) ? 1:0;
}

static inline void uart_enable_idle_interrupt(UART_TypeDef *uart)
{
    uart->CR2 |= UART1_CR2_ILIEN;
}

static inline void uart_disable_idle_interrupt(UART_TypeDef *uart)
{
    uart->CR2 &= ~UART1_CR2_ILIEN;
}

static inline int uart_idle_interrupt_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_ILIEN) ? 1:0;
}

static inline int uart_rx_is_enabled(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_REN) ? 1:0;
}

static inline void uart_send_break(UART_TypeDef *uart)
{
    uart->CR2 |= UART_CR2_SBK;
}

static inline int uart_break_is_sending(UART_TypeDef *uart)
{
    return (uart->CR2 & UART1_CR2_SBK) ? 1:0;
}


static inline void uart_set_smartcard_guard_time(UART_TypeDef *uart, int baudclocks)
{
    uart->GTR = baudclocks;
}

static inline int uart_get_smartcard_guard_time(UART_TypeDef *uart)
{
    return uart->GTR;
}

#endif /* #ifndef STM8_UART_H_ ... */