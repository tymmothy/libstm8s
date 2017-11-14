#ifndef STM8_GPIO_H_
#define STM8_GPIO_H_

/* Includes -----------------------------------------------------------------*/

#include "stm8s.h"


/* Static Inline Functions --------------------------------------------------*/

/** @brief Write data to all 8 output bits of a GPIO port.
  * @param gpio   Pointer to the GPIO port registers
  * @param values Value to write to the GPIO port.
  */
static inline void gpio_write_output(GPIO_TypeDef *gpio, int values)
{
    return gpio->ODR = pins;
}

/** @brief Read all 8 bits from the output data register of a GPIO port.
  * @param gpio   Pointer to the GPIO port registers
  * @return Byte containing the value from the output data register.
  */
static inline uint8_t gpio_read_output(GPIO_TypeDef *gpio)
{
    return gpio->ODR;
}

/** @brief Write data to a masked set of output bits of a GPIO port.
  * @param gpio   Pointer to the GPIO port registers
  * @param pins   Bitmask of pins whose values will be updated
  * @param values Bitmask of values to apply to the specified pins.
  */
static inline void gpio_write_output_pins(GPIO_TypeDef *gpio, uint8_t pins,
                                          uint8_t values)
{
    gpio->ODR = (gpio->ODR & pins) | values;
}

/** @brief Get the current values of a masked set of bits
  * @param gpio   Pointer to the GPIO port registers
  * @return Byte containing the value from the output data register.
  */
static inline uint8_t gpio_read_output_pins(GPIO_TypeDef *gpio, int pins)
{
    return (gpio->ODR & pins);
}

static inline uint8_t gpio_read_input_port(GPIO_TypeDef *gpio)
{
    return gpio->IDR;
}

static inline uint8_t gpio_read_input_pins(GPIO_TypeDef *gpio, int pins)
{
    return (gpio->IDR & pins);
}

static inline void gpio_set_port_directions(GPIO_TypeDef *gpio, uint8_t dir)
{
    gpio->DDR = dir;
}

static inline uint8_t gpio_get_port_directions(GPIO_TypeDef *gpio)
{
    return gpio->DDR;
}

static inline void gpio_set_pins_directions(GPIO_TypeDef *gpio, uint8_t pins,
                                            uint8_t dirs)
{
    gpio->DDR = (gpio->DDR & ~pins) | dirs;
}

static inline uint8_t gpio_get_pins_directions(GPIO_TypeDef *gpio, uint8_t pins)
{
    return gpio->DDR & pins;
}

static inline void gpio_configure(GPIO_TypeDef *gpio, uint8_t pins,
                                  uint8_t mode)
{
    if (mode & GPIO_MODE_OUT) {
        gpio->DDR |= pins;
    } else {
        gpio->DDR &= ~pins;
    }

    /* Select Pullup Enable (input) or Push-Pull (output) */
    if (mode & (1 << 1)) {
        gpio->CR1 |= pins;
    } else {
        gpio->CR1 &= ~pins;
    }

    /* Select External Interrupt Enable (input) or 10MHz (output) */
    if (mode & (1 << 2)) {
        gpio->CR2 |= pins;
    } else {
        gpio->CR2 &= ~pins;
    }
}

static inline uint8_t gpio_get_pin_configuration(GPIO_TypeDef *gpio,
                                                 uint8_t pin)
{
    uint8_t mode = 0;

    if (gpio->DDR & pin) {
        mode |= GPIO_MODE_OUT;
    }

    if (gpio->CR1 & pin) {
        mode |= (1 << 1);
    }

    if (gpio->CR2 & pin) {
        mode |= (1 << 2);
    }
}

#endif /* #ifndef STM8_GPIO_H_ ... */