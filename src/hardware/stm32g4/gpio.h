#pragma once

#include <stdint.h>
#include "stm32g4xx.h"

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,

} gpio_port_t;

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTERNATE = 2,
    GPIO_MODE_ANALOG = 3,
} gpio_mode_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM = 1,
    GPIO_SPEED_HIGH = 2,
    GPIO_SPEED_VERY_HIGH = 3,
} gpio_speed_t;

typedef enum {
    GPIO_TYPE_PUSH_PULL = 0,
    GPIO_TYPE_OPEN_DRAIN = 1,
} gpio_type_t;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP = 1,
    GPIO_PULL_DOWN = 2,
} gpio_pull_t;

typedef enum {
    GPIO_AF0 = 0,
    GPIO_AF1 = 1,
    GPIO_AF2 = 2,
    GPIO_AF3 = 3,
    GPIO_AF4 = 4,
    GPIO_AF5 = 5,
    GPIO_AF6 = 6,
    GPIO_AF7 = 7,
    GPIO_AF8 = 8,
    GPIO_AF9 = 9,
    GPIO_AF10 = 10,
    GPIO_AF11 = 11,
    GPIO_AF12 = 12,
    GPIO_AF13 = 13,
    GPIO_AF14 = 14,
    GPIO_AF15 = 15,
} gpio_af_t;

typedef enum {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1 = 1,
    GPIO_PIN_2 = 2,
    GPIO_PIN_3 = 3,
    GPIO_PIN_4 = 4,
    GPIO_PIN_5 = 5,
    GPIO_PIN_6 = 6,
    GPIO_PIN_7 = 7,
    GPIO_PIN_8 = 8,
    GPIO_PIN_9 = 9,
    GPIO_PIN_10 = 10,
    GPIO_PIN_11 = 11,
    GPIO_PIN_12 = 12,
    GPIO_PIN_13 = 13,
    GPIO_PIN_14 = 14,
    GPIO_PIN_15 = 15,
} gpio_pin_t;

typedef struct {
    gpio_port_t port;
    gpio_pin_t pin;
    gpio_mode_t mode;
    gpio_speed_t speed;
    gpio_type_t type;
    gpio_pull_t pull;
    gpio_af_t af;

} gpio_t;

void gpio_pin_init(gpio_t *pin);
void gpio_pin_deinit(gpio_t *pin);
inline void gpio_pin_set(gpio_t *pin);
inline void gpio_pin_clear(gpio_t *pin);
inline void gpio_pin_toggle(gpio_t *pin);
inline uint16_t gpio_pin_read(gpio_t *pin);
inline void gpio_pin_write(gpio_t *pin, uint16_t value);

//------------------------------------------------------
// Inline Functions
//------------------------------------------------------

inline void
gpio_pin_clear(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->BSRR |= (1 << (pin->pin + 16));

}

inline void
gpio_pin_set(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->BSRR |= (1 << pin->pin);

}

inline uint32_t
gpio_pin_get(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    return (gpio->IDR & (1 << pin->pin)) ? 1 : 0;

}

inline void
gpio_pin_toggle(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->ODR ^= (1 << pin->pin);

}


inline void
gpio_pin_write(gpio_t *pin, uint16_t value) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->ODR = value;

}

inline uint16_t
gpio_pin_read(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    return gpio->IDR;

}
