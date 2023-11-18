#include "hardware/stm32g4/gpio.h"


void 
gpio_pin_init(gpio_t *pin) {

    // Enable GPIO clock
    RCC->AHB2ENR |= (1 << pin->port);
    
    // Set GPIO mode
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->MODER &= ~(0x3 << (pin->pin * 2));
    gpio->MODER |= (pin->mode << (pin->pin * 2));
    
    // Set GPIO speed
    gpio->OSPEEDR &= ~(0x3 << (pin->pin * 2));
    gpio->OSPEEDR |= (pin->speed << (pin->pin * 2));
    
    // Set GPIO output type
    gpio->OTYPER &= ~(0x1 << pin->pin);
    gpio->OTYPER |= (pin->type << pin->pin);
    
    // Set GPIO pull-up/pull-down
    gpio->PUPDR &= ~(0x3 << (pin->pin * 2));
    gpio->PUPDR |= (pin->pull << (pin->pin * 2));
    
    // Set GPIO alternate function
    gpio->AFR[pin->pin / 8] &= ~(0xF << ((pin->pin % 8) * 4));
    gpio->AFR[pin->pin / 8] |= (pin->af << ((pin->pin % 8) * 4));

}

void
gpio_pin_set(gpio_t *pin, uint8_t value) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    if (value) {
        gpio->BSRR |= (1 << pin->pin);
    } else {
        gpio->BSRR |= (1 << (pin->pin + 16));
    }

}

uint32_t
gpio_pin_get(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    return (gpio->IDR & (1 << pin->pin)) ? 1 : 0;

}

void
gpio_pin_toggle(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->ODR ^= (1 << pin->pin);

}

void
gpio_pin_deinit(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->MODER &= ~(0x3 << (pin->pin * 2));
    gpio->OSPEEDR &= ~(0x3 << (pin->pin * 2));
    gpio->OTYPER &= ~(0x1 << pin->pin);
    gpio->PUPDR &= ~(0x3 << (pin->pin * 2));
    gpio->AFR[pin->pin / 8] &= ~(0xF << ((pin->pin % 8) * 4));

}

void
gpio_pin_write(gpio_t *pin, uint16_t value) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    gpio->ODR = value;

}

uint16_t
gpio_pin_read(gpio_t *pin) {

    GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
    return gpio->IDR;

}

