#include "drivers/stm32g4/gpio.h"


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

    // Clear output if it is an output
    if (pin->mode == GPIO_MODE_OUTPUT) {
        gpio->ODR &= ~(1 << pin->pin);
    }
    
    // Set GPIO pull-up/pull-down
    gpio->PUPDR &= ~(0x3 << (pin->pin * 2));
    gpio->PUPDR |= (pin->pull << (pin->pin * 2));
    
    // Set GPIO alternate function
    gpio->AFR[pin->pin / 8] &= ~(0xF << ((pin->pin % 8) * 4));
    gpio->AFR[pin->pin / 8] |= (pin->af << ((pin->pin % 8) * 4));

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
