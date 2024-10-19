#include "cordic.h"



int cordic_init(cordic_t *self) {
    // Enable Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    uint32_t csr_reg = 0;

    csr_reg |= CORDIC_CSR_NRES; // 2 32-bit values are transfered

    // csr_reg |= CORDIC_CSR_ARGSIZE | CORDIC_CSR_RESSIZE; // 16-bit in:out
    csr_reg |= self->cycles  << CORDIC_CSR_PRECISION_Pos; // 4 iterations per cycle
    // CORDIC->CSR |= CORDIC_CSR_SCALE_0;

    // Set Function
    csr_reg |= self->function << CORDIC_CSR_FUNC;


    CORDIC->CSR = csr_reg;

    return 0;

}

void  cordic_write(uint32_t args) {
    CORDIC->WDATA = args;
}  


uint32_t  cordic_read(void) {
    return CORDIC->RDATA;
}


int cordic_deinit(cordic_t *self) {
    CORDIC->CSR = 0;
    RCC->AHB1ENR &= ~(RCC_AHB1ENR_CORDICEN);
}

int cordic_result_is_ready(void) {
    return CORDIC->CSR & CORDIC_CSR_RRDY;

}