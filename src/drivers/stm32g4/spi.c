#include "drivers/stm32g4/spi.h"
#include "drivers/stm32g4/common.h"


inline int spi_is_busy(struct spi *spi) {
  return spi->instance->SR & SPI_SR_BSY_Msk;
}

inline int spi_get_tx_fifo_level(struct spi *spi) {
  return spi->instance->SR & SPI_SR_FTLVL_Msk;
}

inline int spi_get_rx_fifo_level(struct spi *spi) {
  return spi->instance->SR & SPI_SR_FRLVL_Msk;
}

inline int spi_is_overrun(struct spi *spi) {
  return spi->instance->SR & SPI_SR_OVR_Msk;
}

inline int spi_is_tx_buffer_empty(struct spi *spi) {
  return spi->instance->SR & SPI_SR_TXE_Msk;
}

inline int spi_is_rx_buffer_not_empty(struct spi *spi) {
  return spi->instance->SR & SPI_SR_RXNE_Msk;
}

inline int spi_enable(struct spi *spi) {

  spi->instance->CR1 |= SPI_CR1_SPE;

  return 0;
}

inline int spi_disable(struct spi *spi) {


  spi->instance->CR1 &= ~SPI_CR1_SPE;

  return 0;
}


int spi_init_master(struct spi *spi) {

  // Enable peripheral clock
  if (spi->instance == SPI1) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  } else if (spi->instance == SPI2) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
  } else if (spi->instance == SPI3) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
  } else if (spi->instance == SPI4) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
  } else {
    // Failed to initialize
    return -1;
  }

  // Reset Registers
  spi->instance->CR1 = 0;
  spi->instance->CR2 = 0;

  // Set Baudrate
  spi->instance->CR1 = spi->baudrate << SPI_CR1_BR_Pos; 
  // Configure Polarity & Phase
  spi->instance->CR1 |= (spi->polarity << SPI_CR1_CPOL_Pos) | (spi->phase << SPI_CR1_CPHA_Pos);

  // Select Simplex or half-duplex if needed
  // -- TODO add support for these modes

  // Define frame format
  // -- TODO LSBFIRST

  // Configure CRC
  // -- TODO

  // Configure SSI & SSM
  spi->instance->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR; // Software NSS management, Master

  // Define data size
  spi->instance->CR2 |= (spi->data_size - 1) << SPI_CR2_DS_Pos;

  // Configure SSOE
  return 0;
}

int spi_set_data_size(struct spi *spi, uint8_t data_size) {

  spi->data_size = data_size;

  Modify_register_field(spi->instance->CR2, SPI_CR2_DS, (spi->data_size - 1));

  return 0;
}

int spi_set_fifo_rx_threshold_8bit(struct spi *spi) {

  spi->instance->CR2 |= SPI_CR2_FRXTH;

  return 0;
}

int spi_set_fifo_rx_threshold_16bit(struct spi *spi) {

  spi->instance->CR2 &= ~SPI_CR2_FRXTH;

  return 0;
}

int spi_write(struct spi *spi, uint16_t data) {

  while (!spi_is_tx_buffer_empty(spi));
  spi->instance->DR = data;

  return 0;
}

int spi_write_byte(struct spi *spi, uint8_t byte) {

  while (!spi_is_tx_buffer_empty(spi));
  *(volatile uint8_t *) &spi->instance->DR = byte;
  return 0;
}


int spi_read(struct spi *spi, uint16_t *data) {

  while (!spi_is_rx_buffer_not_empty(spi));
  *data = spi->instance->DR;

  return 0;
}

int spi_transfer(struct spi *spi, uint16_t data_tx, uint16_t *data_rx) {

  spi_write(spi, data_tx);

  while (!spi_is_rx_buffer_not_empty(spi));

  *data_rx = spi->instance->DR;

  return 0;
}




