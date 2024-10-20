#ifndef STM32G4_SPI_H
#define STM32G4_SPI_H

#include "stm32g4_common.h"

enum spi_baudrate {
  SPI_BAUDRATE_PCLK_DIV_2 = 0,
  SPI_BAUDRATE_PCLK_DIV_4,
  SPI_BAUDRATE_PCLK_DIV_8,
  SPI_BAUDRATE_PCLK_DIV_16,
  SPI_BAUDRATE_PCLK_DIV_32,
  SPI_BAUDRATE_PCLK_DIV_64,
  SPI_BAUDRATE_PCLK_DIV_128,
  SPI_BAUDRATE_PCLK_DIV_256,
};

struct spi {
  SPI_TypeDef *instance;
  uint8_t data_size;
  enum spi_baudrate baudrate;
  uint8_t polarity;
  uint8_t phase;
};

int spi_is_busy(struct spi *spi);
int spi_get_tx_fifo_level(struct spi *spi);
int spi_get_rx_fifo_level(struct spi *spi);
int spi_is_overrun(struct spi *spi);
int spi_is_tx_buffer_empty(struct spi *spi);
int spi_is_rx_buffer_not_empty(struct spi *spi);
int spi_enable(struct spi *spi);
int spi_disable(struct spi *spi);
int spi_init_master(struct spi *spi);
int spi_set_data_size(struct spi *spi, uint8_t data_size);
int spi_set_fifo_rx_threshold_8bit(struct spi *spi);
int spi_set_fifo_rx_threshold_16bit(struct spi *spi);
int spi_write(struct spi *spi, uint16_t data);
int spi_write_byte(struct spi *spi, uint8_t byte);
int spi_read(struct spi *spi, uint16_t *data);
int spi_transfer(struct spi *spi, uint16_t data_tx, uint16_t *data_rx);


#endif // STM32G4_SPI_H