#include "arducam.h"

#include <stdint.h>

#include "stm32l4xx_hal.h"
#include "spi.h"

uint8_t rev_byte(uint8_t x)
{
  x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1);
  x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2);
  return (((x & 0x0F) << 4) | ((x & 0xF0) >> 4));
}

void arducam_init(uint8_t* version)
{
  if(!version)
  {
    return;
  }

  // Grab the version number
  arducam_read(CAM_VER_REG, NULL, version, sizeof(*version));
}

void arducam_send(uint8_t reg, uint8_t *opts)
{
  uint8_t txlen = 0;
  uint8_t txbuf[2];

  // Add the register of interest to the msg
  txbuf[0] = reg;

  // If there are options, transmit 2 bytes
  if(!opts)
  {
    txlen = 1;
  }
  else
  {
    txlen = 2;
    txbuf[1] = *opts;
  }

  // Enable the chip
  HAL_GPIO_WritePin(ARDUCAM_nSS_GPIO_Port, ARDUCAM_nSS_Pin, GPIO_PIN_RESET);

  // Transmit message (1 byte)
  HAL_SPI_Transmit(&hspi1, txbuf, txlen, (uint32_t)-1);

  // Disable the chip
  HAL_GPIO_WritePin(ARDUCAM_nSS_GPIO_Port, ARDUCAM_nSS_Pin, GPIO_PIN_SET);
}

void arducam_read(uint8_t reg, uint8_t *opts, uint8_t *rxbuf, uint32_t len)
{
  uint8_t txlen = 0;
  uint8_t txbuf[2];

  // Add the register of interest to the msg
  txbuf[0] = reg;

  // If there are options, transmit 2 bytes
  if(!opts)
  {
    txlen = 1;
  }
  else
  {
    txlen = 2;
    txbuf[1] = *opts;
  }

  // Enable the chip
  HAL_GPIO_WritePin(ARDUCAM_nSS_GPIO_Port, ARDUCAM_nSS_Pin, GPIO_PIN_RESET);

  // Transmit message (1 byte)
  (void)HAL_SPI_Transmit(&hspi1, txbuf, txlen, (uint32_t)-1);

  // Receive message (1 byte)
  (void)HAL_SPI_Receive(&hspi1, rxbuf, len, (uint32_t)-1);

  // Disable the chip
  HAL_GPIO_WritePin(ARDUCAM_nSS_GPIO_Port, ARDUCAM_nSS_Pin, GPIO_PIN_SET);

  //for(uint32_t i = 0; i < len; i++)
  //{
  //  rxbuf[i] = rev_byte(rxbuf[i]);
  //}
}

//void arducam_deinit(void)
//{
//
//}
