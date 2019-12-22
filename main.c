#include "Core/Inc/main.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_adc.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_dma.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_gpio.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_spi.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_tim.h"
#include "Drivers/STM32F0xx_HAL_Driver/Inc/stm32f0xx_hal_uart.h"


// ADC
ADC_HandleTypeDef adc1;

// Timer
TIM_HandleTypeDef tim1;
TIM_HandleTypeDef tim2;
TIM_HandleTypeDef tim16;

// SPI Handle + DMA
SPI_HandleTypeDef spi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

// UART Handles + DMA
UART_HandleTypeDef uart1;
DMA_HandleTypeDef dma_usart1_rx;
UART_HandleTypeDef uart2;
DMA_HandleTypeDef dma_usart1_tx;

static void init_peripherals(void)
{
  HAL_Init();
  HAL_TIM_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
}

int main(void)
{
  init_peripherals();

  HAL_delay(100);
  while(1)
  {

  }

  return 0;
}
