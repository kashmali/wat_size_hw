// Peripheral Includes
#include "stm32f0xx_hal.h"

// Peripheral Structures
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "sys.h"
#include "tim.h"
#include "usart.h"

// Application Includes
//#include "ftpc.h"
#include "w5500.h"

// User Defines
#define vcp_uart huart2

void user_SystemClockConfig(void);
void init_peripherals(void);
void putstr(uint8_t* ptr, uint16_t len);

void user_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void init_peripherals(void)
{
  HAL_Init();
  user_SystemClockConfig();
  MX_SYS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
}

void putstr(uint8_t* ptr, uint16_t len)
{
  // Wait forever to place the characters (don't timeout)
  HAL_UART_Transmit(&vcp_uart, ptr, len, (uint32_t)-1);
  return;
}

int main(void)
{
  uint8_t data = 0x55;
  init_peripherals();

  while(1)
  {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    putstr("hello world!\r\n", 14);
    wiz_send_data(0, &data, 1);
    HAL_Delay(1000);
  }

  return 0;
}
