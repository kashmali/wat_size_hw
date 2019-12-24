// Gloabal Includes
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

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
#include "dhcp.h"
#include "dns.h"

// User Defines
#define vcp_uart huart2
#define DHCP_SOCKET 0
#define DNS_SOCKET 1
#define HTTP_SOCKET 2
#define putstr(x) _putstr(x, strlen(x))

void user_SystemClockConfig(void);
void init_peripherals(void);
void _putstr(uint8_t* ptr, uint16_t len);

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

void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&vcp_uart, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
    va_end(args);
}

void _putstr(uint8_t* ptr, uint16_t len)
{
  // Wait forever to place the characters (don't timeout)
  HAL_UART_Transmit(&vcp_uart, ptr, len, HAL_MAX_DELAY);
  return;
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

// Allocate 1KB for the first 2 sockets
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[1024];

bool ip_assigned = false;
void cb_ip_assigned(void)
{
  ip_assigned = true;
}
void cb_ip_conflict(void)
{
  ip_assigned = false;
}

int8_t wizchip_config(void)
{
  int8_t status = 0;
  char name[4];
  uint8_t ver = 0;
  char str_ver[2];
	uint8_t txrx_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
 	wiz_NetInfo net_info = {
        .mac  = { 0xDE, 0xAD, 0xFA, 0xCE, 0xD0, 0x0D },
        .dhcp = NETINFO_DHCP
  };

	// Clear terminal screen
	putstr("\033[2J");

  status = wizchip_init(txrx_sizes, txrx_sizes);
	setSHAR(net_info.mac);
  status = ctlnetwork(CN_SET_NETINFO, (void*)&net_info);
  status = ctlwizchip(CW_GET_ID, name);
  putstr("[");
  putstr(name);
  putstr("] Device Up!\r\n");
  ver = getVERSIONR();
  itoa(ver, str_ver, 10);
  putstr("Version: ");
  putstr(str_ver);
  putstr("\r\n\n");

  putstr("configuring dhcp:\r\n");
  reg_dhcp_cbfunc(cb_ip_assigned, cb_ip_assigned, cb_ip_conflict);
  DHCP_init(DHCP_SOCKET, dhcp_buffer);

  for(uint32_t i = 0; (!ip_assigned) && i < 10000; i++)
  {
    DHCP_run();
    if(0 == i % 1000)
    {
      putstr(".");
    }
  }
  if(ip_assigned)
  {
    putstr("  SUCCESS!\r\n\r\n");
  }
  else
  {
    putstr(" FAILURE!\r\n");
    Error_Handler();
  }

	uint8_t dns[4];
  getIPfromDHCP(net_info.ip);
  getGWfromDHCP(net_info.gw);
  getSNfromDHCP(net_info.sn);
  getDNSfromDHCP(dns);

	UART_Printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nDNS: %d.%d.%d.%d\r\n\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
        dns[0], dns[1], dns[2], dns[3]
  );

  wizchip_setnetinfo(&net_info);

  DNS_init(DNS_SOCKET, dns_buffer);

  uint8_t ex_addr[4];
  char domain_name[] = "www.example.com";
  UART_Printf("Resolving the address of: %s\r\n", domain_name);

  int8_t res = DNS_run(dns, domain_name, ex_addr);
  if(1 != res)
  {
    UART_Printf("DNS lookup failed with code: %d\r\n", res);
    Error_Handler();
  }

  UART_Printf("IP: %d.%d.%d.%d\r\n", ex_addr[0], ex_addr[1], ex_addr[2], ex_addr[3]);

  //ctlnetwork(CN_SET_NETMODE, 0); // Default value already exists
  return status;
}

int main(void)
{
  uint8_t data[5] = {0};
  init_peripherals();

  // Configure the wizchip
  if(0 != wizchip_config())
  {
    Error_Handler();
  }

  memset(data, 0xAA, 5);

  while(1)
  {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
//    putstr("hello world!\r\n", 14);
    wiz_send_data(0, data, 5);
    HAL_Delay(1000);
  }

  return 0;
}
