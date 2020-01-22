// Gloabal Includes
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

// Peripheral Includes
#include "stm32l4xx_hal.h"

// Peripheral Structures
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "lptim.h"
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "sys.h"
#include "tim.h"
#include "usart.h"

// Application Includes
#include "w5500.h"
#include "dhcp.h"
#include "dns.h"
#include "MQTTClient.h"
#include "rplidar.h"
#include "rplidar_msg.h"
#include "turntable.h"

// User Defines
#define vcp_uart huart2
#define rp_uart huart1
#define putstr(x) _putstr(x, strlen(x))
enum socket_type_t
{
  DHCP_SOCKET = 0,
  DNS_SOCKET,
  MQTT_SOCKET,
};

void user_SystemClockConfig(void);
void init_peripherals(void);
void _putstr(uint8_t* ptr, uint16_t len);

void user_SystemClockConfig(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPTIM1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void UART_Printf(const char* fmt, ...) {
    char buff[100];
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

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SYS_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_LPTIM1_Init();
}

// Allocate 1KB for the first 2 sockets
uint8_t dhcp_buffer[500];
uint8_t dns_buffer[500];
uint8_t mqtt_ip[] = { 3, 214, 109, 163 };
#define mqtt_port 1883
uint8_t mqtt_txbuf[1048];
uint8_t mqtt_rxbuf[100];
int32_t rc = 0;

struct opts_struct
{
	char* clientid;
	int nodelimiter;
	char* delimiter;
	enum QoS qos;
	char* username;
	char* password;
	char* host;
	int port;
	int showtopics;
} opts ={ (char*)"stdout-subscriber", 0, (char*)"\n", QOS0, NULL, NULL, mqtt_ip, mqtt_port, 0 };

bool ip_assigned = false;
void cb_ip_assigned(void)
{
  ip_assigned = true;
}
void cb_ip_conflict(void)
{
  ip_assigned = false;
}

// @brief messageArrived callback function
void messageArrived(MessageData* md)
{
	unsigned char testbuffer[50];
	MQTTMessage* message = md->message;

	if (opts.showtopics)
	{
		memcpy(testbuffer,(char*)message->payload,(int)message->payloadlen);
		*(testbuffer + (int)message->payloadlen + 1) = '\n';
    UART_Printf("%s\r\n", testbuffer);
	}

	if (opts.nodelimiter)
		UART_Printf("%.*s", (int)message->payloadlen, (char*)message->payload);
	else
		UART_Printf("%.*s%s", (int)message->payloadlen, (char*)message->payload, opts.delimiter);
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

  UART_Printf("Configuring MQTT session...\r\n");

  Network n;
  MQTTClient c;

  UART_Printf("Creating MQTT socket\r\n");
  NewNetwork(&n, MQTT_SOCKET);
  UART_Printf("Connecting to network\r\n");
  ConnectNetwork(&n, mqtt_ip, mqtt_port);
  UART_Printf("MQTTClientInit...\r\n");
  MQTTClientInit(&c, &n, 1000, mqtt_txbuf, sizeof(mqtt_txbuf), mqtt_rxbuf, sizeof(mqtt_rxbuf));

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.willFlag = 0;
	data.MQTTVersion = 3;
	data.clientID.cstring = opts.clientid;
	data.username.cstring = opts.username;
	data.password.cstring = opts.password;
	data.keepAliveInterval = 60;
	data.cleansession = 1;

  // Connecting to MQTT instance
  UART_Printf("attempting to connect...\r\n");
	rc = MQTTConnect(&c, &data);
	UART_Printf("Connected %d\r\n", rc);
	opts.showtopics = 1;

  // Subscribing
	UART_Printf("Subscribing to %s\r\n", "user_info/name");
	rc = MQTTSubscribe(&c, "user_info/name", opts.qos, messageArrived);
	UART_Printf("Subscribed %d\r\n", rc);
  char payload_data[6] = { 'h', 'e', 'l', 'l', 'o', '\n' };
  MQTTMessage msg = {0};
  msg.id = 1;
  msg.payload = payload_data;
  msg.payloadlen = 6;

  while(1)
  {
  	MQTTYield(&c, data.keepAliveInterval);
    rc = MQTTPublish(&c, "user_info/raw_data", &msg);
  }

  //ctlnetwork(CN_SET_NETMODE, 0); // Default value already exists
  return status;
}

typedef enum
{
  RP_BOOT_S = 0,
  RP_RESET_S,
  RP_SCAN_S,
  RP_STOP_S,
  RP_ERROR_S,
} rp_state_t;

void rp_run(void)
{
  static rp_state_t rp_state = RP_BOOT_S;
  rp_resp_header_t resp = {0};
  rp_info_t info = {0};
  scan_packet_t scan = {0};
  uint8_t status = HAL_OK;
  uint16_t enc_count = 0;

  switch(rp_state)
  {
    case RP_BOOT_S:
    {
      rp_health_t health = {0};
      // Request the health of the lidar
      status = rp_request(RP_GET_HEALTH, &resp);
      //if(status == HAL_TIMEOUT)
      //{
      //  UART_Printf("Timeout!\r\n");
      //  rp_state = RP_STOP_S;
      //  break;
      //}

      // TODO:Verify resp is correct

      // Grab the actual message
      status = rp_get(&health, sizeof(rp_health_t));
      //if(status == HAL_TIMEOUT)
      //{
      //  UART_Printf("Timeout!\r\n");
      //  rp_state = RP_STOP_S;
      //  break;
      //}

      if(RP_GOOD == health.status)
      {
        UART_Printf("RP OK!\r\n");
        rp_state = RP_SCAN_S;
      }
      else if(RP_WARNING == health.status)
      {
        UART_Printf("RP Warning! Continuing...\r\n");
        rp_state = RP_SCAN_S;
      }
      else if(RP_ERROR == health.status)
      {
        rp_state = RP_ERROR_S;
        break;
      }
      else
      {
        // Should not reach here
        rp_state = RP_STOP_S;
        break;
      }

      // Get the device info
      UART_Printf("Getting device info...");
      status = rp_request(RP_GET_INFO, &resp);
      status = rp_get(&info, sizeof(rp_info_t));
      //if(status == HAL_TIMEOUT)
      //{
      //  UART_Printf("Timeout!\r\n");
      //  rp_state = RP_STOP_S;
      //  break;
      //}
      UART_Printf("model: %d\r\nFW min: %d\r\nFW maj: %d\r\nHW: %d\r\n", info.model, info.firmware_min, info.firmware_maj, info.hardware);
      UART_Printf("Serial: ");
      for(uint8_t i = 0; i < 16;i++)
      {
        UART_Printf("%02X ", info.serialnum[i]);
      }
      UART_Printf("\r\n");

      UART_Printf("Spinning up...\r\n");
      HAL_GPIO_WritePin(LIDAR_MTRCTL_GPIO_Port, LIDAR_MTRCTL_Pin, GPIO_PIN_SET);
    }
    break;
    case RP_RESET_S:
    {
      UART_Printf("Resetting...\r\n");
      (void)rp_request(RP_RESET, &resp);

      // Wait 1 second after sending reset message
      HAL_Delay(1000);
      rp_state = RP_BOOT_S;
    }
    break;
    case RP_SCAN_S:
    {
      UART_Printf("Scanning...\r\n");
      (void)rp_request(RP_SCAN, &resp);
      UART_Printf("dist ; theta\r\n");
      UART_Printf("____________\r\n");
      LPTIM_resetEncCount();
      for(;;)
      {
        status = rp_get(&scan, sizeof(scan_packet_t));
        enc_count = LPTIM_getEncCount();
        UART_Printf("%d , %d, %d\r\n", ((scan.dist_h << 8) | scan.dist_l)/4, ((scan.angle_h << 7) | scan.angle_l)/64, enc_count);
        HAL_Delay(10);
      }
    }
    break;
    case RP_STOP_S:
    {
      UART_Printf("Stopping...\r\n");
      (void)rp_request(RP_STOP, NULL);
    }
    break;
    case RP_ERROR_S:
    {
      UART_Printf("RP Error!\r\n");
      rp_state = RP_RESET_S;
    }
    break;
  }
}

int main(void)
{
  uint8_t data[5] = {0};
  init_peripherals();

	// Clear terminal screen
	putstr("\033[2J");

  // Configure the wizchip
  if(0 != wizchip_config())
  {
    Error_Handler();
  }
  memset(data, 0xAA, 5);

  //rp_init(&rp_uart);

  while(1)
  {
    //rp_run();
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_Delay(1000);
    wiz_send_data(0, data, 5);
  }

  return 0;
}

