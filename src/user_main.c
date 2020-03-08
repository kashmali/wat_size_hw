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
#include "rng.h"
#include "spi.h"
#include "sys.h"
#include "tim.h"
#include "usart.h"

// Application Includes
#include "arducam.h"
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
void _putstr(char* ptr, uint16_t len);

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
                              |RCC_PERIPHCLK_RNG | RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
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

void _putstr(char* ptr, uint16_t len)
{
  // Wait forever to place the characters (don't timeout)
  HAL_UART_Transmit(&vcp_uart, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
  MX_RNG_Init();
  MX_LPTIM1_Init();

  // Disable the camera to start
  HAL_GPIO_WritePin(ARDUCAM_nSS_GPIO_Port, ARDUCAM_nSS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(W5500_nSS_GPIO_Port, W5500_nSS_Pin, GPIO_PIN_SET);
}

// Allocate 1KB for the first 2 sockets
uint8_t dhcp_buffer[500];
uint8_t dns_buffer[500];
char mqtt_ip[] = { 3, 214, 109, 163 };
#define mqtt_port 1883
uint8_t mqtt_txbuf[2048];
uint8_t mqtt_rxbuf[200];
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

bool mqtt_start_scan = false;
// @brief messageArrived callback function
void lidar_messageArrived(MessageData* md)
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

  mqtt_start_scan = true;
}

//void camera_messageArrived(MessageData* md)
//{
//	unsigned char testbuffer[50];
//	MQTTMessage* message = md->message;
//
//	if (opts.showtopics)
//	{
//		memcpy(testbuffer,(char*)message->payload,(int)message->payloadlen);
//		*(testbuffer + (int)message->payloadlen + 1) = '\n';
//    UART_Printf("%s\r\n", testbuffer);
//	}
//
//	if (opts.nodelimiter)
//		UART_Printf("%.*s", (int)message->payloadlen, (char*)message->payload);
//	else
//		UART_Printf("%.*s%s", (int)message->payloadlen, (char*)message->payload, opts.delimiter);
//
//  mqtt_start_scan = true;
//}

// TODO(aamali): Pass these into the function
Network n;
MQTTClient c;
MQTTPacket_connectData data;

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

  int8_t res = DNS_run(dns, (uint8_t *)domain_name, ex_addr);
  if(1 != res)
  {
    UART_Printf("DNS lookup failed with code: %d\r\n", res);
    Error_Handler();
  }

  UART_Printf("IP: %d.%d.%d.%d\r\n", ex_addr[0], ex_addr[1], ex_addr[2], ex_addr[3]);

  UART_Printf("Configuring MQTT session...\r\n");

  UART_Printf("Creating MQTT socket\r\n");
  NewNetwork(&n, MQTT_SOCKET);
  UART_Printf("Connecting to network\r\n");

  uint32_t myport = (uint32_t)-1;
  HAL_RNG_GenerateRandomNumber(&hrng, &myport);
  ConnectNetwork(&n, (uint8_t *)mqtt_ip, mqtt_port, (uint16_t)(myport & 0x0000FFFF));
  UART_Printf("Rng: %d\r\n", (uint16_t)(myport & 0x0000FFFF));

  UART_Printf("MQTTClientInit...\r\n");
  MQTTClientInit(&c, &n, 1000, mqtt_txbuf, sizeof(mqtt_txbuf), mqtt_rxbuf, sizeof(mqtt_rxbuf));

	data = (MQTTPacket_connectData)MQTTPacket_connectData_initializer;
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

  // Subscribing
	UART_Printf("Subscribing to %s\r\n", "lidar_lidar_status/scan");
	rc = MQTTSubscribe(&c, "lidar/lidar_status/scan", opts.qos, lidar_messageArrived);
	//rc = MQTTSubscribe(&c, "camera/camera_status/capture", opts.qos, camera_messageArrived);
	UART_Printf("Subscribed %d\r\n", rc);
  return status;
}

void arducam_config(void)
{
  HAL_Delay(1000);
  uint8_t test = 0x55;
  arducam_send(CAM_TEST_REG | CAM_WR_FLAG, &test);
  HAL_Delay(100);
  test = 0;
  arducam_read(CAM_TEST_REG, NULL, &test, sizeof(test));
  HAL_Delay(100);
  UART_Printf("Wrote: 0x55, Read: 0x%x\r\n", test);

  HAL_Delay(1000);

  uint8_t ver = 0;
  arducam_init(&ver);
  while(!((7 == (ver >> 4)) && 3 == (ver & CAM_VER_MIN_MASK)))
  {
    arducam_init(&ver);
  }
  UART_Printf("ARDUCAM_VERSION...\r\n");
  UART_Printf("maj:%d min:%d\r\n", (ver >> 4), (ver & CAM_VER_MIN_MASK));
  UART_Printf("RAW:0x%x\r\n", ver);
}

void arducam_capture_send(void)
{
  UART_Printf("Clear fifo flag...\r\n");
  uint8_t in = CAM_FIFO_CL_W_FLAG;
  arducam_send(CAM_FIFO_CTRL_REG | CAM_WR_FLAG, &in);
  in = CAM_FIFO_CL_W_FLAG;
  arducam_send(CAM_FIFO_CTRL_REG | CAM_WR_FLAG, &in);
  in = CAM_SIT_FIFO_MODE_CTRL(1);
  arducam_send(CAM_SIT_REG | CAM_WR_FLAG, &in);

  uint8_t num_frames = 0;
  arducam_send(CAM_CC_REG | CAM_WR_FLAG, &num_frames);

  UART_Printf("Starting capture...\r\n");
  in = CAM_FIFO_START_CAP;
  arducam_send(CAM_FIFO_CTRL_REG | CAM_WR_FLAG, &in);

  uint8_t out = 0;
  arducam_read(CAM_WRITE_DONE_REG, NULL, &out, 1);
  while(!(out & CAM_WRITE_DONE_MASK))
  {
    HAL_Delay(10);
    UART_Printf("trying...\r\n");
    arducam_read(CAM_WRITE_DONE_REG, NULL, &out, 1);
  }

  HAL_Delay(100);

  uint8_t len1, len2, len3;
  arducam_read(CAM_FIFO_SIZE_0_REG, NULL, &len1, 1);
  UART_Printf("first reg: %x\r\n", len1);
  HAL_Delay(100);
  arducam_read(CAM_FIFO_SIZE_1_REG, NULL, &len2, 1);
  UART_Printf("2nd reg: %x\r\n", len2);
  HAL_Delay(100);
  arducam_read(CAM_FIFO_SIZE_2_REG, NULL, &len3, 1);
  UART_Printf("3rd reg: %x\r\n", len3);
  HAL_Delay(100);
  uint32_t total_length = len1 + (len2 << 8) + (len3 << 16);

#define MAX_FIFO_SIZE 0x5FFFF

  UART_Printf("Total length to read: %d\r\n", total_length);
  if(total_length > MAX_FIFO_SIZE)
  {
    UART_Printf("RESULTANT IMAGE TOO BIG!\r\n");
  }

  uint8_t txbuf[256];
  MQTTMessage msg = {0};
  msg.id = 1;
  msg.payload = txbuf;
  msg.payloadlen = sizeof(txbuf);
  for(uint32_t j = 0; j < total_length/sizeof(txbuf); j++)
  {
    for(uint32_t i = 0; i < sizeof(txbuf); i++)
    {
      arducam_read(CAM_RO_SINGLE_FIFO_REG, NULL, &txbuf[i], 1);
      //UART_Printf("%x,", txbuf[i]);
    }
    rc = MQTTPublish(&c, "camera/raw_data", &msg);
  }

  uint8_t leftover_buf[256];
  for(uint32_t j = 0; j < total_length % sizeof(txbuf); j++)
  {
    arducam_read(CAM_RO_SINGLE_FIFO_REG, NULL, &leftover_buf[j], 1);
    UART_Printf("%x,", leftover_buf[j]);
  }

  MQTTMessage leftover = {0};
  msg.id = 1;
  msg.payload = leftover_buf;
  msg.payloadlen = total_length % sizeof(txbuf);
  rc = MQTTPublish(&c, "camera/raw_data", &leftover);

  UART_Printf("\r\nsending leftovers\r\n");

  uint8_t empty[1] = {0};
  MQTTMessage finished = {0};
  msg.id = 1;
  msg.payload = empty;
  msg.payloadlen = sizeof(empty);
  rc = MQTTPublish(&c, "camera/camera_status/capture_complete", &finished);
  UART_Printf("\r\ndone transmition\r\n");
}

typedef enum
{
  RP_BOOT_S = 0,
  RP_RESET_S,
  RP_IDLE_S,
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

  switch(rp_state)
  {
    case RP_BOOT_S:
    {
      rp_health_t health = {0};
      // Request the health of the lidar
      (void)rp_request(RP_GET_HEALTH, &resp);
      // Grab the actual message
      (void)rp_get(&health, sizeof(rp_health_t));
      if(RP_GOOD == health.status)
      {
        UART_Printf("RP OK!\r\n");
        rp_state = RP_IDLE_S;
      }
      else if(RP_WARNING == health.status)
      {
        UART_Printf("RP Warning! Continuing...\r\n");
        rp_state = RP_IDLE_S;
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
      (void)rp_request(RP_GET_INFO, &resp);
      (void)rp_get(&info, sizeof(rp_info_t));

      UART_Printf("model: %d\r\nFW min: %d\r\nFW maj: %d\r\nHW: %d\r\n", info.model, info.firmware_min, info.firmware_maj, info.hardware);
      UART_Printf("Serial: ");
      for(uint8_t i = 0; i < 16;i++)
      {
        UART_Printf("%02X ", info.serialnum[i]);
      }
      UART_Printf("\r\n");

      UART_Printf("Spinning up...\r\n");
      HAL_GPIO_WritePin(LIDAR_MTRCTL_GPIO_Port, LIDAR_MTRCTL_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
    }
    break;
    case RP_IDLE_S:
    {
      UART_Printf("Waiting for message from server...\r\n");
      while(!mqtt_start_scan)
      {
    	  MQTTYield(&c, data.keepAliveInterval);
        HAL_Delay(500);
      }
      mqtt_start_scan = false; // TODO(aamali):Change this (when multiple devices)
      rp_state = RP_SCAN_S;
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
      // Take a picture and send it to the server
      arducam_capture_send();

      uint16_t to_publish[360];
      MQTTMessage msg = {0};
      msg.id = 1;
      msg.payload = to_publish;
      msg.payloadlen = sizeof(to_publish);

      scan_packet_t temp_buf[128];
      UART_Printf("Scanning...\r\n");
      (void)rp_request(RP_SCAN, &resp);
      //UART_Printf("dist ; theta\r\n");
      //UART_Printf("____________\r\n");
      uint16_t temp_dist = 0;
      uint16_t temp_angle = 0;
      uint16_t enc_count = 0;
      (void)rp_get(temp_buf, sizeof(temp_buf));

      tt_state_t tt_s = START_ROTATE_TT;
      tt_fsm(&tt_s);

      HAL_Delay(10);
      tt_s = ROTATE_TT;
      tt_fsm(&tt_s);
      LPTIM_resetEncCount();
      for(int j = 0;enc_count < 45000;j++)
      {
        for(uint32_t i = 0; i < 360;)
        {
          (void)rp_get(&scan, sizeof(scan_packet_t));
          enc_count = LPTIM_getEncCount();
          temp_dist = scan.dist_h << 8;
          temp_dist |= scan.dist_l;
          temp_angle = scan.angle_h << 7;
          temp_angle |= scan.angle_l;
          temp_angle /= 64;
          if(temp_angle > 360)
          {
            temp_angle -= 360;
          }
          to_publish[i++] = temp_dist;
          to_publish[i++] = temp_angle;
          to_publish[i++] = enc_count;
          //UART_Printf("%d, %d, %d\n", temp_dist/4, temp_angle, enc_count);
        }
        tt_fsm(&tt_s);
        rc = MQTTPublish(&c, "lidar/raw_data", &msg);
        UART_Printf("sent %d\r\n", j);
      }

      tt_s = END_ROTATE_TT;
      tt_fsm(&tt_s);

      uint8_t empty[1] = {0};
      MQTTMessage finished = {0};
      msg.id = 1;
      msg.payload = empty;
      msg.payloadlen = sizeof(empty);
      rc = MQTTPublish(&c, "lidar/lidar_status/scan_complete", &finished);

      // Go to the stopped state
      rp_state = RP_STOP_S;
    }
    break;
    case RP_STOP_S:
    {
      UART_Printf("Stopping...\r\n");
      // Stop the lidar from spinning
      HAL_GPIO_WritePin(LIDAR_MTRCTL_GPIO_Port, LIDAR_MTRCTL_Pin, GPIO_PIN_RESET);
      (void)rp_request(RP_STOP, NULL);
      // Go back to top of loop
      rp_state = RP_BOOT_S;
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
  init_peripherals();

	// Clear terminal screen
	putstr("\033[2J");

  // Configure the wizchip
  if(0 != wizchip_config())
  {
    Error_Handler();
  }

  rp_init(&rp_uart);
  arducam_config();

  tt_init();
  tt_state_t tt_s = IDLE_TT;
  tt_fsm(&tt_s);

  while(1)
  {
    rp_run();
    HAL_Delay(10);
  }

  // Should never reach here
  tt_s = END_ROTATE_TT;
  tt_fsm(&tt_s); // More advanced turntable motion

  return 0;
}

