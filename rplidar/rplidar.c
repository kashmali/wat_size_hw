#include "stm32f0xx_hal.h"
#include "rplidar_msg.h"

// Acts as lock on driver and global instance of uart driver
static UART_HandleTypeDef rp_uart;

static uint8_t putbuf(uint8_t* ptr, uint16_t len)
{
  // Wait 100ms to receive data
  return HAL_UART_Transmit(&rp_uart, ptr, len, 100);
}

static uint8_t get_descriptor(uint8_t *out)
{
  uint8_t data[2];
  do
  {
    HAL_UART_Receive(&rp_uart, &data[0], 1, HAL_MAX_DELAY);
  }
  while(START_FLAG == data[0]);

  HAL_UART_Receive(&rp_uart, &data[1], 1, HAL_MAX_DELAY);

  if(RESP_FLAG == data[1])
  {
    out[0] = data[0];
    out[1] = data[1];
    return HAL_UART_Receive(&rp_uart, (uint8_t*)&out[2], sizeof(rp_resp_header_t) - 2, HAL_MAX_DELAY);
  }
  return HAL_ERROR;
}

static uint8_t checksum(uint8_t *buf, uint8_t size)
{
  if(!buf)
  {
    return 0;
  }

  uint8_t out = 0;
  for(uint8_t i = 0; i < size; i++)
  {
    out ^= buf[i];
  }
  return out;
}

void rp_init(UART_HandleTypeDef uart)
{
  rp_uart = uart;
}

uint8_t rp_get(void *buf, uint32_t size)
{
  return HAL_UART_Receive(&rp_uart, buf, size, HAL_MAX_DELAY);
}

uint8_t rp_request(rp_req_t cmd, rp_resp_header_t *resp)
{
  uint8_t status = HAL_OK;

  switch(cmd)
  {
    // No Response
    case RP_STOP:
    case RP_RESET:
      {
        // Send the request
        uint8_t msg[2];
        msg[0] = START_FLAG;
        msg[1] = cmd;
        status = putbuf(msg, 2);
        // No expected response
      }
      break;
    // Multiple Response
    case RP_SCAN:
    case RP_FORCE_SCAN:
      {
        // Send the request
        uint8_t msg[2];
        msg[0] = START_FLAG;
        msg[1] = cmd;
        status = putbuf(msg, 2);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }

        // Expect a response that fits in the resp header
        status = get_descriptor(resp);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }
      }
      break;
    case RP_EXPRESSS_SCAN:
      {
        uint8_t msg[5];
        msg[0] = START_FLAG;
        msg[1] = RP_EXPRESSS_SCAN;
        msg[2] = 5;
        msg[3] = 1; // as long as it's not 0 it SHOULD use new mode
        msg[4] = 0;
        msg[5] = 0;
        msg[6] = 0;
        msg[7] = 0;
        msg[8] = checksum(msg, 8);
        status = putbuf(msg, 9);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }

        // Expect a response that fits in the resp header
        status = get_descriptor(resp);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }
      }
      break;
    // Single Response
    case RP_GET_INFO:
    case RP_GET_HEALTH:
    case RP_GET_SAMPLE_RATE:
      {
        // Send the request
        uint8_t msg[2];
        msg[0] = START_FLAG;
        msg[1] = cmd;
        status = putbuf(msg, 2);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }

        // Expect a response that fits in the resp header
        status = get_descriptor(resp);
        if(HAL_OK != status)
        {
          // ERROR!!
          break;
        }
      }
      break;
    case RP_GET_LIDAR_CONF:
      break;
  }

  return status;
}
