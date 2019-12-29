#include "rplidar_msg.h"

// Acts as lock on driver and global instance of uart driver
static UART_HandleTypeDef rp_uart = NULL;

static void putbuf(uint8_t* ptr, uint16_t len)
{
  // Wait forever to place the characters (don't timeout)
  HAL_UART_Transmit(&rp_uart, ptr, len, HAL_MAX_DELAY);
}

static uint8_t get_descriptor(void *out)
{
  return HAL_UART_Receive(&rp_uart, (uint8_t*) out, sizeof(rp_resp_header_t), HAL_MAX_DELAY);
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

uint8_t rp_request(rp_req_t cmd, rp_resp_header_t *resp)
{
  switch(cmd)
  {
    // No Response
    case STOP:
    case RESET:
      {
        // Send the request
        uint8_t msg[2];
        msg[0] = START_FLAG;
        msg[1] = cmd;
        putbuf(msg, 2);
        // No expected response
      }
      break;
    // Multiple Response
    case SCAN:
      break;
    case EXPRESSS_SCAN:
      {
        uint8_t msg[5];
        msg[0] = START_FLAG;
        msg[1] = EXPRESSS_SCAN;
        msg[2] = 5;
        msg[3] = 1; // as long as it's not 0 it SHOULD use new mode
        msg[4] = 0;
        msg[5] = 0;
        msg[6] = 0;
        msg[7] = 0;
        msg[8] = checksum(msg, 8);
        putbuf(msg, 9);
      }
      break;
    case FORCE_SCAN:
      break;
    // Single Response
    case GET_INFO:
    case GET_HEALTH:
    case GET_SAMPLE_RATE:
      {
        // Send the request
        uint8_t msg[2];
        msg[0] = START_FLAG;
        msg[1] = cmd;
        putbuf(msg, 2);

        // Expect a response that fits in the resp header
        if(HAL_OK != get_descriptor(resp))
        {
          // ERROR!!
          break;
        }
      }
      break;
    case GET_LIDAR_CONF:
      break;
  }
}
