#include "stm32f0xx_hal.h"
#include "rplidar_msg.h"
#include "fifo.h"

// Acts as lock on driver and global instance of uart driver
static UART_HandleTypeDef *rp_uart;

#define RP_BUF_SIZE 1000
static uint8_t _rx_buf[RP_BUF_SIZE];
static fifo_t rp_recv = {0};
static uint8_t resv[1];

void rp_recv_it(UART_HandleTypeDef *uart)
{
  uint32_t isrflags   = READ_REG(uart->Instance->ISR);
  uint32_t cr1its     = READ_REG(uart->Instance->CR1);
  uint8_t data = 0;

  if (((isrflags & USART_ISR_RXNE) != 0U)
      && ((cr1its & USART_CR1_RXNEIE) != 0U))
  {
    data = READ_REG(uart->Instance->RDR);
    fifo_push(&rp_recv, data);
    isrflags = (isrflags & ~USART_ISR_RXNE);
  }

}

static uint8_t putbuf(uint8_t* ptr, uint16_t len)
{
  // Wait 100ms to receive data
  return HAL_UART_Transmit(rp_uart, ptr, len, 100);
}

static uint8_t get_descriptor(uint8_t *out)
{
  uint8_t temp = 0;
  while(1 == fifo_peek(&rp_recv, &temp) || (temp != START_FLAG))
  {
    // Do nothing while waiting for input
    // Delay for 10ms then look again
    HAL_Delay(10);
  }

  uint8_t i = 0;
  for(; i < sizeof(rp_resp_header_t); i++)
  {
    while(1 == fifo_peek(&rp_recv, &temp))
    {
      // Wait for data to come in
      HAL_Delay(10);
    }
    (void)fifo_pop(&rp_recv, &temp);
    out[i] = temp;
  }

  // Return the number of bytes copied
  return i;
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

void rp_init(UART_HandleTypeDef *uart)
{
  rp_uart = uart;
  fifo_init(&rp_recv, _rx_buf, RP_BUF_SIZE);
  //(void)HAL_UART_Receive_IT(&rp_uart, resv, 1);
  /* Enable the UART Data Register Not Empty interrupt */
  SET_BIT(rp_uart->Instance->CR1, USART_CR1_RXNEIE);
}

uint8_t rp_get(uint8_t *buf, uint32_t size)
{
  uint8_t temp = 0;
  uint8_t i = 0;
  for(; i < size; i++)
  {
    while(1 == fifo_pop(&rp_recv, &temp))
    {
      // Wait for data to come in
      HAL_Delay(10);
    }
    buf[i] = temp;
  }

  // Return the number of bytes copied
  return i;
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
    case RP_MOTOR_SPEED_CTRL:
        // Send the request
        //uint8_t msg[6];
        //msg[0] = START_FLAG;
        //msg[1] = cmd;
        //msg
        //status = putbuf(msg, 2);
        //if(HAL_OK != status)
        //{
        //  // ERROR!!
        //  break;
        //}
      break;
  }

  return status;
}
