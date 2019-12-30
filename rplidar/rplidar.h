#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <stdint.h>

#include "rplidar_msg.h"
#include "stm32f0xx_hal.h"

uint8_t rp_init(UART_HandleTypeDef uart);
uint8_t rp_request(rp_req_t cmd, rp_resp_header_t *resp);
uint8_t rp_get(void *buf, uint32_t size);

#endif
