#ifndef RPLIDAR_H
#define RPLIDAR_H

#include "rplidar_msg.h"


uint8_t rp_request(rp_req_t cmd, rp_resp_header_t *resp);
uint8_t rp_run(void);

#endif
