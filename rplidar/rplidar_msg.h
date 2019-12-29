#ifndef RPLIDAR_MSG_H
#define RPLIDAR_MSG_H

#define START_FLAG 0xA5
#define RESP_FLAG 0x5A

// Error codes
#define RP_GOOD 0
#define RP_WARNING 1
#define RP_ERROR 2

typedef enum
{
  // No Response
  STOP = 0x25,
  RESET = 0x40,
  // Multiple Response
  SCAN = 0x20,
  EXPRESSS_SCAN = 0x82,
  FORCE_SCAN = 0x21,
  // Single Response
  GET_INFO = 0x50,
  GET_HEALTH = 0x52,
  GET_SAMPLE_RATE = 0x59,
  GET_LIDAR_CONF = 0x84;
}rp_req_t;

typedef struct
{
  uint8_t sync; // Should be START_FLAG
  rp_req_t cmd;
  uint8_t size;
  uint8_t *data;
} rp_req_packet_t;

typedef struct
{
  uint8_t sync; // Should be START_FLAG
  uint8_t sync2; // Should be RESP_FLAG
  uint32_t size : 30;
  uint32_t mode : 2;
  uint8_t type;
} rp_resp_header_t;

typedef struct
{
  uint8_t quality : 6;
  uint8_t new_scan : 1;
  uint8_t nnew_scan : 1;
  uint8_t angle_l : 7; // q6
  uint8_t check : 1; // Should always be 1
  uint8_t angle_h; // q6
  uint8_t dist_l; // q2
  uint8_t dist_h; // q2
} scan_packet_t

typedef struct
{
  // First
  uint8_t  dist1_l : 6;
  uint8_t  d_theta1_h : 2;
  uint8_t  dist1_h;
  // Second
  uint8_t  dist2_l : 6;
  uint8_t  d_theta2_h : 2;
  uint8_t  dist2_h;
  // Offsets
  uint8_t  d_theta2_h : 4;
  uint8_t  d_theta1_h : 4;
} cabin_t;

typedef struct
{
  uint8_t predict2;
  uint8_t predict1;
  uint8_t major;
} ultra_cabin_t;

typedef struct
{
  uint8_t sync1 : 4;
  uint8_t chksum_l : 4;
  uint8_t sync2 : 4;
  uint8_t chksum_h : 4;
  uint8_t start_angle_l; // q6
  uint8_t new_scan : 1;
  uint8_t start_angle_h : 7;
  cabin_t cabin[16];
} xscan_packet_t;

typedef struct
{
  uint8_t sync1 : 4;
  uint8_t chksum_l : 4;
  uint8_t sync2 : 4;
  uint8_t chksum_h : 4;
  uint8_t start_angle_l; // q6
  uint8_t new_scan : 1;
  uint8_t start_angle_h : 7;
  ultra_cabin_t ucabin[32];
} uxscan_packet_t

#endif
