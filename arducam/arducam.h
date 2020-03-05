#ifndef ARDUCAM_H
#define ARDUCAM_H

#include <stdint.h>
#include <stdbool.h>

// Register defines for ARDUCAM

// Write Flag (1 << 7)
#define CAM_WR_FLAG 0x81

// Test register
#define CAM_TEST_REG 0x00

// Capture control register Bit[2:0] # of frames to be captured
#define CAM_CC_REG 0x01

// Sensor Interface Timing Register
#define CAM_SIT_REG 0x03
  // SIT options
  // Sensor HSync Polarity (0 active high, 1 active low)
  #define CAM_SIT_HSYNC_POL(x) ((x) << 0)
  // Sensor VSync Polarity (0 active high, 1 active low)
  #define CAM_SIT_VSYNC_POL(x) ((x) << 1)
  // Sensor Data Delay (0 no delay, 1 delay = 1 PCLK)
  #define CAM_SIT_SENSOR_DATA_DEL(x) ((x) << 3)
  // FIFO mode control (0 FIFO mode disabled, 1 FIFO mode enabled)
  #define CAM_SIT_FIFO_MODE_CTRL(x) ((x) << 4)
  // Low Power mode control (0 normal mode, 1 low power mode)
  #define CAM_SIT_LOW_PWR_CTRL(x) ((x) << 6)

// FIFO Control Register
#define CAM_FIFO_CTRL_REG 0x04
  // FIFO Ctrl Reg options
  // Clear FIFO write done flag
  #define CAM_FIFO_CL_W_FLAG (1 << 0)
  // Start Capture
  #define CAM_FIFO_START_CAP (1 << 1)
  // Reset FIFO write pointer
  #define CAM_FIFO_RE_R_FLAG (1 << 4)

// GPIO Direction register
#define CAM_GPIO_DIR_REG 0x05
  // GPIO Dir Register options
  // Sensor reset IO direction (0 input, 1 output)
  #define CAM_GPIO_RE_DIR(x) ((x) << 0)
  // Sensor power down IO dir (0 input, 1 output)
  #define CAM_GPIO_PWR0_DIR(x) ((x) << 1)
  // Sensor power enable IO dir (0 input, 1 output)
  #define CAM_GPIO_PWR1_DIR(x) ((x) << 2)

// GPIO Write Register
#define CAM_GPIO_WR_REG 0x06
  // GPIO Write Register Options
  // Sensor Reset IO value
  #define CAM_GPIO_RS_IO (1 << 0)
  // Sensor Power Down IO Value
  #define CAM_GPIO_PWE_DWN (1 << 1)
  // Sensor Power Enable IO Value
  #define CAM_GPIO_PWE_UP (1 << 2)

// Reset OV chip
#define CAM_RESET_CPLD_REG 0x07
  // Reset OV Chip Options
  #define CAM_RESET_CPLD_FLAG (1 << 7)

// Burst FIFO read Operation
#define CAM_RO_BURST_FIFO_REG 0x3C

// Single FIFO read Operation
#define CAM_RO_SINGLE_FIFO_REG 0x3D

// Arduchip version (should be 0x40)
#define CAM_VER_REG 0x40
  // Version response masks
  // Major
  #define CAM_VER_MAJ_MASK 0xF0
  // Minor
  #define CAM_VER_MIN_MASK 0x0F

#define CAM_WRITE_DONE_REG 0x41
  // Write done register values
  #define CAM_WRITE_DONE_MASK (1 << 3)

// Camera write FIFO size Resp: [7:0]
#define CAM_FIFO_SIZE_0_REG 0x42
// Camera write FIFO size Resp: [15:8]
#define CAM_FIFO_SIZE_1_REG 0x43
// Camera write FIFO size Resp: [18:16]
#define CAM_FIFO_SIZE_2_REG 0x44

// GPIO Read Register
#define CAM_GPIO_RD_REG 0x45
  // Following is same as above
  //// GPIO Read Register Response masks
  //// Sensor Reset IO value
  //#define CAM_GPIO_RS_IO (1 << 0)
  //// Sensor Power Down IO Value
  //#define CAM_GPIO_PWE_DWN (1 << 1)
  //// Sensor Power Enable IO Value
  //#define CAM_GPIO_PWE_UP (1 << 2)

// Public Function Declarations
void arducam_init(uint8_t* version);
void arducam_read(uint8_t reg, uint8_t *opts, uint8_t *rxbuf, uint32_t len);
void arducam_send(uint8_t reg, uint8_t *opts);

#endif
