#include "turntable.h"

#include <string.h>

#include "tim.h"
#include "lptim.h"
#include "gpio.h"

typedef struct
{
  int16_t motorPWM;
//  uint16_t curr_enc_count;
}tt_t;


static tt_t tt;

void tt_init(void)
{
	LPTIM_initTimers();
	TIM_initTimers();
	HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);	// Set table direction clockwise
  memset(&tt, 0, sizeof(tt));
}

void tt_fsm(tt_state_t *state)
{
  // Gains for the control loop
#define PROP_GAIN 1
#define INT_GAIN 0
#define DERI_GAIN 0

  switch(*state)
  {
    case IDLE_TT:
      {
        tt.motorPWM = 0;
        LPTIM_resetEncCount();
      }
      break;
    case START_ROTATE_TT:
      {
        tt.motorPWM = 4000;
        // Start the system up gradually...
        *state = ROTATE_TT;
      }
      break;
    case ROTATE_TT:
      {
        tt.motorPWM = 4000;
				HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      }
      break;
    case END_ROTATE_TT:
      {
        // Spin the system down gradually...
        tt.motorPWM = 0;
      }
      break;
    case ERROR_TT:
      {
        // An error has occured
        tt.motorPWM = 0;
      }
      break;
  }

  TIM_setSpeed((uint16_t)tt.motorPWM);
}

// mdeg_p_s milli degrees per second the loop should attain
// @ 45000 counts per revolution, each count is 8mdeg
int16_t turntable_ctrl(uint32_t mdeg_p_s)
{
  static bool init = true;
  static uint32_t last_time = 0;
  static uint32_t last_count = 0;

  int32_t last_time = 0;
  uint32_t last_count = 0;
  int32_t err = 0;

  if(init)
  {
    last_time = HAL_GetTick();
    last_count = LPTIM_getEncCount();
    err = 0;

    init = false;
  }

  // Get the current counts
  curr_time = HAL_GetTick();
  curr_count = LPTIM_getEncCount();

  // determine the rate
  err = ((curr_count - last_count)/(curr_time - last_time)) - mdeg_p_s; //TODO(aamalI)Adjust to the conversion!!!

  // End of the function, update the curr -> last
  last_count = curr_count;
  last_time = curr_time;
}
