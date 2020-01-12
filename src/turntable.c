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
