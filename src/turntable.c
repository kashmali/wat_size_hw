#include "turntable.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#include "tim.h"
#include "lptim.h"
#include "gpio.h"
#include "usart.h"

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

#if 0
static void UART_Printf(const char* fmt, ...) {
    char buff[100];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff),
                      HAL_MAX_DELAY);
    va_end(args);
}
#endif

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
        //tt.motorPWM = 4000;
        // Start the system up gradually...
        for(int i = 0; i < 250; i++)
        {
          HAL_Delay(10);
          tt.motorPWM = i*10;
          TIM_setSpeed((uint16_t)tt.motorPWM);
        }
        *state = ROTATE_TT;
      }
      break;
    case ROTATE_TT:
      {
        //tt.motorPWM = turntable_ctrl(1500);
        tt.motorPWM = 2500; // Use a constant for now
      }
      break;
    case END_ROTATE_TT:
      {
        for(int i = 0; i < 250; i++)
        {
          HAL_Delay(10);
          tt.motorPWM = 2500 - (i*10);
          TIM_setSpeed((uint16_t)tt.motorPWM);
        }
        *state = IDLE_TT;
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

#if 0
// mdeg_p_s milli degrees per second the loop should attain
// @ 45000 counts per revolution, each count is 8mdeg
int16_t turntable_ctrl(uint32_t mdeg_p_s)
{

// Gains for the control loop
#define PROP_GAIN 110
#define INT_GAIN 0
#define DERI_GAIN 110

  static bool init = true;
  static uint32_t last_tick = 0;
  static uint32_t last_count = 0;
  static int32_t last_err = 0;
  static int16_t pwm = 0;

  int32_t curr_tick = 0;
  uint32_t curr_count = 0;
  int32_t curr_err = 0;

  if(init)
  {
    last_count = LPTIM_getEncCount();
    last_tick = HAL_GetTick();
    last_err = 0;
    pwm = 0;

    init = false;
  }

  // Get the current counts
  curr_tick = HAL_GetTick();
  curr_count = LPTIM_getEncCount();

  // determine the error in the rate
	int32_t rate = (((curr_count - last_count)*8*1000)/(curr_tick - last_tick));
  curr_err = rate - mdeg_p_s;

	UART_Printf("rate: %d\r\n", rate);
	UART_Printf("curr err: %d\r\n", curr_err);

  // Determine the new PWM to apply
  curr_err *= -1;

  pwm += curr_err * (PROP_GAIN + INT_GAIN + DERI_GAIN)/100;

  // Saturate the output if out of bounds
  if(pwm > 5000)
  {
  	pwm = 5000;
  }
  else if (pwm < 0)
  {
  	pwm = 0;
  }

  // End of the function, update the curr -> last
  last_count = curr_count;
  last_tick = curr_tick;
  last_err = curr_err;

  return pwm;
}
#endif
