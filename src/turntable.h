#ifndef TURNTABLE_H
#define TURNTABLE_H

#include <stdint.h>

typedef enum
{
  IDLE_TT = 0,
  START_ROTATE_TT,
  ROTATE_TT,
  END_ROTATE_TT,
  ERROR_TT,

} tt_state_t;

void tt_init(void);
void tt_fsm(tt_state_t *state);
int16_t turntable_ctrl(uint32_t deg_p_s);

#endif
