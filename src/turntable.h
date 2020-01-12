#ifndef TURNTABLE_H
#define TURNTABLE_H

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

#endif
