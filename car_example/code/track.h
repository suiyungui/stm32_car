#include "headfile.h"

extern uint8_t track_state;
extern uint16_t track_distance[5];
extern float track_angle[5];
extern uint8_t is_turning;
extern uint8_t task_complete;

void track_init(void);
void check_turn_complete(void);
void track_task_check(void);

