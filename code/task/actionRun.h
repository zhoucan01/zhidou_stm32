#ifndef ACTIONRUN__H
#define ACTIONRUN__H
/* Serialized actuator-command queue data type and task API. */




#include <stdint.h>
typedef enum { ACTION_57_RUN, ACTION_57_STOP, ACTION_42_RUN, ACTION_42_STOP,
  ACTION_28_FORWARD, ACTION_28_BACKWARD, ACTION_28_STOP } action_command_id_t;
typedef struct { action_command_id_t id; int32_t value0; uint32_t value1, value2, value3; } action_command_t;
void actionRunSubmit(const action_command_t *command);

#endif
