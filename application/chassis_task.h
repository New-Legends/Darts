#ifndef  CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "remote_control.h"

#define CHASSIS_CONTROL_TIME_MS 2

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
  int16_t ch0_cal;
  int16_t ch3_cal;
} chassis_move_t;

//底盘运动数据
extern chassis_move_t chassis_move;

extern void chassis_task(void const *pvParameters);

#endif
