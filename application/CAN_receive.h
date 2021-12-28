/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define SHOOT_CAN hcan1


/* CAN send and receive ID */
typedef enum
{
  //左发射电机接收ID
  CAN_LEFT_SHOOT_ALL_ID = 0x200,
  CAN_LEFT_3508_M1_ID = 0x201,
  CAN_LEFT_3508_M2_ID = 0x202,
  CAN_LEFT_3508_M3_ID = 0x203,
  CAN_TRIGGER_MOTOR_ID = 0x204,
  //右发射电机接受ID
  CAN_RIGHT_SHOOT_ALL_ID = 0x1FF,
  CAN_RIGHT_3508_M4_ID = 0x205,
  CAN_RIGHT_3508_M5_ID = 0x206,
  CAN_RIGHT_3508_M6_ID = 0x207,
} can_msg_id_e;

//rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      left_fric: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      right_fric: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      保留: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_left_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t trigger);

extern void CAN_cmd_right_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_left_fric_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_right_fric_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_fric_motor_measure_point(uint8_t i);



#endif
