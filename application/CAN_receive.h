/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
#define LASER_DISTANCE hcan2


/* CAN send and receive ID */
typedef enum
{
  //����������ID
  CAN_LEFT_SHOOT_ALL_ID = 0x200,
  CAN_LEFT_3508_M1_ID = 0x201,
  CAN_LEFT_3508_M2_ID = 0x202,
  CAN_LEFT_3508_M3_ID = 0x203,
  //�ҷ���������ID
  CAN_RIGHT_SHOOT_ALL_ID = 0x1FF,
  CAN_RIGHT_3508_M4_ID = 0x205,
  CAN_RIGHT_3508_M5_ID = 0x206,
  CAN_RIGHT_3508_M6_ID = 0x207,
  CAN_PULL_2006_ID = 0x208,
} can1_msg_id_e;
typedef enum
{
  CAN_SENSOR_ALL_ID = 0x200,
  CAN_LASER_DISTANCE_ID = 0x201,
  CAN_TRIGGER_MOTOR_ID = 0x202,
} can2_msg_id_e;

//rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int16_t round;
} motor_measure_t;

typedef struct
{
    uint8_t dis1;
    uint16_t dis2;
    uint8_t dis_status;
    uint16_t signal_strength;
    int16_t dis0;
    float dis;
} sensor_measure_t;



/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      left_fric: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      right_fric: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      trigger: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      ����: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_left_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t trigger);

extern void CAN_cmd_right_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern void CAN2_cmd_sensor(int16_t laser, int16_t model2, int16_t model3, int16_t model4);

extern void can_receive_init(void);

extern const motor_measure_t *get_trigger_motor_measure_point(void);

extern const motor_measure_t *get_pull_motor_measure_point(void);

/**
  * @brief          ����Ħ���ֵ�� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_left_fric_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_right_fric_motor_measure_point(uint8_t i);

extern const motor_measure_t *get_fric_motor_measure_point(uint8_t i);

extern const sensor_measure_t *get_sensor_measure_point(uint8_t i);

#endif
