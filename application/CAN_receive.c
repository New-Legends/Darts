/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             锟斤拷锟斤拷锟斤拷CAN锟叫断斤拷锟秸猴拷锟斤拷锟斤拷锟斤拷锟秸碉拷锟斤拷锟斤拷锟�,CAN锟斤拷锟酵猴拷锟斤拷锟斤拷锟酵碉拷锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷.
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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

/*
锟斤拷锟斤拷锟斤拷锟�, 

*/
static motor_measure_t motor_shoot[8];

static CAN_TxHeaderTypeDef shoot_left_tx_message;
static uint8_t shoot_left_can_send_data[8];

static CAN_TxHeaderTypeDef shoot_right_tx_message;
static uint8_t shoot_right_can_send_data[8];

/**
  * @brief          hal锟斤拷CAN锟截碉拷锟斤拷锟斤拷,锟斤拷锟秸碉拷锟斤拷锟斤拷锟�
  * @param[in]      hcan:CAN锟斤拷锟街革拷锟�
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  switch (rx_header.StdId)
  {
  case CAN_LEFT_3508_M1_ID:
  case CAN_LEFT_3508_M2_ID:
  case CAN_LEFT_3508_M3_ID:
  case CAN_TRIGGER_MOTOR_ID:
  case CAN_RIGHT_3508_M4_ID:
  case CAN_RIGHT_3508_M5_ID:
  case CAN_RIGHT_3508_M6_ID:
  case CAN_PULL_2006_ID:
  {

    static uint8_t i = 0;
    //get motor id
    i = rx_header.StdId - CAN_LEFT_3508_M1_ID;
    get_motor_measure(&motor_shoot[i], rx_data);
  }

  default:
  {
    break;
  }
  }
}

/**
  * @brief          锟斤拷锟酵碉拷锟斤拷锟斤拷频锟斤拷锟�(0x205,0x206,0x207,0x208)
  * @param[in]      left_fric: (0x205) 3508锟斤拷锟斤拷锟斤拷频锟斤拷锟�, 锟斤拷围 [-16384,16384]
  * @param[in]      right_fric: (0x206) 3508锟斤拷锟斤拷锟斤拷频锟斤拷锟�, 锟斤拷围 [-16384,16384]
  * @param[in]      trigger: (0x207) 2006锟斤拷锟斤拷锟斤拷频锟斤拷锟�, 锟斤拷围 [-10000,10000]
  * @param[in]      锟斤拷锟斤拷: (0x208) 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷频锟斤拷锟�
  * @retval         none
  */
void CAN_cmd_left_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t trigger)
{
  uint32_t send_mail_box;
  shoot_left_tx_message.StdId = CAN_LEFT_SHOOT_ALL_ID;
  shoot_left_tx_message.IDE = CAN_ID_STD;
  shoot_left_tx_message.RTR = CAN_RTR_DATA;
  shoot_left_tx_message.DLC = 0x08;
  shoot_left_can_send_data[0] = (motor1 >> 8);
  shoot_left_can_send_data[1] = motor1;
  shoot_left_can_send_data[2] = (motor2 >> 8);
  shoot_left_can_send_data[3] = motor2;
  shoot_left_can_send_data[4] = (motor3 >> 8);
  shoot_left_can_send_data[5] = motor3;
  shoot_left_can_send_data[6] = (trigger >> 8);
  shoot_left_can_send_data[7] = trigger;

  HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_left_tx_message, shoot_left_can_send_data, &send_mail_box);
}

void CAN_cmd_right_shoot(int16_t motor1, int16_t motor2, int16_t motor3, int16_t pull)
{
  uint32_t send_mail_box;
  shoot_right_tx_message.StdId = CAN_RIGHT_SHOOT_ALL_ID;
  shoot_right_tx_message.IDE = CAN_ID_STD;
  shoot_right_tx_message.RTR = CAN_RTR_DATA;
  shoot_right_tx_message.DLC = 0x08;
  shoot_right_can_send_data[0] = (motor1 >> 8);
  shoot_right_can_send_data[1] = motor1;
  shoot_right_can_send_data[2] = (motor2 >> 8);
  shoot_right_can_send_data[3] = motor2;
  shoot_right_can_send_data[4] = (motor3 >> 8);
  shoot_right_can_send_data[5] = motor3;
  shoot_right_can_send_data[6] = (pull >> 8);
  shoot_right_can_send_data[7] = pull;

  HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_right_tx_message, shoot_right_can_send_data, &send_mail_box);
}

/**
  * @brief          锟斤拷锟截诧拷锟斤拷锟斤拷锟� 3508锟斤拷锟斤拷锟斤拷锟街革拷锟�
  * @param[in]      none
  * @retval         锟斤拷锟斤拷锟斤拷锟街革拷锟�
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_shoot[3];
}

/**
  * @brief          2006推弹电机数据反馈
  * @param[in]      none
  * @retval         锟斤拷锟斤拷锟斤拷锟街革拷锟�
  */
const motor_measure_t *get_pull_motor_measure_point(void)
{
  return &motor_shoot[8];
}

/**
  * @brief          锟斤拷锟斤拷锟斤拷摩锟斤拷锟斤拷 3508锟斤拷锟斤拷锟斤拷锟街革拷锟�
  * @param[in]      i: 锟斤拷锟斤拷锟斤拷,锟斤拷围[0,3]
  * @retval         锟斤拷锟斤拷锟斤拷锟街革拷锟�
  */
const motor_measure_t *get_left_fric_motor_measure_point(uint8_t i)
{
  return &motor_shoot[i];
}

/**
  * @brief          锟斤拷锟斤拷锟斤拷摩锟斤拷锟斤拷锟斤拷锟斤拷
  * @param[in]      i: 锟斤拷锟斤拷锟斤拷,锟斤拷围[4,6]
  * @retval         锟斤拷锟斤拷锟斤拷锟街革拷锟�
  */
const motor_measure_t *get_right_fric_motor_measure_point(uint8_t i)
{
  return &motor_shoot[i];
}

const motor_measure_t *get_fric_motor_measure_point(uint8_t i)
{
  return &motor_shoot[i];
}
