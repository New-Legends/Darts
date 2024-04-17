#include "chassis_task.h"
#include "main.h"
#include "tim.h"

chassis_move_t chassis_move;

static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_set_control(chassis_move_t *chassis_move_init);

void chassis_task(void const *pvParameters)
{
    chassis_init(&chassis_move);
    Referee_init();
    while (1)
    {
        chassis_set_control(&chassis_move);
        Referee_unpack();
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

void chassis_init(chassis_move_t *chassis_move_init)
{
    chassis_move_init->chassis_RC = get_remote_control_point();
}

void chassis_set_control(chassis_move_t *chassis_move_mode)
{
    //步进电机速度赋值
    if(chassis_move_mode->chassis_RC->rc.ch[0] >= 0)
    {
            chassis_move.ch0_cal = 670 - chassis_move_mode->chassis_RC->rc.ch[0];
    }
    else if(chassis_move_mode->chassis_RC->rc.ch[0] < 0)
    {
            chassis_move.ch0_cal = 670 + chassis_move_mode->chassis_RC->rc.ch[0];
    }

    if(chassis_move_mode->chassis_RC->rc.ch[3] >= 0)
    {
            chassis_move.ch3_cal = 670 - chassis_move_mode->chassis_RC->rc.ch[3];
    }
    else if(chassis_move_mode->chassis_RC->rc.ch[3] < 0)
    {
            chassis_move.ch3_cal = 670 + chassis_move_mode->chassis_RC->rc.ch[3];
    }


    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[0]))
    {    
        // 飞镖YAW轴控制
        if (chassis_move_mode->chassis_RC->rc.ch[0] > 0)
        {
            //控制电机正反转
            HAL_GPIO_WritePin(PUSH_YAW_GPIO_Port, PUSH_YAW_Pin, GPIO_PIN_RESET);
            servo_speed_set(chassis_move.ch0_cal/2, 1);
        }
        if (chassis_move_mode->chassis_RC->rc.ch[0] < 0)
        {
            HAL_GPIO_WritePin(PUSH_YAW_GPIO_Port, PUSH_YAW_Pin, GPIO_PIN_SET);
            servo_speed_set(chassis_move.ch0_cal/2, 1);
        }


        //飞镖PITCH轴控制
        if (chassis_move_mode->chassis_RC->rc.ch[3] > 300)
        {
            //控制电机正反转
            HAL_GPIO_WritePin(PUSH_PITCH_GPIO_Port, PUSH_PITCH_Pin, GPIO_PIN_RESET);
            servo_speed_set(chassis_move.ch3_cal/2, 2);
        }
        if (chassis_move_mode->chassis_RC->rc.ch[3] < -300)
        {
            HAL_GPIO_WritePin(PUSH_PITCH_GPIO_Port, PUSH_PITCH_Pin, GPIO_PIN_SET);
            servo_speed_set(chassis_move.ch3_cal/2, 2);
        }
    }
    
}



