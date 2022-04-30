#include "bsp_servo_pwm.h"
#include "main.h"

int8_t cnt;

void servo_speed_set(uint16_t speed, uint8_t i)
{
    switch(i)
    {
        case 1://YAW轴步进
        {
            HAL_GPIO_WritePin(DIR_YAW_GPIO_Port, DIR_YAW_Pin, GPIO_PIN_SET);
            delay_us(speed);
            HAL_GPIO_WritePin(DIR_YAW_GPIO_Port, DIR_YAW_Pin, GPIO_PIN_RESET);
            delay_us(speed);
        }break;
        case 2://PITCH轴步进
        {
            HAL_GPIO_WritePin(DIR_PITCH_GPIO_Port, DIR_PITCH_Pin, GPIO_PIN_SET);
            delay_us(speed);
            HAL_GPIO_WritePin(DIR_PITCH_GPIO_Port, DIR_PITCH_Pin, GPIO_PIN_RESET);
            delay_us(speed);
        }break;
        case 3://拨弹步进
        {
            //每次前进90度（每高低电平翻转一次前进0.9度）
            for(cnt = 0;cnt < 100;cnt++) 
            {
                HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
                delay_us(speed);
                HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
                delay_us(speed);
            }
        }break;
    }
}
