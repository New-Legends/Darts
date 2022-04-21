#include "bsp_servo_pwm.h"
#include "main.h"

int8_t cnt;

void servo_pwm_set(uint16_t speed, uint8_t i)
{
    switch(i)
    {
        case 1:
        {
            HAL_GPIO_WritePin(DIR_YAW_GPIO_Port, DIR_YAW_Pin, GPIO_PIN_SET);
            delay_us(speed);
            HAL_GPIO_WritePin(DIR_YAW_GPIO_Port, DIR_YAW_Pin, GPIO_PIN_RESET);
            delay_us(speed);
        }break;
        case 2:
        {
            HAL_GPIO_WritePin(DIR_PITCH_GPIO_Port, DIR_PITCH_Pin, GPIO_PIN_SET);
            delay_us(speed);
            HAL_GPIO_WritePin(DIR_PITCH_GPIO_Port, DIR_PITCH_Pin, GPIO_PIN_RESET);
            delay_us(speed);
        }break;
    }
}
