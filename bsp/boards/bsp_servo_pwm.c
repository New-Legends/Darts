#include "bsp_servo_pwm.h"
#include "main.h"

/**
  * @brief          步进电机高低电平函数
  * @param[in]      speed、i
  * @retval         返回IO控制值
  */
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
            HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
            delay_ms(speed);
            HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
            delay_ms(speed);
        }break;
        case 4:
        {
            HAL_GPIO_WritePin(PULL_PUL_GPIO_Port, PULL_PUL_Pin, GPIO_PIN_SET);
            vTaskDelay(speed);
            HAL_GPIO_WritePin(PULL_PUL_GPIO_Port, PULL_PUL_Pin, GPIO_PIN_RESET);
            vTaskDelay(speed);
        }
    }
}
