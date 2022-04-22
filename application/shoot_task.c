/**
  ****************************(C) COPYR1 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYR1 2019 DJI****************************
  */

#include "shoot_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"

#include "CAN_receive.h"
#include "pid.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

//通过读取裁判数据,直接修改射速和射频等级
//射速等级  摩擦电机
fp32 shoot_fric_grade[4] = {0, 15 * FRIC_REFEREE_PARA, 18 * FRIC_REFEREE_PARA, 30 * FRIC_REFEREE_PARA};

//射频等级 拨弹电机
fp32 shoot_grigger_grade[6] = {0, 5.0f * GRIGGER_SPEED_TO_RADIO, 10.0f * GRIGGER_SPEED_TO_RADIO, 15.0f * GRIGGER_SPEED_TO_RADIO, 28.0f * GRIGGER_SPEED_TO_RADIO, 40.0f * GRIGGER_SPEED_TO_RADIO};

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

/**
  * @brief          推杆控制，完成一次发射
  * @param[in]      void
  * @retval         void
  */

shoot_control_t shoot_control; //射击数据

/**
  * @brief          射击任务，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_task(void const *pvParameters)
{
    //初始化延时
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    //发射结构初始化
    shoot_init();

    while (1)
    {
        shoot_set_mode();        //设置状态机
        shoot_feedback_update(); //更新数据
        shoot_set_control();     //射击任务控制循环
        //CAN发送
        CAN_cmd_left_shoot(shoot_control.fric_motor[L1].give_current, shoot_control.fric_motor[L2].give_current, shoot_control.fric_motor[L3].give_current, shoot_control.given_current);
        CAN_cmd_right_shoot(shoot_control.fric_motor[R1].give_current, shoot_control.fric_motor[R2].give_current, shoot_control.fric_motor[R3].give_current, 0);
        //CAN_cmd_left_shoot(0, 0, shoot_control.fric_motor[L3].give_current, 0);
        vTaskDelay(SHOOT_CONTROL_TIME);
    }
}

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const fp32 Fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FRIC_SPEED_PID_KI, FRIC_SPEED_PID_KD};

    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();

    for (int i = 0; i < 7; i++)
    {
        shoot_control.motor_state[i] = get_fric_motor_measure_point(i);
    }
    //电机指针 拨弹 摩擦轮
    shoot_control.trigger_motor_measure = shoot_control.motor_state[3];

    shoot_control.fric_motor[L1].fric_motor_measure = shoot_control.motor_state[0];
    shoot_control.fric_motor[L2].fric_motor_measure = shoot_control.motor_state[1];
    shoot_control.fric_motor[L3].fric_motor_measure = shoot_control.motor_state[2];

    shoot_control.fric_motor[R1].fric_motor_measure = shoot_control.motor_state[4];
    shoot_control.fric_motor[R2].fric_motor_measure = shoot_control.motor_state[5];
    shoot_control.fric_motor[R3].fric_motor_measure = shoot_control.motor_state[6];
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[L1], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[R1], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[L2], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[R2], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[L3], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    PID_init(&shoot_control.fric_speed_pid[R3], PID_POSITION, Fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);

    //设置最大 最小值  左摩擦轮顺时针转 右摩擦轮逆时针转
    shoot_control.fric_motor[L1].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L1].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L1].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[R1].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R1].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R1].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[L2].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L2].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L2].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[R2].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R2].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R2].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[L3].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L3].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[L3].require_speed = -FRIC_REQUIRE_SPEED_RMP;

    shoot_control.fric_motor[R3].max_speed = FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R3].min_speed = -FRIC_MAX_SPEED_RMP;
    shoot_control.fric_motor[R3].require_speed = -FRIC_REQUIRE_SPEED_RMP;
    //摩擦轮,弹仓舵机,限位舵机状态
    shoot_control.fric_status = FALSE;
    shoot_control.magazine_status = FALSE;
    shoot_control.limit_switch_status = FALSE;

    //记录上一次按键值
    shoot_control.shoot_last_key_v = 0;

    //更新数据
    shoot_feedback_update();

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    //更新摩擦轮电机速度
    shoot_control.fric_motor[L1].speed = shoot_control.fric_motor[L1].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fric_motor[R1].speed = shoot_control.fric_motor[R1].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    shoot_control.fric_motor[L2].speed = shoot_control.fric_motor[L2].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fric_motor[R2].speed = shoot_control.fric_motor[R2].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;

    shoot_control.fric_motor[L3].speed = shoot_control.fric_motor[L3].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    shoot_control.fric_motor[R3].speed = shoot_control.fric_motor[R3].fric_motor_measure->speed_rpm * FRIC_RPM_TO_SPEED;
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.trigger_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
void shoot_set_control(void)
{
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = shoot_grigger_grade[2] * SHOOT_TRIGGER_DIRECTION;
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        shoot_control.fric_motor[L1].give_current = 0.0f;
        shoot_control.fric_motor[R1].give_current = 0.0f;

        shoot_control.fric_motor[L2].give_current = 0.0f;
        shoot_control.fric_motor[R2].give_current = 0.0f;

        shoot_control.fric_motor[L3].give_current = 0.0f;
        shoot_control.fric_motor[R3].give_current = 0.0f;
        shoot_control.fric_status = FALSE;
    }
    else
    {
        shoot_laser_on(); //激光开启
        //设置摩擦轮转速
        shoot_control.fric_motor[L1].speed_set = shoot_fric_grade[3];
        shoot_control.fric_motor[R1].speed_set = -shoot_fric_grade[3];

        shoot_control.fric_motor[L2].speed_set = shoot_fric_grade[3];
        shoot_control.fric_motor[R2].speed_set = -shoot_fric_grade[3];

        shoot_control.fric_motor[L3].speed_set = shoot_fric_grade[3];
        shoot_control.fric_motor[R3].speed_set = -shoot_fric_grade[3];
        //        //连发模式 控制17mm发射机构射速和热量控制
        //        if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        //            shoot_id1_17mm_speed_and_cooling_control(&shoot_control);

        if (shoot_control.shoot_mode == SHOOT_READY_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
            trigger_motor_turn_back(); //将设置的拨盘旋转角度,转化为速度,且防止卡弹

        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        //计算摩擦轮电机PID
        PID_calc(&shoot_control.fric_speed_pid[L1], shoot_control.fric_motor[L1].speed, shoot_control.fric_motor[L1].speed_set);
        PID_calc(&shoot_control.fric_speed_pid[R1], shoot_control.fric_motor[R1].speed, shoot_control.fric_motor[R1].speed_set);

        PID_calc(&shoot_control.fric_speed_pid[L2], shoot_control.fric_motor[L2].speed, shoot_control.fric_motor[L2].speed_set);
        PID_calc(&shoot_control.fric_speed_pid[R2], shoot_control.fric_motor[R2].speed, shoot_control.fric_motor[R2].speed_set);

        PID_calc(&shoot_control.fric_speed_pid[L3], shoot_control.fric_motor[L3].speed, shoot_control.fric_motor[L3].speed_set);
        PID_calc(&shoot_control.fric_speed_pid[R3], shoot_control.fric_motor[R3].speed, shoot_control.fric_motor[R3].speed_set);
        //确保摩擦轮未达到最低转速不会转动拨盘
        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }

        //设置发送电流
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        shoot_control.fric_motor[L1].give_current = shoot_control.fric_speed_pid[L1].out;
        shoot_control.fric_motor[R1].give_current = shoot_control.fric_speed_pid[R1].out;

        shoot_control.fric_motor[L2].give_current = shoot_control.fric_speed_pid[L2].out;
        shoot_control.fric_motor[R2].give_current = shoot_control.fric_speed_pid[R2].out;

        shoot_control.fric_motor[L3].give_current = shoot_control.fric_speed_pid[L3].out;
        shoot_control.fric_motor[R3].give_current = shoot_control.fric_speed_pid[R3].out;
    }
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP; //记录上一次遥控器按键值
    if (switch_is_mid((shoot_control.shoot_rc->rc.s[RC_MODE_CHANNEL_RIGHT])))
    {

        //上拨判断， 一次开启，再次关闭
        if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
        {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
        else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
        {
            shoot_control.shoot_mode = SHOOT_STOP;
        }

        shoot_control.shoot_last_key_v = shoot_control.shoot_rc->key.v;

        //摩擦轮速度达到一定值,才可开启拨盘  为了便于测试,这里至少需要一个摩擦轮电机达到拨盘启动要求就可以开启拨盘
        // && (abs(shoot_control.fric_motor[L1].fric_motor_measure->speed_rpm) > abs(shoot_control.fric_motor[L1].require_speed) || abs(shoot_control.fric_motor[R1].fric_motor_measure->speed_rpm) > abs(shoot_control.fric_motor[R1].require_speed))
        if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
        {
            shoot_control.fric_status = TRUE;
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
        else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY;
        }
        else if (shoot_control.shoot_mode == SHOOT_READY)
        {
            //下拨一次或者鼠标按下一次，进入射击状态
            if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)))
            {
                shoot_control.shoot_mode = SHOOT_BULLET;
            }
        }

        last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
    }
}

static void trigger_motor_turn_back(void)
{
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //每次拨动的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ONCE);
        shoot_control.move_flag = 1;
    }
    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = shoot_grigger_grade[1] * SHOOT_TRIGGER_DIRECTION;
        trigger_motor_turn_back();
    }
    else
    {
        //TODO:这里需要添加推杆控制
        shoot_control.move_flag = 0;
        shoot_control.shoot_mode = SHOOT_READY;
    }
}
