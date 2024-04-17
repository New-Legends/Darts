/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_receive.h"
#include "user_lib.h"

//是否开启读裁判自动射击
#define AUTO_SHOOT 0

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME 201 
#define SHOOT_CONTROL_TIME 1

#define TRIGGER_CCW 1 //拨盘顺时针
#define TRIGGER_CW -1 //拨盘逆时针


#define SHOOT_TRIGGER_DIRECTION TRIGGER_CCW

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
#define RC_MODE_CHANNEL_RIGHT       0




//开启摩擦轮的斜坡
#define SHOOT_FRIC_ADD_VALUE    0.1f

//射击摩擦轮激光打开 关闭
#define SHOOT_KEYBOARD           KEY_PRESSED_OFFSET_G

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//摩擦轮开启按键延时
#define KEY_FRIC_LONG_TIME             200


//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191


//拨盘电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED         0.000415809748903494517209f
#define MOTOR_ECD_TO_ANGLE         0.000039940741761990380111011191762043f * SHOOT_TRIGGER_DIRECTION
#define FULL_COUNT                 19


//推弹电机rmp 变化成 旋转速度的比例
#define PULL_MOTOR_RPM_TO_SPEED         0.00290888208665721596153948461415f
#define PULL_MOTOR_ECD_TO_ANGLE         0.000021305288720633905968306772076277f * SHOOT_TRIGGER_DIRECTION
#define PULL_FULL_COUNT                 18


//拨弹速度
#define TRIGGER_SPEED               10.0f * SHOOT_TRIGGER_DIRECTION   //10
#define CONTINUE_TRIGGER_SPEED      15.0f * SHOOT_TRIGGER_DIRECTION  //15
#define READY_TRIGGER_SPEED         5.0f * SHOOT_TRIGGER_DIRECTION //5


//推弹速度
#define PULL_SPEED               10.0f * SHOOT_TRIGGER_DIRECTION   //10
#define CONTINUE_PULL_SPEED      15.0f * SHOOT_TRIGGER_DIRECTION  //15
#define READY_PULL_SPEED         5.0f * SHOOT_TRIGGER_DIRECTION //5


//推弹卡弹时间 以及反转时间
#define BLOCK_PULL_SPEED            1.0f
#define BLOCK_TIME                  1000
#define REVERSE_TIME                700
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f



//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        10.0f  //800 //10.0f
#define TRIGGER_ANGLE_PID_KI        0.0f   //0.5  //0.0f
#define TRIGGER_ANGLE_PID_KD        100.0f  //0.0  //100.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


//推弹轮电机PID

#define PULL_ANGLE_PID_KP        1000.0f  //800
#define PULL_ANGLE_PID_KI        0.2f  //0.5
#define PULL_ANGLE_PID_KD        0.0f

#define PULL_BULLET_PID_MAX_OUT  10000.0f
#define PULL_BULLET_PID_MAX_IOUT 9000.0f

#define PULL_READY_PID_MAX_OUT   10000.0f
#define PULL_READY_PID_MAX_IOUT  7000.0f


//摩擦轮电机rmp 变化成 旋转速度的比例
#define FRIC_RPM_TO_SPEED           0.000415809748903494517209f*5



//摩擦轮电机PID
#define FRIC_SPEED_PID_KP        1800.0f
#define FRIC_SPEED_PID_KI        0.5f
#define FRIC_SPEED_PID_KD        2.0f

#define FRIC_PID_MAX_OUT  8000.0f
#define FRIC_PID_MAX_IOUT 200.0f


#define FRIC_MAX_SPEED_RMP 4000.0f
#define FRIC_REQUIRE_SPEED_RMP 500.0f

#define SHOOT_HEAT_REMAIN_VALUE     80


//拨盘格数
#define TRIGGER_GRID_NUM 4   
#define TRIGGER_ONCE 2.0*PI/TRIGGER_GRID_NUM

#define PULL_GRID_NUM 4   
#define PULL_ONCE 2.0*PI/PULL_GRID_NUM

#define L1 0
#define L2 1
#define L3 2

#define R1 3
#define R2 4
#define R3 5

#define FRIC_REFEREE_PARA  0.8f            //射速裁判规定数值转实际输入
#define GRIGGER_SPEED_TO_RADIO  0.8f      //射频裁判规定数值转实际输入

typedef enum
{
    SHOOT_STOP = 0,    //停止发射结构
    SHOOT_READY_FRIC,  //摩擦轮准备中
    SHOOT_READY_BULLET,//拨盘准备中,摩擦轮已达到转速
    SHOOT_READY,       //整个发射机构准备完成
    SHOOT_BULLET,      //单发
    SHOOT_CONTINUE_BULLET,//连发
    SHOOT_DONE,      
} shoot_mode_e;

typedef enum
{
    TRIGGER_SPIN = 0, //拨弹轮转动
    TRIGGER_STOP,  //拨弹轮停止
}trigger_mode_e;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

  fp32 max_speed;  //摩擦轮旋转最大速度
  fp32 min_speed;  //摩擦轮旋转最小速度
  fp32 require_speed; //允许拨盘开启的最低速度

} fric_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;
    trigger_mode_e trigger_mode;
    const RC_ctrl_t *shoot_rc;
    uint16_t shoot_last_key_v;

    //拨弹电机数据
    const motor_measure_t *trigger_motor_measure;
    pid_type_def trigger_motor_pid;    
    fp32 trigger_speed;
    fp32 trigger_speed_set;
    fp32 trigger_angle;
    fp32 trigger_set_angle;    
    fp32 trigger_NB_angel;
    fp32 trigger_NB_last_angel;
    int16_t trigger_given_current;
    int8_t trigger_ecd_count;

    //推弹电机数据
    const motor_measure_t *pull_motor_measure;
    pid_type_def pull_motor_pid;
    fp32 pull_speed;
    fp32 pull_speed_set;
    fp32 pull_angle;
    fp32 pull_set_angle;
    int16_t pull_given_current;
    int8_t pull_ecd_count;


    //摩擦轮电机数据
    fric_motor_t fric_motor[6]; 
    pid_type_def fric_speed_pid[6];

    //摩擦轮电机 弹仓舵机 限位开关 状态
    bool_t fric_status;
    bool_t magazine_status;
    bool_t limit_switch_status;
  
    //鼠标状态
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;
    bool_t pull_flag;
    bool_t pull_gpio_flag;
    bool_t first_pull_flag;
    bool_t shoot_would;
    uint8_t step_time;
    uint16_t pull_time;
    uint8_t half_angle;

    const motor_measure_t *motor_state[8];
 
}shoot_control_t;



extern shoot_control_t shoot_control;          //射击数据

//摩擦轮按键控制
#define KEY_FRIC (shoot_control.shoot_rc->key.v  & KEY_PRESSED_OFFSET_G) && !(shoot_control.shoot_last_key_v & KEY_PRESSED_OFFSET_G) 


//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_task(void const *pvParameters);
extern void shoot_init(void);
extern void shoot_set_control(void);
extern void trigger_set_control(void);
extern bool_t shoot_cmd_to_gimbal_stop(void);
extern void autoshoot(void);
#endif
