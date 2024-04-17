#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"
#include "fifo.h"

#define RED   0
#define BLUE  1


enum errorList
{
  DBUS_TOE = 0,
  SHOOT_LEFT_FRIC_MOTOR_ID,
  SHOOT_RIGHT_FRIC_MOTOR_ID,
  SHOOT_TRIGGER_MOTOR_TOE,
  SHOOT_COVER_MOTOR_TOE,
  CHASSIS_MOTIVE_FR_MOTOR_TOE,
  CHASSIS_MOTIVE_FL_MOTOR_TOE,
  CHASSIS_MOTIVE_BL_MOTOR_TOE,
  CHASSIS_MOTIVE_BR_MOTOR_TOE,
  CHASSIS_RUDDER_FR_MOTOR_TOE,
  CHASSIS_RUDDER_FL_MOTOR_TOE,
  CHASSIS_RUDDER_BL_MOTOR_TOE,
  CHASSIS_RUDDER_BR_MOTOR_TOE,
  GIMBAL_YAW_MOTOR_TOE,
  GIMBAL_PITCH_MOTOR_TOE,
  BOARD_GYRO_TOE,
  BOARD_ACCEL_TOE,
  BOARD_MAG_TOE,
  RM_IMU_TOE,
  REFEREE_TOE,
  SUPER_CAP_TOE,
  OLED_TOE,
  VISION_TOE,
  BOARD_COM,
  ERROR_LIST_LENGHT,
};

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct //0002
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct//0x0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct //0101
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct//0x0104
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

typedef __packed struct//0x0105
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

typedef __packed struct //0x0201
{
  	uint8_t robot_id;                                     //本机器人id
	  uint8_t robot_level;                                  //机器人等级
	  uint16_t remain_HP;                                   //机器人剩余血量
	  uint16_t max_HP;                                      //机器人上限血量

    uint16_t shooter_id1_17mm_cooling_rate;               //机器人1号17mm枪口每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit;              //机器人1号17mm枪口热量上限
    uint16_t shooter_id1_17mm_speed_limit;                //机器人1号17mm 枪口上限速度 单位 m/s

    uint16_t shooter_id2_17mm_cooling_rate;               //机器人2号17mm枪口每秒冷却值
    uint16_t shooter_id2_17mm_cooling_limit;              //机器人2号17mm枪口热量上限
    uint16_t shooter_id2_17mm_speed_limit;                //机器人2号17mm 枪口上限速度 单位 m/s

    uint16_t shooter_id1_42mm_cooling_rate;               //机器人42mm枪口每秒冷却值
    uint16_t shooter_id1_42mm_cooling_limit;              //机器人42mm枪口热量上限
    uint16_t shooter_id1_42mm_speed_limit;                //机器人42mm枪口上限速度 单位 m/s

    uint16_t chassis_power_limit;                         //机器人底盘功率限制上限
    uint8_t mains_power_gimbal_output : 1;                //电管gimbal口输出
    uint8_t mains_power_chassis_output : 1;               //电管chassis口输出
    uint8_t mains_power_shooter_output : 1;               //电管shooter口输出

} ext_game_robot_state_t;         //发送频率：10Hz

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;        //底盘输出电压
    uint16_t chassis_current;     //底盘输出电流
    fp32 chassis_power;          //底盘输出功率
    uint16_t chassis_power_buffer; //底盘功率缓冲
    uint16_t shooter_id1_17mm_cooling_heat;  //1号17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat;  //2号17mm枪口热量
    uint16_t shooter_id1_42mm_cooling_heat;  //42mm 枪口热量
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    fp32 x;
    fp32 y;
    fp32 z;
    fp32 yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;  //子弹类型: 1：17mm弹丸 2：42mm弹丸
    uint8_t shooter_id;  //发射机构ID： 1：1号17mm发射机构 2：2号17mm发射机构 3：42mm 发射机构
    uint8_t bullet_freq; //子弹射频 单位 Hz
    fp32 bullet_speed;  //子弹射速 单位 m/s
} ext_shoot_data_t;

typedef __packed struct//0x0208
{
    uint16_t bullet_remaining_num_17mm; //17mm子弹剩余发射数目 
    uint16_t bullet_remaining_num_42mm; // 42mm子弹剩余发射数目
    uint16_t coin_remaining_num;        //剩余金币数量
} ext_bullet_remaining_t;

typedef __packed struct//0x0209
{
 uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct//0x020A
{
 uint8_t dart_launch_opening_status;//飞镖发射口状态1：关闭；2正在开启或关闭；0：已经开启
 uint8_t dart_attack_target;//打击目标
 uint16_t target_change_time;//切换目标比赛剩余时间
 uint16_t operate_launch_cmd_time;//最近一次操作手确定发射指令时的比赛剩余时间
}ext_dart_client_cmd_t;

typedef __packed struct //0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
    fp32 data1;
    fp32 data2;
    fp32 data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;


extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern ext_dart_client_cmd_t dart_client_t;

//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat);
//17mm枪口枪口射速上限,17mm实时射速 默认ID1
void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed);
//17mm枪口热量冷却 默认ID1
void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate);


//42mm枪口热量上限, 42mm枪口实时热量
void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat);
//42mm枪口枪口射速上限,42mm实时射速
void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, uint16_t *bullet_speed);
//42mm枪口热量冷却
void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate);;


extern void determine_ID(void);
extern bool_t is_red_or_blue(void);
extern uint8_t get_dart_client(void);
extern bool_t if_hit(void);
extern uint8_t get_game_progress(void);
#endif
