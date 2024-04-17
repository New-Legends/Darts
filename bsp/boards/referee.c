#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "bsp_usart.h"
#include "usart.h"
#include "fifo.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;
ext_dart_client_cmd_t dartclient;

ext_game_state_t game_state;//比赛状态数据 
ext_game_result_t game_result;//比赛结果数据
ext_game_robot_HP_t game_robot_HP_t;//机器人血量数据

ext_event_data_t field_event;//场地事件数据
ext_supply_projectile_action_t supply_projectile_action_t;//补给站动作标识
ext_supply_projectile_booking_t supply_projectile_booking_t;//请求补给站补弹数据
ext_referee_warning_t referee_warning_t;//裁判警告数据
ext_dart_remaining_time_t dart_remaining_time_t;//飞镖发射口倒计时


ext_game_robot_state_t robot_state;//机器人状态数据
ext_power_heat_data_t power_heat_data_t;//实时功率热量数据
ext_game_robot_pos_t game_robot_pos_t;//机器人位置数据
ext_buff_musk_t buff_musk_t;//机器人增益数据
aerial_robot_energy_t robot_energy_t;//空中机器人能量数据
ext_robot_hurt_t robot_hurt_t;//伤害状态数据
ext_shoot_data_t shoot_data_t;//实时设计数据
ext_bullet_remaining_t bullet_remaining_t;//子弹剩余发射数
ext_dart_client_cmd_t dart_client_t;//飞镖机器人客户端指令
ext_student_interactive_data_t student_interactive_data_t;//交互数据接收信息

uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024
fifo_s_t referee_fifo;
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
unpack_data_t referee_unpack_obj;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];


void Referee_init(){

    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
}

void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee_fifo, (char *)(usart6_buf[0]), this_time_rx_len);
            } 
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee_fifo, (char *)(usart6_buf[1]), this_time_rx_len);
            }
        }
    }


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&dart_remaining_time_t, 0, sizeof(ext_dart_remaining_time_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
    memset(&dart_client_t, 0, sizeof(ext_dart_client_cmd_t));
    

    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
    memset(&dartclient, 0, sizeof(ext_dart_client_cmd_t));



}
void Referee_unpack(){

    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo))
    {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;

        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;

        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;

        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;

        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;

        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case DART_REMAINING_TIME_CMD_ID:
        {
            memcpy(&dart_client_t, frame + index, sizeof(ext_dart_remaining_time_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;

        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;

        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;

        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;

        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;

        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;

        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;

        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;

        case DART_CLIENT_CMD_ID:
        {
            memcpy(&dart_client_t, frame + index, sizeof(ext_dart_client_cmd_t));
        }
        break;

        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;

        default:
        {
            break;
        }
    }
}



//返回机器人ID
uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}


//************************功率控制***********************************
//底盘输出功率,底盘功率缓存
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}


//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat)
{
    *id1_17mm_cooling_limit = robot_state.shooter_id1_17mm_cooling_limit;
    *id1_17mm_cooling_heat = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}


//17mm枪口枪口射速上限,17mm实时射速 默认ID1
void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed)
{
    *id1_17mm_speed_limit = robot_state.shooter_id1_17mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}

//17mm枪口热量冷却 默认ID1
void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate)
{   
    *id1_17mm_cooling_rate = robot_state.shooter_id1_17mm_cooling_rate;
}

//42mm枪口热量上限, 42mm枪口实时热量
void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat)
{
    *id1_42mm_cooling_limit = robot_state.shooter_id1_42mm_cooling_limit;
    *id1_42mm_cooling_heat = power_heat_data_t.shooter_id1_42mm_cooling_heat;
}

//42mm枪口枪口射速上限,42mm实时射速
void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, uint16_t *bullet_speed)
{
    *id1_42mm_speed_limit = robot_state.shooter_id1_42mm_speed_limit;
    *bullet_speed = shoot_data_t.bullet_speed;
}


//42mm枪口热量冷却
void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate)
{   
    *id1_42mm_cooling_rate = robot_state.shooter_id1_42mm_cooling_rate;
}


//当前血量
uint16_t get_remain_hp()
{
    return robot_state.remain_HP;

}

//是否被击打
bool_t if_hit()
{
    static uint16_t hp_detect_time = 0;    //血量检测间隔
    static uint16_t last_hp = 0;
  
    //初始化血量记录
    if (last_hp == 0)
        last_hp = robot_state.remain_HP;
    
    if (hp_detect_time++ > 300)
    {
        last_hp = robot_state.remain_HP;
        hp_detect_time = 0;
    }

    //受到高于10点的伤害,开始扭腰
    if(last_hp - robot_state.remain_HP >= 10)
        return TRUE;
    else
        return FALSE;
}




/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool_t Color;
bool_t is_red_or_blue(void)
{
	Judge_Self_ID = robot_state.robot_id;//读取当前机器人ID
	
	if(robot_state.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}
/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-0x10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}

/**
  * @brief  获取飞镖机器人客户端指令数据
  * @param  void
  * @retval 1 2 0
  */

uint8_t get_dart_client(void)
{
    return dart_client_t.dart_launch_opening_status;
}

/**
  * @brief  获取比赛状态数据:当前比赛阶段
  * @param  void
  * @retval 0 1 2 3 4 5 
  */
uint8_t get_game_progress(void)
{
    return game_state.game_progress;
}