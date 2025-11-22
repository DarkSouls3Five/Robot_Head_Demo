#include "referee_task.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "struct_typedef.h"
#include "can.h"
#include "main.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "usart.h"
#include "shoot_task.h"
#include "string.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "gimbal_task.h"



void init_referee_struct_data(void);
void referee_unpack_fifo_data(void);
void referee_data_solve(uint8_t *frame);
void referee_data_solve(uint8_t *frame);
void send_multi_graphic(void);
void referee_send_multi_graphic(ext_id_t target_id, ext_client_custom_graphic_seven_t* graphic_draw);
void send_double_text(void);
void send_capvol_graphic(int capvols);
void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw);
void send_rub_graphic(char graphname[3], int x, int y, int color, int type);
void send_spinning_graphic(char name[3], int x, int y, int color, int type);
void send_autoaim_state(void);

bool capdraw = true;
bool auto_state = true;

ext_id_t MY_CLIENT_ID = clientid_red_hero;
int MY_ROBOT_ID = robotid_red_hero;

uint16_t cmd_id;  // 数据包ID

//裁判信息相关结构体
ext_game_status_t                           ext_game_status;//比赛状态数据（0x0001）
ext_game_result_t                           ext_game_result;//比赛结果数据 (0x0002)
ext_game_robot_HP_t                         ext_game_robot_HP;//机器人存存活数据（0x0003）
ext_ICRA_buff_debuff_zone_status_t          ext_ICRA_buff_debuff_zone_status; //人工智能挑战赛加成与惩罚区状态(0x0005)

ext_event_data_t                           	ext_event_data;//场地时事件数据（0x0101）
ext_supply_projectile_action_t             	ext_supply_projectile_action;//补给站动作标识（0x0102）
ext_referee_warning_t                      	ext_referee_warning;//裁判警告信息(0x0104)
ext_dart_remaining_time_t                  	ext_dart_remaining_time;//飞镖发射口倒计时(0x0105)

ext_game_robot_status_t                    	ext_game_robot_status;//比赛机器人状态(0x0201)
ext_power_heat_data_t                      	ext_power_heat_data;//实时功率热量数据（0x0202）
ext_game_robot_pos_t                       	ext_game_robot_pos;//机器人位置（0x0203）
ext_buff_t                                 	ext_buff;//机器人增益（0x0204）
aerial_robot_energy_t                      	ext_aerial_robot_energy;//空中机器人能量状态（0x0205）
ext_robot_hurt_t                           	ext_robot_hurt;//伤害状态（0x0206）
ext_shoot_data_t                           	ext_shoot_data;//实时射击信息（0x0207）
ext_bullet_remaining_t                     	ext_bullet_remaining;//子弹剩余发射数(0x0208)
ext_rfid_status_t                          	ext_rfid_status;//机器人RFID状态(0x0209)
ext_dart_client_cmd_t          				ext_dart_client_cmd;//飞镖机器人客户端指令数据(0x020A)

//-------------0x0301部分开始-------------------
ext_student_interactive_header_data_t      	ext_student_interactive_header_data;//交互数据接收信息（0x0301）
robot_interactive_data_t                   	robot_interactive_data;//机器人间交互数据，内容 ID:0x0200~0x02FF
ext_client_custom_graphic_delete_t         	ext_client_custom_graphic_delete;//客户端删除图形，内容 ID:0x0100;
graphic_data_struct_t                      	graphic_data_struct;//图形数据
ext_client_custom_graphic_single_t         	ext_client_custom_graphic_single;//客户端绘制一个图形
ext_client_custom_graphic_double_t         	ext_client_custom_graphic_double;//客户端绘制两个图形
ext_client_custom_graphic_five_t            ext_client_custom_graphic_five;//客户端绘制五个图形
ext_client_custom_graphic_seven_t           ext_client_custom_graphic_seven;//客户端绘制七个图形
ext_client_custom_character_t               ext_client_custom_character;//客户端绘制字符
//-------------0x0301部分结束-------------------

ext_robot_command_t                         ext_robot_command; //小地图下发信息标识(0x0303)/*发送频率：触发时发送.*/
ext_client_map_command_t                    ext_client_map_command; //小地图接收信息标识(0x0305)

frame_header_struct_t ext_referee_receive_header;
frame_header_struct_t ext_referee_send_header;

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

uint8_t USART6_dma[80];		//DMA接收数据
uint8_t Personal_Data[128];	//DMA发送数据


void Referee_Task(void const * argument){
    
    init_referee_struct_data();//为用于盛装数据的各个结构体分配空间。
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);//初始化用于交换裁判系统信息的队列缓存
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);//通过6号串口收发来自电源管理模块的数据，送至主控模块通过WIFI与服务器通信。
	//USART6通过串口中断接收数据。
	
	uint16_t UI_PushUp_Counter = 261;
	/* 裁判系统初始化 */
	vTaskDelay(300);

	while(1){
		referee_unpack_fifo_data();//接收数据

	    vTaskDelay(4);

		UI_PushUp_Counter++;
				
		if(UI_PushUp_Counter>=1000){
			UI_PushUp_Counter = 10;
		}
			
		if(UI_PushUp_Counter % 500 == 0){
		    send_multi_graphic();
		    capdraw = true;
		    auto_state = true;
		    continue;
		}

		if(UI_PushUp_Counter % 600 == 0){
			//send_text_graphic("009");
			send_double_text();
			continue;
		}
        
		//自瞄数据
		if(UI_PushUp_Counter % 23 ==0){
				//REST ENERGY
			send_autoaim_state();
			continue;
		}

        //摩擦轮是否开启
		if(UI_PushUp_Counter % 36 ==0){
			//STATE OF RUB
			if (shoot_data.fric_state == FRIC_ON)
			{
				send_rub_graphic("004", 1800,500,2,1);//green
			}
			else
			{
				send_rub_graphic("004", 1800,500,2,3);
			}
			continue;
		}
	}
}



void init_referee_struct_data(void)
{
    memset(&ext_referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&ext_game_status, 0, sizeof(ext_game_status_t));
    memset(&ext_game_result, 0, sizeof(ext_game_result_t));
    memset(&ext_game_robot_HP, 0, sizeof(ext_game_robot_HP_t));

    memset(&ext_event_data, 0, sizeof(ext_event_data_t));
    memset(&ext_supply_projectile_action, 0, sizeof(ext_supply_projectile_action_t));
    memset(&ext_referee_warning, 0, sizeof(ext_referee_warning_t));
		memset(&ext_dart_remaining_time,             0, sizeof(ext_dart_remaining_time));

    memset(&ext_game_robot_status, 0, sizeof(ext_game_robot_status_t));
    memset(&ext_power_heat_data, 0, sizeof(ext_power_heat_data_t));
    memset(&ext_game_robot_pos, 0, sizeof(ext_game_robot_pos_t));
    memset(&ext_buff, 0, sizeof(ext_buff));
    memset(&ext_aerial_robot_energy, 0, sizeof(aerial_robot_energy));
    memset(&ext_robot_hurt, 0, sizeof(ext_robot_hurt_t));
    memset(&ext_shoot_data, 0, sizeof(ext_shoot_data_t));
    memset(&ext_bullet_remaining, 0, sizeof(ext_bullet_remaining_t));
		memset(&ext_rfid_status,                     0, sizeof(ext_rfid_status));
		memset(&ext_dart_client_cmd,                 0, sizeof(ext_dart_client_cmd));

    memset(&ext_student_interactive_header_data, 0, sizeof(ext_student_interactive_header_data_t));
		memset(&robot_interactive_data,          0, sizeof(robot_interactive_data));
		memset(&ext_robot_command,                   0, sizeof(ext_robot_command));
		memset(&ext_client_map_command,              0, sizeof(ext_client_map_command));
}


void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
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

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&ext_referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&ext_game_status, frame + index, sizeof(ext_game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&ext_game_result, frame + index, sizeof(ext_game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&ext_game_robot_HP, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
        case ICRA_BUFF_DEBUFF_ZONE_STATUS_CMD_ID:
        {
            memcpy(&ext_ICRA_buff_debuff_zone_status, frame + index, sizeof(ext_ICRA_buff_debuff_zone_status_t));
        }
        break;

        case EVENTS_DATA_CMD_ID:
        {
            memcpy(&ext_event_data, frame + index, sizeof(ext_event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&ext_supply_projectile_action, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;

        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&ext_referee_warning, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&ext_game_robot_status, frame + index, sizeof(ext_game_robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&ext_power_heat_data, frame + index, sizeof(ext_power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&ext_game_robot_pos, frame + index, sizeof(ext_game_robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&ext_buff, frame + index, sizeof(ext_buff_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&ext_aerial_robot_energy, frame + index, sizeof(aerial_robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&ext_robot_hurt, frame + index, sizeof(ext_robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&ext_shoot_data, frame + index, sizeof(ext_shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&ext_bullet_remaining, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&ext_rfid_status, frame + index, sizeof(ext_rfid_status_t));
        }
        break;
		case DART_CLIENT_CMD_CMD_ID:
        {
            memcpy(&ext_dart_client_cmd, frame + index, sizeof( ext_dart_client_cmd_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&ext_student_interactive_header_data, frame + index, sizeof(ext_student_interactive_header_data_t));
        }
        break;
        case ROBOT_COMMAND_CMD_ID://0x0303客户端下发信息
        {
            memcpy(&ext_robot_command, frame + index, sizeof(ext_robot_command_t));
        }
        break;
        default:
        {
            break;
        }
    }
}


void send_multi_graphic(void)//向客户端发送七个图形的数据。每个图形包含起点终点、操作类型、颜色、线宽等属性。
{

	ext_client_custom_graphic_seven_t graphic_draw;
	switch(ext_game_robot_status.robot_id){
				case robotid_red_hero:{
						MY_CLIENT_ID = clientid_red_hero;
						MY_ROBOT_ID = robotid_red_hero;	
					  break;
				}
				case robotid_red_infantry_1:{
						MY_CLIENT_ID = clientid_red_infantry_1;
						MY_ROBOT_ID = robotid_red_infantry_1;	
					  break;
				}
				case robotid_red_infantry_2:{
						MY_CLIENT_ID = clientid_red_infantry_2;
						MY_ROBOT_ID = robotid_red_infantry_2;	
					  break;					
				}
				case robotid_red_infantry_3:{
						MY_CLIENT_ID = clientid_red_infantry_3;
						MY_ROBOT_ID = robotid_red_infantry_3;
					  break;					
				}
			
				case robotid_blue_hero:{
						MY_CLIENT_ID = clientid_blue_hero;
						MY_ROBOT_ID = robotid_blue_hero;	
					  break;
				}
				case robotid_blue_infantry_1:{
						MY_CLIENT_ID = clientid_blue_infantry_1;
						MY_ROBOT_ID = robotid_blue_infantry_1;	
					  break;
				}
				case robotid_blue_infantry_2:{
						MY_CLIENT_ID = clientid_blue_infantry_2;
						MY_ROBOT_ID = robotid_blue_infantry_2;
					  break;					
				}
				case robotid_blue_infantry_3:{
						MY_CLIENT_ID = clientid_blue_infantry_3;
						MY_ROBOT_ID = robotid_blue_infantry_3;
					  break;					
				}
			}
	
	//graph 1：一条线宽为3的黄色直线。起点为（960，200）终点为（960，540）
	
	//graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	//if (first_multi_draw)
	//{//第一次画则设置模式为新增
	//	first_multi_draw = false;
		graphic_draw.graphic_data_struct[0].operate_tpye = 1;
		graphic_draw.graphic_data_struct[1].operate_tpye = 1;
		graphic_draw.graphic_data_struct[2].operate_tpye = 1;
		graphic_draw.graphic_data_struct[3].operate_tpye = 1;
		graphic_draw.graphic_data_struct[4].operate_tpye = 1;
		graphic_draw.graphic_data_struct[5].operate_tpye = 1;
		graphic_draw.graphic_data_struct[6].operate_tpye = 1;
//	}

/*	else
	{//不是第一次画则设置模式为修改
		graphic_draw.graphic_data_struct[0].operate_tpye = 2;
		graphic_draw.graphic_data_struct[1].operate_tpye = 2;
		graphic_draw.graphic_data_struct[2].operate_tpye = 2;
		graphic_draw.graphic_data_struct[3].operate_tpye = 2;
		graphic_draw.graphic_data_struct[4].operate_tpye = 2;
		graphic_draw.graphic_data_struct[5].operate_tpye = 2;
		graphic_draw.graphic_data_struct[6].operate_tpye = 2;
	}
	*/
	char name1[3] = "701";
	graphic_draw.graphic_data_struct[0].graphic_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].graphic_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].graphic_name[2] = name1[2];
	graphic_draw.graphic_data_struct[0].graphic_tpye = 1;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 8;
	graphic_draw.graphic_data_struct[0].width = 1;
	//graphic_draw.graphic_data_struct[0].start_angle=10;
	//graphic_draw.graphic_data_struct[0].end_angle=10;
	graphic_draw.graphic_data_struct[0].start_x = 760;
	graphic_draw.graphic_data_struct[0].start_y = 80;
	graphic_draw.graphic_data_struct[0].end_x = 1162;
	graphic_draw.graphic_data_struct[0].end_y = 60;
	//graphic_draw.graphic_data_struct[0].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[0].graphic_name, (uint8_t*)name1, strlen(name1));
	
	
	//graph 2：一条线宽为3的黄色直线，起点为（850，540），终点为（1070，540）
	char name2[3] = "202";
	graphic_draw.graphic_data_struct[1].graphic_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].graphic_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].graphic_name[2] = name2[2];
	graphic_draw.graphic_data_struct[1].graphic_tpye = 1;
	//graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width = 2;
	//graphic_draw.graphic_data_struct[1].start_angle=10;
	//graphic_draw.graphic_data_struct[1].end_angle=10;
	graphic_draw.graphic_data_struct[1].start_x = 930;
	graphic_draw.graphic_data_struct[1].start_y = 540;
	graphic_draw.graphic_data_struct[1].end_x = 990;
	graphic_draw.graphic_data_struct[1].end_y = 540;
	//graphic_draw.graphic_data_struct[1].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[1].graphic_name, (uint8_t*)name2, strlen(name2));
	
	
	//graph 3：也是线宽为3的黄色直线，起点为（860，520），终点为（1060，520）
	char name3[3] = "203";
	graphic_draw.graphic_data_struct[2].graphic_name[0] = name3[0];
	graphic_draw.graphic_data_struct[2].graphic_name[1] = name3[1];
	graphic_draw.graphic_data_struct[2].graphic_name[2] = name3[2];
	graphic_draw.graphic_data_struct[2].graphic_tpye = 1;
	//graphic_draw.graphic_data_struct[2].operate_tpye = 1;
	graphic_draw.graphic_data_struct[2].layer = 0;
	graphic_draw.graphic_data_struct[2].color = 2;
	graphic_draw.graphic_data_struct[2].width = 2;
	//graphic_draw.graphic_data_struct[2].start_angle=10;
	//graphic_draw.graphic_data_struct[2].end_angle=10;
	graphic_draw.graphic_data_struct[2].start_x = 930;
	graphic_draw.graphic_data_struct[2].start_y = 440;
	graphic_draw.graphic_data_struct[2].end_x = 990;
	graphic_draw.graphic_data_struct[2].end_y = 440;
	//graphic_draw.graphic_data_struct[2].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[2].graphic_name, (uint8_t*)name3, strlen(name3));
	
	
	//graph 4
	char name4[3] = "204";
	graphic_draw.graphic_data_struct[3].graphic_name[0] = name4[0];
	graphic_draw.graphic_data_struct[3].graphic_name[1] = name4[1];
	graphic_draw.graphic_data_struct[3].graphic_name[2] = name4[2];
	graphic_draw.graphic_data_struct[3].graphic_tpye = 1;
	//graphic_draw.graphic_data_struct[3].operate_tpye = 1;
	graphic_draw.graphic_data_struct[3].layer = 0;
	graphic_draw.graphic_data_struct[3].color = 2;
	graphic_draw.graphic_data_struct[3].width = 2;
	//graphic_draw.graphic_data_struct[3].start_angle=10;
	//graphic_draw.graphic_data_struct[3].end_angle=10;
	graphic_draw.graphic_data_struct[3].start_x = 930;
	graphic_draw.graphic_data_struct[3].start_y = 340;
	graphic_draw.graphic_data_struct[3].end_x = 990;
	graphic_draw.graphic_data_struct[3].end_y = 340;
	//graphic_draw.graphic_data_struct[3].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[3].graphic_name, (uint8_t*)name4, strlen(name4));
	//graph 5
	char name5[3] = "205";
	graphic_draw.graphic_data_struct[4].graphic_name[0] = name5[0];
	graphic_draw.graphic_data_struct[4].graphic_name[1] = name5[1];
	graphic_draw.graphic_data_struct[4].graphic_name[2] = name5[2];
	graphic_draw.graphic_data_struct[4].graphic_tpye = 2;
	//graphic_draw.graphic_data_struct[4].operate_tpye = 1;
	graphic_draw.graphic_data_struct[4].layer = 0;
	graphic_draw.graphic_data_struct[4].color = 8;
	graphic_draw.graphic_data_struct[4].width = 3;
	//graphic_draw.graphic_data_struct[4].start_angle=10;
	//graphic_draw.graphic_data_struct[4].end_angle=10;
	graphic_draw.graphic_data_struct[4].start_x = 1800;
	graphic_draw.graphic_data_struct[4].start_y = 500;
	//graphic_draw.graphic_data_struct[4].end_x = 1000;
	//graphic_draw.graphic_data_struct[4].end_y = 410;
	graphic_draw.graphic_data_struct[4].radius = 33;
	//memcpy(graphic_draw.graphic_data_struct[4].graphic_name, (uint8_t*)name5, strlen(name5));
	//graph 6
	char name6[3] = "206";
	graphic_draw.graphic_data_struct[5].graphic_name[0] = name6[0];
	graphic_draw.graphic_data_struct[5].graphic_name[1] = name6[1];
	graphic_draw.graphic_data_struct[5].graphic_name[2] = name6[2];
	graphic_draw.graphic_data_struct[5].graphic_tpye = 2;
	//graphic_draw.graphic_data_struct[5].operate_tpye = 1;
	graphic_draw.graphic_data_struct[5].layer = 0;
	graphic_draw.graphic_data_struct[5].color = 8;
	graphic_draw.graphic_data_struct[5].width = 3;
	//graphic_draw.graphic_data_struct[5].start_angle=10;
	//graphic_draw.graphic_data_struct[5].end_angle=10;
	graphic_draw.graphic_data_struct[5].start_x = 1800;
	graphic_draw.graphic_data_struct[5].start_y = 700;
	//graphic_draw.graphic_data_struct[5].end_x = 980;
	//graphic_draw.graphic_data_struct[5].end_y = 300;
	graphic_draw.graphic_data_struct[5].radius = 33;
	//memcpy(graphic_draw.graphic_data_struct[5].graphic_name, (uint8_t*)name6, strlen(name6));
	//graph 7
	char name7[3] = "207";
	graphic_draw.graphic_data_struct[6].graphic_name[0] = name7[0];
	graphic_draw.graphic_data_struct[6].graphic_name[1] = name7[1];
	graphic_draw.graphic_data_struct[6].graphic_name[2] = name7[2];
	graphic_draw.graphic_data_struct[6].graphic_tpye = 0;
	//graphic_draw.graphic_data_struct[6].operate_tpye = 1;
	graphic_draw.graphic_data_struct[6].layer = 0;
	graphic_draw.graphic_data_struct[6].color = 2;
	graphic_draw.graphic_data_struct[6].width = 2;
	//graphic_draw.graphic_data_struct[6].start_angle=10;
	//graphic_draw.graphic_data_struct[6].end_angle=10;
	graphic_draw.graphic_data_struct[6].start_x = 960;
	graphic_draw.graphic_data_struct[6].start_y = 540;
	graphic_draw.graphic_data_struct[6].end_x = 960;
	graphic_draw.graphic_data_struct[6].end_y = 200;
	//graphic_draw.graphic_data_struct[6].radius = 30;
	//memcpy(graphic_draw.graphic_data_struct[6].graphic_name, (uint8_t*)name7, strlen(name7));

	referee_send_multi_graphic(MY_CLIENT_ID, &graphic_draw);
}


void send_capvol_graphic(int capvols)
{
	graphic_data_struct_t graphic_draw;
	char capname[3] = "209";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(capdraw){
	graphic_draw.operate_tpye = 1;
	capdraw = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	
	graphic_draw.color = 1;
	graphic_draw.graphic_tpye = 0;
	graphic_draw.layer = 0;

	graphic_draw.width = 18; //线条宽度
	graphic_draw.start_x = 761;
	graphic_draw.start_y = 70;
	
	graphic_draw.end_x = capvols;
	graphic_draw.end_y = 70;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

void referee_send_multi_graphic(ext_id_t target_id, ext_client_custom_graphic_seven_t* graphic_draw){

	static ext_robot_sev_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 7 *15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = student_interactive_header;
	robot_data.data_id = 0x0104;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去

}

void referee_send_two_graphic(ext_id_t target_id, ext_client_custom_graphic_double_t* graphic_draw) {

	static ext_robot_two_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	//robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	robot_data.header.data_length = 6 + 2 * 15;
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = student_interactive_header;
	robot_data.data_id = 0x0102;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;

	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));//将定制好的图形数据传输出去
}

void send_double_text(void)
{
	ext_client_custom_graphic_double_t graphic_draw;
	char name1[3] = "221";
	graphic_draw.graphic_data_struct[0].graphic_name[0] = name1[0];
	graphic_draw.graphic_data_struct[0].graphic_name[1] = name1[1];
	graphic_draw.graphic_data_struct[0].graphic_name[2] = name1[2];
	graphic_draw.graphic_data_struct[0].operate_tpye = 1;
	graphic_draw.graphic_data_struct[0].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[0].layer = 0;
	graphic_draw.graphic_data_struct[0].color = 2;
	graphic_draw.graphic_data_struct[0].width =2;
	graphic_draw.graphic_data_struct[0].start_x =556;
	graphic_draw.graphic_data_struct[0].start_y =0;
	graphic_draw.graphic_data_struct[0].end_x =706;
	graphic_draw.graphic_data_struct[0].end_y =240;
	
	char name2[3] = "223";
	graphic_draw.graphic_data_struct[1].graphic_name[0] = name2[0];
	graphic_draw.graphic_data_struct[1].graphic_name[1] = name2[1];
	graphic_draw.graphic_data_struct[1].graphic_name[2] = name2[2];
	graphic_draw.graphic_data_struct[1].operate_tpye = 1;
	graphic_draw.graphic_data_struct[1].graphic_tpye = 0;
	graphic_draw.graphic_data_struct[1].layer = 0;
	graphic_draw.graphic_data_struct[1].color = 2;
	graphic_draw.graphic_data_struct[1].width =2;
	graphic_draw.graphic_data_struct[1].start_x =1364;
	graphic_draw.graphic_data_struct[1].start_y =0;
	graphic_draw.graphic_data_struct[1].end_x =1214;
	graphic_draw.graphic_data_struct[1].end_y =240;
	
	referee_send_two_graphic(MY_CLIENT_ID, &graphic_draw);
	
}


void referee_send_client_graphic(ext_id_t target_id, graphic_data_struct_t* graphic_draw) 
{
	static ext_robot_graphic_data_t robot_data;

	robot_data.header.sof = REFEREE_FRAME_HEADER_SOF;
	robot_data.header.seq++;
	robot_data.header.data_length = sizeof(robot_data) - sizeof(robot_data.header) - sizeof(robot_data.cmd_id) - sizeof(robot_data.crc16);
	
	append_CRC8_check_sum((uint8_t*)&robot_data.header, sizeof(robot_data.header));
	
	robot_data.cmd_id = student_interactive_header;
	robot_data.data_id = 0x0101;
	robot_data.sender_id = MY_ROBOT_ID;
	robot_data.robot_id = target_id;
	
	robot_data.graphic_data = *graphic_draw;
	append_CRC16_check_sum((uint8_t*)&robot_data, sizeof(robot_data));//帧头CRC校验

	memcpy(Personal_Data, (uint8_t*)&robot_data, sizeof(robot_data));
	usart6_tx_dma_enable(Personal_Data, sizeof(robot_data));

}


void send_rub_graphic(char graphname[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	graphic_draw.graphic_name[0] = graphname[0];
	graphic_draw.graphic_name[0] = graphname[1];
	graphic_draw.graphic_name[0] = graphname[2];
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.radius = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}



void send_spinning_graphic(char name[3], int x, int y, int color, int type)
{
	
	graphic_data_struct_t graphic_draw;
	
	graphic_draw.graphic_name[0] =name[0];
	graphic_draw.graphic_name[1] =name[1];
	graphic_draw.graphic_name[2] =name[2];
	
	
	graphic_draw.operate_tpye = type;
	
	graphic_draw.color = color;
	graphic_draw.graphic_tpye = 2;
	
	graphic_draw.layer = 0;
	graphic_draw.width = 20; //线条宽度
	
	graphic_draw.start_x = x;
	graphic_draw.start_y = y;
	
	graphic_draw.radius = 20;
	
	//memcpy(graphic_draw.graphic_name, (uint8_t*)name, strlen(name));

	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}


void USART6_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
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
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
        }
    }
}

void send_autoaim_state(void){
	graphic_data_struct_t graphic_draw;
	char capname[3] = "409";
	graphic_draw.graphic_name[0] = capname[0];
	graphic_draw.graphic_name[1] = capname[1];
	graphic_draw.graphic_name[2] = capname[2];
	if(auto_state){
	graphic_draw.operate_tpye = 1;
	auto_state = false;
	}
	else
	{
	graphic_draw.operate_tpye = 2;
	}
	graphic_draw.graphic_tpye = 1;
	graphic_draw.layer = 0;
	if(gimbal_data.gimbal_behaviour == GIMBAL_AUTO)
		graphic_draw.color = 2;
	else
		graphic_draw.color = 1;
	graphic_draw.width = 10; 
	graphic_draw.start_x = 1790;
	graphic_draw.start_y = 610;
	
	graphic_draw.end_x = 1810;
	graphic_draw.end_y = 590;
	referee_send_client_graphic(MY_CLIENT_ID, &graphic_draw);
}

//读取底盘功率
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = ext_power_heat_data.chassis_power;
    *buffer = ext_power_heat_data.chassis_power_buffer;
}
