#ifndef _REMOTE_CONTROL_H_
#define _REMOTE_CONTROL_H_

#include "struct_typedef.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

typedef struct
{
struct
{
	unsigned short ch0;
	unsigned short ch1;
	unsigned short ch2;
	unsigned short ch3;
	unsigned char s1;
	unsigned char s2;
  
}rc;

struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	unsigned char press_l;
	unsigned char press_r;
	
}mouse;

struct
{
	unsigned short v;
}key;

}Rc_ctrl;

//遥感中心死区
#define ZeroCh ((uint16_t)50)

/* ----------------------- RC Channel Definition---------------------------- */

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define    RC_CH_VALUE_MAX       ((uint16_t)1684)


/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)


/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* 获取遥控器摇杆偏移值 
   RLR：右摇杆左右移动  LUD：左摇杆上下移动	*/
#define    RC_CH0_RLR_OFFSET    (ctl.rc.ch0 - RC_CH_VALUE_OFFSET)
#define    RC_CH1_RUD_OFFSET  	(ctl.rc.ch1 - RC_CH_VALUE_OFFSET)
#define    RC_CH2_LLR_OFFSET  	(ctl.rc.ch2 - RC_CH_VALUE_OFFSET)
#define    RC_CH3_LUD_OFFSET  	(ctl.rc.ch3 - RC_CH_VALUE_OFFSET)


/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (ctl.rc.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (ctl.rc.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (ctl.rc.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (ctl.rc.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (ctl.rc.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (ctl.rc.s2 == RC_SW_DOWN)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (ctl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (ctl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (ctl.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (ctl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (ctl.mouse.press_r == 1)


/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  ctl.key.v  )
#define    IF_KEY_PRESSED_W       ( (ctl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (ctl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (ctl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (ctl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (ctl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (ctl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (ctl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (ctl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (ctl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (ctl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (ctl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (ctl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (ctl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (ctl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (ctl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (ctl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

void remote_control_init(void);

extern Rc_ctrl ctl;


#endif
