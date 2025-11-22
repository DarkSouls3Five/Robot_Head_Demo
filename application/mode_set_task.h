#ifndef MODE_SET_TASK_H
#define MODE_SET_TASK_H
#include "struct_typedef.h"
#include "pid.h"


#define MODE_SET_TASK_INIT_TIME 300
#define MODE_SET_TIME_MS 5

typedef enum
{
  MODE_FREE,	//自由模式
	MODE_WORK,	//工作模式
	
} head_mode_e;


typedef struct
{
  head_mode_e last_head_mode;               //state machine. ???????????
	head_mode_e head_mode; 

} head_mode_t;



#endif
