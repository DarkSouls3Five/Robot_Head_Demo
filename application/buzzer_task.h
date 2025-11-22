#ifndef _BUZZER_TASK_H_
#define _BUZZER_TASK_H_


void BuzzerTask(void const * argument);


#include "main.h"


#define  ZERO  3000//
	#define  R  1000000 //F_CLOCK/(psc+1)=50M/(49+1)=1000000
	//set tune *****************
	#define  L1       (R/262)-1  
	#define  half_L1  (R/277)-1
	#define  L2       (R/294)-1
	#define  half_L2  (R/311)-1
	#define  L3       (R/330)-1
	#define  L4       (R/349)-1
	#define  half_L4  (R/370)-1
	#define  L5       (R/392)-1
	#define  half_L5  (R/410)-1
	#define  L6       (R/440)-1
	#define  half_L6  (R/466)-1
	#define  L7       (R/494)-1

	#define  M1       (R/523)-1
	#define  half_M1  (R/554)-1        
	#define  M2       (R/587)-1
	#define  half_M2  (R/622)-1
	#define  M3       (R/659)-1
	#define  M4       (R/698)-1
	#define  half_M4  (R/740)-1
	#define  M5       (R/784)-1
	#define  half_M5  (R/831)-1
	#define  M6       (R/880)-1
	#define  half_M6  (R/932)-1
	#define  M7       (R/988)-1

	#define  H1       (R/1046)-1
	#define  half_H1  (R/1109)-1
	#define  H2       (R/1175)-1
	#define  half_H2  (R/1245)-1
	#define  H3       (R/1318)-1
	#define  H4       (R/1397)-1
	#define  half_H4  (R/1480)-1
	#define  H5       (R/1568)-1
	#define  half_H5  (R/1661)-1
	#define  H6       (R/1760)-1
	#define  half_H6  (R/1865)-1
	#define  H7       (R/1967)-1


void fn_BuzzerPrompt(void);
void fn_BuzzerClose(void);

#endif
