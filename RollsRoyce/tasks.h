#ifndef __TASKS_H
#define __TASKS_H
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define LED_THREAD
#define MOTOR_THREAD
#define MATH_THREAD
#define KEYSCAN_THREAD

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t BTData[];
extern float MsgFromHost[];
extern int cv_start_led;

typedef enum 
{
	P0_Y_OFS=0,
	P0_X_OFS,	

	HOME_X,
	HOME_Y,
	
	MISSION_ID,
	MISSION_STAGE,
	
	Car_X,
	Car_Y,
	Car_YAW,
	
	START,
	
	CV_READY,
	CV_START,
	
	MsgLength
}MsgFromHostIndexE;
typedef enum 
{
		Header_1 = 0,
		Header_2,
		ID,
		Byte0,
		Byte1,
		Byte2,
		Byte3,
		Tail_1,
		Tail_2,
		FrameLength
}FrameFormatIndex;
typedef __packed struct 
{
		unsigned char Header_1;
		unsigned char Header_2;
		unsigned char ID;
		float Data;
		unsigned char Tail_1;
		unsigned char Tail_2;
} FrameS;
typedef struct 
{
	float speed;
	float yaw;
	int start;
	float x2edge;
	float y2edge;
	float yaw2edge;
	float y2home;
}CarS;
extern CarS Car;
typedef enum 
{
	StandBy=-1,
	Park=0,
	Change,
	Return
}	MissionE;
typedef struct 
{
	int LEDState;
	int KeyPressedID;
}KeyScanRltS;
extern KeyScanRltS KeyScanRlt;
typedef enum 
{
	Standby=-1,
	Steering0,
	Reversing,
	Steering1,
	Moving1,
	Steering2,
	Moving2,
	Steering3,
}	MissionStageE;
typedef struct 
{
	MissionE MissionID;
	int Finished;
	int targetID;
	MissionStageE MissionStage;
	float x_ofs;
	float y_ofs;
}MissionS;
extern MissionS Mission;

void LEDTask(void const * argument);
void MotorTask(void const * argument);
void BTTask(void const * argument);
void MathTask(void const * argument);
void KeyScanTask(void const * argument);
void FrameAnalysis(unsigned char buff[],int buff_len);
void BTSendMsg(MsgFromHostIndexE ID,float data);
float myabs(float x);

#endif
