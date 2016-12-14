/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "tasks.h"
#include "math.h"
#define MAXSPEED 2000
/* USER CODE END Includes */

float myabs(float x)
{
	return x>0?x:-x;
}

uint8_t t[1];
int cv_start_led=0;
void LEDTask(void const * argument)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

	while(1)
	{
		if(cv_start_led)
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
		else 
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		
		if(MsgFromHost[CV_READY])
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		else 
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		
		if(Car.start)
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		else 
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		
		osDelay(200);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//		osDelay(200);
	}
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}

typedef enum MotorStatusE
{
	MotorStaus_Reverse=-1,
	MotorStaus_Stop=0,
	MotorStaus_Forward=1
} MotorStatusE;

typedef enum MotorIdE
{
	Motor0=0,
	Motor1,
	Motor2,
	Motor3
} MotorIdE;

struct MotorS
{
	TIM_HandleTypeDef* tim;
	__IO uint32_t* IH1;
	__IO uint32_t* IH2;	
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	MotorStatusE MotorStatus;
	uint32_t targetSpeed;
} Motor[4];

void MotorInit(void)
{
	//Motor3
	Motor[3].tim=&htim8;
	Motor[3].IH1=&(TIM8->CCR4);
	Motor[3].IH2=&(TIM8->CCR3);
	Motor[3].GPIOx=GPIOB;
	Motor[3].GPIO_Pin=GPIO_PIN_13;	
	Motor[3].targetSpeed=0;
	Motor[3].MotorStatus=MotorStaus_Stop;
	
	//Motor2
	Motor[2].tim=&htim8;
	Motor[2].IH1=&(TIM8->CCR2);
	Motor[2].IH2=&(TIM8->CCR1);
	Motor[2].GPIOx=GPIOB;
	Motor[2].GPIO_Pin=GPIO_PIN_12;
	Motor[2].targetSpeed=0;
	Motor[2].MotorStatus=MotorStaus_Stop;

	//Motor1
	Motor[1].tim=&htim4;
	Motor[1].IH1=&(TIM4->CCR2);
	Motor[1].IH2=&(TIM4->CCR1);
	Motor[1].GPIOx=GPIOD;
	Motor[1].GPIO_Pin=GPIO_PIN_2;
	Motor[1].targetSpeed=0;
	Motor[1].MotorStatus=MotorStaus_Stop;

	//Motor0
	Motor[0].tim=&htim4;
	Motor[0].IH1=&(TIM4->CCR4);
	Motor[0].IH2=&(TIM4->CCR3);
	Motor[0].GPIOx=GPIOC;
	Motor[0].GPIO_Pin=GPIO_PIN_12;
	Motor[0].targetSpeed=0;
	Motor[0].MotorStatus=MotorStaus_Stop;
}	


 CarS Car;

/**
  * @brief  MotorSpeedControl.
  * @param  MotorId.
  * @param  speed.
	* @retval 0:normal 1:deadzone
  */
int MotorSpeedControl(MotorIdE MotorId,float speed)
{
	int rlt=0;
	if(myabs(speed)<=400) 
	{
		speed=0;
		rlt=1;
	}
	if(speed>MAXSPEED)
	{
		speed=MAXSPEED;
	}
	else if(speed<-MAXSPEED)
	{
		speed=-MAXSPEED;
	}
	if(speed==0)
	{
		*Motor[MotorId].IH1=0;
		*Motor[MotorId].IH2=0;
		Motor[MotorId].MotorStatus=MotorStaus_Stop;
	}
	else if(speed*Motor[MotorId].MotorStatus<0)
	{
		*Motor[MotorId].IH1=0;
		*Motor[MotorId].IH2=0;
		osDelay(1);
		Motor[MotorId].MotorStatus*=-1;
		if(speed>0)
		{
			*Motor[MotorId].IH1=speed;
			*Motor[MotorId].IH2=0;
			HAL_GPIO_WritePin(Motor[MotorId].GPIOx,Motor[MotorId].GPIO_Pin,GPIO_PIN_SET);
			Motor[MotorId].MotorStatus=MotorStaus_Forward;
		}
		else if(speed<0)
		{
			*Motor[MotorId].IH1=0;
			*Motor[MotorId].IH2=-speed;
			HAL_GPIO_WritePin(Motor[MotorId].GPIOx,Motor[MotorId].GPIO_Pin,GPIO_PIN_RESET);
			Motor[MotorId].MotorStatus=MotorStaus_Reverse;
		}
	}
	else if(speed*Motor[MotorId].MotorStatus>=0)
	{
		if(speed>0)
		{
			*Motor[MotorId].IH1=speed;
			*Motor[MotorId].IH2=0;
			HAL_GPIO_WritePin(Motor[MotorId].GPIOx,Motor[MotorId].GPIO_Pin,GPIO_PIN_SET);
			Motor[MotorId].MotorStatus=MotorStaus_Forward;
		}
		else if(speed<0)
		{
			*Motor[MotorId].IH1=0;
			*Motor[MotorId].IH2=-speed;
			HAL_GPIO_WritePin(Motor[MotorId].GPIOx,Motor[MotorId].GPIO_Pin,GPIO_PIN_RESET);
			Motor[MotorId].MotorStatus=MotorStaus_Reverse;
		}
	}
	return rlt;
}

/**
  * @brief  CarSpeedControl.
  * @param  speed.
	* @retval 0:normal 1:deadzone
  */
void CarYawControl(float speed,float yaw)	
{
	MotorSpeedControl(Motor0,speed-yaw);
	MotorSpeedControl(Motor3,(speed-yaw)*1.3);

	MotorSpeedControl(Motor1,speed+yaw);
	MotorSpeedControl(Motor2,speed+yaw);
}

void MotorTask(void const * argument)
{
	MotorInit();
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
//	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)!=GPIO_PIN_RESET) 
//	{}
	while(1)
	{
		Car.start=MsgFromHost[START];
		if(Car.start)
		{
//			if(myabs(Car.speed)<30)	Car.speed=0;
//			if(myabs(Car.yaw)<3)	Car.yaw=0;
			CarYawControl(Car.speed,Car.yaw);
		}
		else if(!Car.start)
		{
			CarYawControl(0,0);
		}
		osDelay(10);
	}
	for(;;)
  {
    osDelay(1);
  }
}
