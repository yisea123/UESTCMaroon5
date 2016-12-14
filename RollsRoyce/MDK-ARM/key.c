/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "tasks.h"
#include "math.h"
/* USER CODE END Includes */

typedef enum 
{
	KeyState_Press=0,
	KeyState_Npres
}KeyStateE;

struct KeyS
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	KeyStateE KeySate;
}Key[4];

KeyScanRltS KeyScanRlt;

//struct KeyStateMachineS
//{
//	int 
//}
FrameS Frame;

void KeyInit(void)
{
	Key[0].GPIOx=GPIOC;
	Key[0].GPIO_Pin=GPIO_PIN_0;
	Key[0].KeySate=KeyState_Npres;
	
	Key[1].GPIOx=GPIOC;
	Key[1].GPIO_Pin=GPIO_PIN_1;
	Key[1].KeySate=KeyState_Npres;
		
	Key[2].GPIOx=GPIOC;
	Key[2].GPIO_Pin=GPIO_PIN_2;
	Key[2].KeySate=KeyState_Npres;
		
	Key[3].GPIOx=GPIOC;
	Key[3].GPIO_Pin=GPIO_PIN_3;
	Key[3].KeySate=KeyState_Npres;
	
	Frame.Header_1=0x11;
	Frame.Header_2=0x12;
	
	Frame.Tail_1=0xFF;
	Frame.Tail_2=0xFE;
}
void BTSendMsg(MsgFromHostIndexE ID,float data)
{
	Frame.ID=ID;
	Frame.Data=data;
	HAL_UART_Transmit(&huart3,(uint8_t*)&Frame,sizeof(Frame),20);
}
void KeyScanTask(void const * argument)
{
	KeyInit();
	while(1)
	{
		for(int i=0;i<4;i++)
		{
			if(HAL_GPIO_ReadPin(Key[i].GPIOx,Key[i].GPIO_Pin)==GPIO_PIN_SET)		//未按下
			{
				if(Key[i].KeySate==KeyState_Press)
				{
					KeyScanRlt.KeyPressedID=i;
					if(i<3)
					{
						Mission.MissionID=i;
						BTSendMsg(MISSION_ID,i);
					}
					else if(i==3)
					{
						cv_start_led=1;
						BTSendMsg(CV_START,1);
					}
				}
				Key[i].KeySate=KeyState_Npres;
			}
			else if(HAL_GPIO_ReadPin(Key[i].GPIOx,Key[i].GPIO_Pin)==GPIO_PIN_RESET)		//按下
			{
				Key[i].KeySate=KeyState_Press;
			}
		}
	}
	for(;;)
  {
    osDelay(1);
  }
}