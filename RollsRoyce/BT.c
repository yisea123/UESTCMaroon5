/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "tasks.h"
/* USER CODE END Includes */

#define HEADER_1 0x11 
#define HEADER_2 0x12

#define MPUDATA 0x20	//陀螺仪数据
#define MTRDATA 0x21	//电机数据

#define TAIL_1 0xFF
#define TAIL_2 0xFE
#define TAIL_3 0xFD
//uint8_t BTData[12];

void BTTask(void const * argument)
{
//	HAL_UART_Receive_DMA(&huart3,BTData,12);
	for(;;)
  {
    osDelay(1);
  }
}

MsgFromHostIndexE MsgFromHostIndex;
float MsgFromHost[MsgLength];

union BitConverterU
{
	unsigned char byte[4];
	float value;
} BitConverter;

void FrameAnalysis(unsigned char buff[],int buff_len)
{
	unsigned char frame_fifo[FrameLength];
	for(int i=0;i<buff_len;i++)
	{
			//FIFO??
			for(int j=0;j<FrameLength-1;j++)
			{
					frame_fifo[j] = frame_fifo[j + 1]; 
			}
			frame_fifo[FrameLength - 1] = buff[i];
			
			if (frame_fifo[Header_1] == 0x11
					&& frame_fifo[Header_2] == 0x12
					&& frame_fifo[Tail_1] == 0xFF
					&& frame_fifo[Tail_2] == 0xFE)
			{           
					unsigned char id = frame_fifo[ID];
					BitConverter.byte[0] = frame_fifo[Byte0];
					BitConverter.byte[1] = frame_fifo[Byte1];
					BitConverter.byte[2] = frame_fifo[Byte2];
					BitConverter.byte[3] = frame_fifo[Byte3];

					if(id<=MsgLength)
					{
							MsgFromHost[id] = BitConverter.value;
					}
			}
		}
}
