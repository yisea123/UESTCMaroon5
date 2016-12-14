
#ifndef _SIM_IIC_H
#define _SIM_IIC_H
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
//#include "sys.h"


#define IIC_SCL_L		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define IIC_SCL_H		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define IIC_SDA_L		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define IIC_SDA_H		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define	IIC_SDA_Read	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)

void IIC_Init(void);
uint8_t IIC_SingleSend(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t IIC_SingleRead(uint8_t addr, uint8_t reg, uint8_t *data);
uint8_t IIC_SendBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t IIC_ReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);


void IIC_Delay(uint32_t time);
uint8_t IIC_Start(void);
void IIC_Stop(void);
void IIC_ACK(void);
void IIC_NACK(void);
uint8_t IIC_WaitACK(void);
void IIC_SendByte(uint8_t byte);
uint8_t IIC_ReadByte(void);



#endif
