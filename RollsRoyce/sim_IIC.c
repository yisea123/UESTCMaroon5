
#include "sim_IIC.h"


//SCL:E7
//SDA:E8


#define IIC_DELAY IIC_Delay(15);


/**
  * @brief  ģ��IIC��ʼ��
  * @note   SCL:E7		SDA:E8
  * @param  void
  * @retval void
  */
void IIC_Init(void)
{
	;
}


/**
  * @brief  IIC��ָ���豸ָ���Ĵ���д��һ��byte
  * @param  Ŀ���豸��ַ
  * @param  �Ĵ�����ַ
  * @param  Ҫд�������
  * @retval 0 �ɹ�		1 ʧ��
  */
uint8_t IIC_SingleSend(uint8_t addr, uint8_t reg, uint8_t data)
{
	if(IIC_Start())
	{
		return 1;
	}
	IIC_SendByte(addr << 1);
	if(IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitACK();
	IIC_SendByte(data);
	IIC_WaitACK();
	IIC_Stop();
	return 0;
}


/**
  * @brief  IIC��ָ���豸ָ���Ĵ�����ȡһ��byte
  * @param  Ŀ���豸��ַ
  * @param  �Ĵ�����ַ
  * @param  �������ݴ�ŵ�ַ
  * @retval 0 �ɹ�		1 ʧ��
  */
uint8_t IIC_SingleRead(uint8_t addr, uint8_t reg, uint8_t *data)
{
	if(IIC_Start())
	{
		return 1;
	}
	IIC_SendByte(addr << 1);
	if(IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitACK();
	IIC_Start();
	IIC_SendByte(addr << 1 | 0x01);
	IIC_WaitACK();
	*data = IIC_ReadByte();
	IIC_NACK();
	IIC_Stop();
	return 0;
	
}


/**
  * @brief  IIC���豸��ĳһ����ַд��ָ�����ȵ�����
  * @param  �豸��ַ
  * @param  �Ĵ�����ַ
  * @param  ���ݳ���
  * @param  Ҫ����������ڴ��ַ
  * @retval 0 �ɹ�		1 ʧ��
  */
uint8_t IIC_SendBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	uint8_t i;
	
	if(IIC_Start())	//����ʧ��
	{
		return 1;
	}
	IIC_SendByte(addr << 1);
	if(IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitACK();
	for(i = 0; i < len; i++)
	{
		IIC_SendByte(data[i]);
		if(IIC_WaitACK())
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}


/**
  * @brief  IIC���豸��ĳһ����ַ��ȡָ�����ȵ�����
  * @param  �豸��ַ
  * @param  �Ĵ�����ַ
  * @param  ���ݳ���
  * @param  ��ȡ��ȡ�����ݵĵ�ַ
  * @retval 0 �ɹ�		1 ʧ��
  */
uint8_t IIC_ReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	if(IIC_Start())
	{
		return 1;
	}
	IIC_SendByte(addr << 1);
	if(IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg);
	IIC_WaitACK();
	IIC_Start();
	IIC_SendByte(addr << 1 | 0x01);
	IIC_WaitACK();
	while(len)
	{
		*data = IIC_ReadByte();
		if(len == 1)
		{
			IIC_NACK();
		}
		else
		{
			IIC_ACK();
		}
		data++;
		len--;
	}
	IIC_Stop();
	return 0;
}


//����us����ʱ
void IIC_Delay(uint32_t time)
{
	while(time--);
}


/**
  * @brief  ����IIC��ʼ�ź�
  * @param  void
  * @retval 0 �����ɹ�		1 ����ʧ��
  */
uint8_t IIC_Start(void)
{
	IIC_SDA_H;
	IIC_SCL_H;
	IIC_DELAY;
	if(!IIC_SDA_Read)
	{
		return 1;
	}
	IIC_SDA_L;
	IIC_DELAY;
	if(IIC_SDA_Read)
	{
		return 1;
	}
	IIC_DELAY;
	return 0;
}


/**
  * @brief  ����IICֹͣ�ź�
  * @param  void
  * @retval void 
  */
void IIC_Stop(void)
{
	IIC_SCL_L;
	IIC_DELAY;
	IIC_SDA_L;
	IIC_DELAY;
	IIC_SCL_H;
	IIC_DELAY;
	IIC_SDA_H;
	IIC_DELAY;
}


/**
  * @brief  ����IIC ACK�ź�
  * @param  void
  * @retval void 
  */
void IIC_ACK(void)
{
	IIC_SCL_L;
	IIC_DELAY;
	IIC_SDA_L;
	IIC_DELAY;
	IIC_SCL_H;
	IIC_DELAY;
	IIC_SCL_L;
	IIC_DELAY;
}


/**
  * @brief  ����IIC NACK�ź�
  * @param  void
  * @retval void 
  */
void IIC_NACK(void)
{
	IIC_SCL_L;
	IIC_DELAY;
	IIC_SDA_H;
	IIC_DELAY;
	IIC_SCL_H;
	IIC_DELAY;
	IIC_SCL_L;
	IIC_DELAY;
}


/**
  * @brief  �ȴ�IIC NACK�ź�
  * @param  void
  * @retval 0 ����ACK�ɹ�		1 ����ACKʧ�� 
  */
uint8_t IIC_WaitACK(void)
{
	IIC_SCL_L;
	IIC_DELAY;
	IIC_SDA_H;
	IIC_DELAY;
	IIC_SCL_H;
	IIC_DELAY;
	if(IIC_SDA_Read)
	{
		IIC_SCL_L;
		return 1;
	}
	IIC_SCL_L;
	return 0;
}


/**
  * @brief  IIC����һ��byte
  * @param  Ҫ���͵�byte
  * @retval void
  */
void IIC_SendByte(uint8_t byte)
{
	uint8_t i = 8;
	while(i--)
	{
		IIC_SCL_L;
		IIC_DELAY;
		if(byte & 0x80)
		{
			IIC_SDA_H;
		}
		else
		{
			IIC_SDA_L;
		}
		byte <<= 1;
		IIC_DELAY;
		IIC_SCL_H;
		IIC_DELAY;
	}
	IIC_SCL_L;
}


/**
  * @brief  IIC����һ��byte
  * @param  void
  * @retval ���յ���byte
  */
uint8_t IIC_ReadByte(void)
{
	uint8_t i = 8;
	uint8_t byte = 0;
	
	IIC_SDA_H;
	while(i--)
	{
		byte <<= 1;
		IIC_SCL_L;
		IIC_DELAY;
		IIC_SCL_H;
		IIC_DELAY;
		if(IIC_SDA_Read)
		{
			byte |= 0x01;
		}
	}
	IIC_SCL_L;
	return byte;
}







