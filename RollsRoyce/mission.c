/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "tasks.h"
#include "mission.h"
/* USER CODE END Includes */

PidTypeDef pidSpeed;
PidTypeDef pidSpeed1;
PidTypeDef pidYaw;
void PID_Init(void)
{
	PID_SetParam(&pidSpeed,6,0.1,0.2,1,0,0,0);
	PID_SetParam(&pidSpeed1,1,0.1,0.2,1,0,0,0);
	PID_SetParam(&pidYaw,16,0.1,0.2,1,0,0,0);
}

//设置参数
/*
beta
dif_prior
kaff加速度前馈
kvff速度前馈
*/
void PID_SetParam(PidTypeDef * pid, float p, float i, float d, float beta,
                  float dif_prior, float kaff, float kvff)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->beta = beta;
    pid->dif_prior = dif_prior;
    pid->Kaff = kaff;
    pid->Kvff = kvff;
}
const float INTEGRAL_MAX=1000.f;
//PID计算
void PID_Calc(PidTypeDef * pid,  float rel_val, float set_val,int32_t dertT)
{
	float max=2000;
    float p = 0,
          i = 0,
          d = 0,
          kvff,
          kaff,
          Deriv,
          dt= dertT/1000.0;//dertT微妙
    pid->s[2] = set_val;
    pid->e[2] = set_val - rel_val; //获取当前误差
    pid->r[2] = rel_val; //获取当前值

    if (dt==0) return;
    /*计算增量*/
    p = pid->Kp * pid->e[2]; //位置式p
    //p = pid->Kp * (pid->e[2] - pid->e[1]); //增量式P
    
    /*积分*/
  
//    if(i==0) 
//    {
//        pid->integral=0;
//        i=0;
//    }
    
    /*微分先行*/
    
    if (pid->dif_prior)
    {
        Deriv=(pid->r[2] - pid->r[1])/dt;
        d = pid->Kd * Deriv;
    }
    else
    {
        Deriv=(pid->e[2] - pid->e[1])/dt;
        d = pid->Kd * Deriv;
    }
    kvff = (pid->s[2] - pid->s[1])/dt * pid->Kvff;//速度前馈
    kaff = (pid->s[2] - 2 * pid->s[1] + pid->s[0])/dt * pid->Kaff;//加速度前馈
    /*积分分离*/
    if (pid->beta == 0)
    {
        pid->integral += pid->e[2] * pid->Ki * dt;
        i = pid->integral;
    }
    else
    {
        if (((pid->e[2])<= pid->beta && (pid->e[2]) >= -pid->beta))
           // &&(d*p>0))
        {
            pid->integral += pid->e[2] * pid->Ki * dt;
            i = pid->integral;
        }
        else
        {
            i = 0;
        }
    }
    
    if (pid->integral > INTEGRAL_MAX)
    {
        pid->integral=INTEGRAL_MAX;
    }
    else if (pid->integral < -INTEGRAL_MAX)
    {
        pid->integral=-INTEGRAL_MAX;
    }
//    pid->delta_U = p + i + d + kvff + kaff; //增量式PID
    pid->U = p + i + d + kvff + kaff;//pid->last_U + pid->delta_U; //位置式PID
    if (pid->U > max)
    {
        pid->U = max;
    }
    else if (pid->U < -max)
    {
        pid->U = -max;
    }

    /*记录上一次输出*/
    pid->last_U = pid->U;
    /*迭代设定值*/
    pid->s[0] = pid->s[1];
    pid->s[1] = pid->s[2];
    /*迭代误差*/
    pid->e[0] = pid->e[1];
    pid->e[1] = pid->e[2];
    /*迭代实际值*/
    pid->r[0] = pid->r[1];
    pid->r[1] = pid->r[2];
}
MissionS Mission;
void MissionInit(void)
{
	Mission.MissionID=-1;
	Mission.targetID=-1;
	Mission.Finished=0;
	Mission.MissionStage=Standby;
}

void setCarParam(float speed,float yaw)
{
	Car.speed=speed;
	Car.yaw=yaw;
}

void DataUpdate(void)
{
	Mission.x_ofs=MsgFromHost[P0_X_OFS];
	Mission.y_ofs=MsgFromHost[P0_Y_OFS];
	Car.x2edge=MsgFromHost[Car_X];
	Car.y2edge=MsgFromHost[Car_Y];
	Car.yaw2edge=MsgFromHost[Car_YAW];
	Car.y2home=MsgFromHost[HOME_Y];
}

void MathTask(void const * argument)
{
	MissionInit();
	PID_Init();
	float initYaw=0,initX=0,initY=0;
	while(1)
	{
		switch(Mission.MissionID)
		{
			case StandBy:
					initYaw=Car.yaw2edge;
					initX=Car.x2edge;
					break;
			case Park:
					switch(Mission.MissionStage)		//start to reverse
					{
						case Standby:
//							if(Mission.MissionStage!=Reversing)
							osDelay(1000);
							Mission.MissionStage=Reversing; 
							break;
						case Reversing:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(475-Car.x2edge>100)
							{
								PID_Calc(&pidSpeed,Car.x2edge,475,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.y2edge,Car.y2home,10);	
								setCarParam(-pidSpeed.U,pidYaw.U);
							}
							else
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								osDelay(500);
								Mission.MissionStage=Steering1;
							}
							break;
						case Steering1:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(myabs(myabs(Car.yaw2edge)-180)>10)
							{
								PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,180*(Car.yaw2edge>0?1:-1),10);	
//								setCarParam(0,pidYaw.U);
								setCarParam(0,0);
							}
							else
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Moving1;
							}
							break;
						case Moving1:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(myabs(Mission.y_ofs)>10)
							{
								PID_Calc(&pidSpeed,Mission.y_ofs,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.x2edge,475,10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Steering2;
							}
							break;
						case Steering2:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(myabs(Car.yaw2edge-90)>5)
							{
								PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
								setCarParam(0,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Moving2;
							}
							break;
						case Moving2:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(myabs(Mission.x_ofs)>50||myabs(Mission.y_ofs)>10)
							{
								PID_Calc(&pidSpeed,Mission.x_ofs,50,10);		  //Data Calc
								PID_Calc(&pidYaw,Mission.y_ofs,90,10);	
								setCarParam(-pidSpeed.U,pidYaw.U);						//倒车所以速度是负的
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Steering3;
							}
							break;
						case Steering3:
							BTSendMsg(MISSION_STAGE,Mission.MissionStage);
							if(Mission.targetID==1)
							{
								if(myabs(Car.yaw2edge)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,0,10);	
									setCarParam(0,pidYaw.U);
								}
								else 
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;
								}
							}
							else if(Mission.targetID==2)
							{
								if(myabs(myabs(Car.yaw2edge)-180)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,180*(Car.yaw2edge>0?1:-1),10);	
									setCarParam(0,pidYaw.U);
								}
								else
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;								
								}
							}
							else if(Mission.targetID==0||Mission.targetID==3)
							{
								if(myabs(Car.yaw2edge-90)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
									setCarParam(0,pidYaw.U);
								}
								else 
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;			
								}
							}
							break;
					}	
				break;
			case Change:
					switch(Mission.MissionStage)		//start to reverse
					{
						case Standby:
							if(Mission.MissionStage!=Steering0)
								osDelay(1000);
							Mission.MissionStage=Steering0;
							if(myabs(Car.yaw2edge-90)>1)
							{
								PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Reversing;
							}
							break;
						case Reversing:
							if(myabs(Car.x2edge-475)>20)
							{
								PID_Calc(&pidSpeed,Car.x2edge,475,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Steering1;
							}
							break;
						case Steering1:						
							if(myabs(myabs(Car.yaw2edge)-90)>1)
							{
								PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,(Mission.y_ofs>90?0:180)*(Car.yaw2edge>0?1:-1),10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Moving1;
							}
							break;
						case Moving1:
							if(myabs(Mission.y_ofs-90)>1)
							{
								PID_Calc(&pidSpeed,myabs(cos(Mission.y_ofs)*Mission.x_ofs),0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,(myabs(initYaw)>90?180:0)*(Car.yaw2edge>0?1:-1),10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Steering2;
							}
							break;
						case Steering2:
							if(myabs(Car.yaw2edge-90)>1)
							{
								PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
								PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
								setCarParam(pidSpeed.U,pidYaw.U);
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Reversing;
							}
							break;
						case Moving2:
							if(myabs(Mission.x_ofs)>20||myabs(Mission.y_ofs-90)>1)
							{
								PID_Calc(&pidSpeed,Mission.x_ofs,50,10);		  //Data Calc
								PID_Calc(&pidYaw,Mission.y_ofs,90,10);	
								setCarParam(-pidSpeed.U,pidYaw.U);						//倒车所以速度是负的
							}
							else 
							{
								initX=Car.x2edge;
								initYaw=Car.yaw2edge;
								setCarParam(0,0);
								Mission.MissionStage=Steering3;
							}
							break;
						case Steering3:
							if(Mission.targetID==1)
							{
								if(myabs(Car.yaw2edge)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,0,10);	
									setCarParam(pidSpeed.U,pidYaw.U);
								}
								else 
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;
								}
							}
							else if(Mission.targetID==2)
							{
								if(myabs(myabs(Car.yaw2edge)-180)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,180*(Car.yaw2edge>0?1:-1),10);	
									setCarParam(pidSpeed.U,pidYaw.U);
								}
								else
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;								
								}
							}
							else if(Mission.targetID==0||Mission.targetID==3)
							{
								if(myabs(Car.yaw2edge-90)>1)
								{
									PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
									PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
									setCarParam(pidSpeed.U,pidYaw.U);
								}
								else 
								{
									initX=Car.x2edge;
									initYaw=Car.yaw2edge;
									setCarParam(0,0);
									Mission.MissionID=StandBy;
									Mission.MissionStage=StandBy;
									Mission.targetID=-1;			
								}
							}
							break;
					}	
				break;
			case Return:
				/////////////////////////////////////////////////////////////////////////////////////
				switch(Mission.MissionStage)		//start to reverse
				{
					case Standby:
						if(Mission.MissionStage!=Steering0)
								osDelay(1000);
						Mission.MissionStage=Steering0;
						if(myabs(Car.yaw2edge-90)>1)
						{
							PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
							PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
							setCarParam(pidSpeed.U,pidYaw.U);
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionStage=Reversing;
						}
						break;
					case Reversing:
						if(myabs(Car.x2edge-475)>20)
						{
							PID_Calc(&pidSpeed,Car.x2edge,475,10);		  //Data Calc
							PID_Calc(&pidYaw,Car.yaw2edge,90,10);	
							setCarParam(pidSpeed.U,pidYaw.U);
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionStage=Steering1;
						}
						break;
					case Steering1:						
						if(myabs(myabs(Car.yaw2edge)-90)>1)
						{
							PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
							PID_Calc(&pidYaw,Car.yaw2edge,(Mission.y_ofs>90?0:180)*(Car.yaw2edge>0?1:-1),10);	
							setCarParam(pidSpeed.U,pidYaw.U);
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionStage=Moving1;
						}
						break;
					case Moving1:
						if(myabs(Mission.y_ofs-90)>1)
						{
							PID_Calc(&pidSpeed,myabs(cos(Mission.y_ofs)*Mission.x_ofs),0,10);		  //Data Calc
							PID_Calc(&pidYaw,Car.yaw2edge,(myabs(initYaw)>90?180:0)*(Car.yaw2edge>0?1:-1),10);	
							setCarParam(pidSpeed.U,pidYaw.U);
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionStage=Steering2;
						}
						break;
					case Steering2:
						if(myabs(Car.yaw2edge+90)>1)
						{
							PID_Calc(&pidSpeed,0,0,10);		  //Data Calc
							PID_Calc(&pidYaw,Car.yaw2edge,-90,10);	
							setCarParam(pidSpeed.U,pidYaw.U);
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionStage=Reversing;
						}
						break;
					case Moving2:
						if(myabs(Mission.x_ofs)>20||myabs(Mission.y_ofs-90)>1)
						{
							PID_Calc(&pidSpeed,Mission.x_ofs,50,10);		  //Data Calc
							PID_Calc(&pidYaw,Mission.y_ofs,90,10);	
							setCarParam(pidSpeed.U,pidYaw.U);						//倒车所以速度是负的
						}
						else 
						{
							initX=Car.x2edge;
							initYaw=Car.yaw2edge;
							setCarParam(0,0);
							Mission.MissionID=StandBy;
							Mission.MissionStage=StandBy;
							Mission.targetID=-1;		
						}
						break;
					}	
				break;
		}		
				
		DataUpdate();
		
		osDelay(10);
		}
	
	for(;;)
  {
    osDelay(1);
  }
}

