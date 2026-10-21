/* Includes ------------------------------------------------------------------*/
#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_speed.h"

// 每个控制周期使用同一份三轴目标；版本用于阻止旧计算结果下发电机。
static int16_t target_ix, target_iy, target_iw;
static uint32_t target_version;


//函数定义
void ROBOT_MEC_Kinematics(void);   //麦轮模型
void ROBOT_FWD_Kinematics(void);   //四轮差速模型

void AX_ROBOT_Stop(void);   //机器人停止运动函数


/**
  * @简  述  机器人运动学处理，不同类型底盘进入不同处理函数
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{	
	
	//判断机器人运动是否开启
	if(ax_robot_move_enable == 0)
	{
		//目标速度设置为0
		AX_ROBOT_ClearTarget();
	}
	target_version = AX_ROBOT_GetTargetSnapshot(&target_ix, &target_iy, &target_iw);
	if(ax_robot_move_enable == 0) target_ix = target_iy = target_iw = 0;
	
	switch(ax_robot_type)
	{
		case ROBOT_MEC: 
			ROBOT_MEC_Kinematics();
			break;
		case ROBOT_FWD: 
			ROBOT_FWD_Kinematics();
			break;
		default: 
			AX_ROBOT_Stop();
			break;
	}
}


/**
  * @简  述  机器人停止运动状态
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Stop(void)
{
	
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed(0);
	AX_MOTOR_B_SetSpeed(0);  
	AX_MOTOR_C_SetSpeed(0);
	AX_MOTOR_D_SetSpeed(0);  
	
}

//R5/R10系列运动学处理
#if (defined ROBOT_R5) || (defined ROBOT_R10)
/**
  * @简  述  机器人运动学处理-麦克纳姆轮
  * @参  数  无
  * @返回值  无
  */
void ROBOT_MEC_Kinematics(void)
{

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float) ((int16_t)AX_ENCODER_A_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float)-((int16_t)AX_ENCODER_B_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float) ((int16_t)AX_ENCODER_C_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float)-((int16_t)AX_ENCODER_D_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = ((-R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT - R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/4/(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2))*1000;	
	
	//机器人目标速度限制
	if( target_ix > R_VX_LIMIT )    target_ix = R_VX_LIMIT;
	if( target_ix < (-R_VX_LIMIT))  target_ix = (-R_VX_LIMIT);
	if( target_iy > R_VY_LIMIT)     target_iy = R_VY_LIMIT;
	if( target_iy < (-R_VY_LIMIT))  target_iy = (-R_VY_LIMIT);
	if( target_iw > R_VW_LIMIT)     target_iw = R_VW_LIMIT;
	if( target_iw < (-R_VW_LIMIT))  target_iw = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = target_ix/1000.0;
	R_Vel.TG_FY = target_iy/1000.0;
	R_Vel.TG_FW = target_iw/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_C.TG = R_Vel.TG_FX + R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_D.TG = R_Vel.TG_FX - R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	//设置电机PWM值
	AX_ROBOT_CommitMotorSpeeds(target_version,
		-R_Wheel_A.PWM, -R_Wheel_B.PWM, R_Wheel_C.PWM, R_Wheel_D.PWM);
	
	//printf("A%f B%f C%f D%f \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT, MOTOR_C.Wheel_RT, MOTOR_D.Wheel_RT  );
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}


/**
  * @简  述  机器人运动学处理-四轮差速
  * @参  数  无
  * @返回值  无
  */
void ROBOT_FWD_Kinematics(void)
{

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float) ((int16_t)AX_ENCODER_A_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float)-((int16_t)AX_ENCODER_B_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float) ((int16_t)AX_ENCODER_C_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float)-((int16_t)AX_ENCODER_D_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/(2*FWD_WB_SCALE*FWD_WHEEL_BASE))*1000;	
	
	//机器人目标速度限制
	if( target_ix > R_VX_LIMIT )    target_ix = R_VX_LIMIT;
	if( target_ix < (-R_VX_LIMIT))  target_ix = (-R_VX_LIMIT);
	if( target_iy > R_VY_LIMIT)     target_iy = R_VY_LIMIT;
	if( target_iy < (-R_VY_LIMIT))  target_iy = (-R_VY_LIMIT);
	if( target_iw > R_VW_LIMIT)     target_iw = R_VW_LIMIT;
	if( target_iw < (-R_VW_LIMIT))  target_iw = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = target_ix/1000.0;
	R_Vel.TG_FY = 0;
	R_Vel.TG_FW = target_iw/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_C.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_D.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	if(target_ix ==0 && target_iw ==0)
	{
		R_Wheel_A.PWM = 0;   
		R_Wheel_B.PWM = 0;  
		R_Wheel_C.PWM = 0;  
		R_Wheel_D.PWM = 0;  	
	}
	
	//设置电机PWM值
	AX_ROBOT_CommitMotorSpeeds(target_version,
		-R_Wheel_A.PWM, -R_Wheel_B.PWM, R_Wheel_C.PWM, R_Wheel_D.PWM);
	
	//输出调试信息
	//printf("@%f  %f  %f %f  \r\n",R_Wheel_A.RT,R_Wheel_B.RT, R_Wheel_C.RT ,R_Wheel_D.RT);
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}
#endif

//TT马达系列运动学处理
#ifdef ROBOT_TT
/**
  * @简  述  机器人运动学处理-麦克纳姆轮
  * @参  数  无
  * @返回值  无
  */
void ROBOT_MEC_Kinematics(void)
{

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float)-((int16_t)AX_ENCODER_C_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float) ((int16_t)AX_ENCODER_D_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = ((-R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT - R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/4/(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2))*1000;	
	
	//机器人目标速度限制
	if( target_ix > R_VX_LIMIT )    target_ix = R_VX_LIMIT;
	if( target_ix < (-R_VX_LIMIT))  target_ix = (-R_VX_LIMIT);
	if( target_iy > R_VY_LIMIT)     target_iy = R_VY_LIMIT;
	if( target_iy < (-R_VY_LIMIT))  target_iy = (-R_VY_LIMIT);
	if( target_iw > R_VW_LIMIT)     target_iw = R_VW_LIMIT;
	if( target_iw < (-R_VW_LIMIT))  target_iw = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = target_ix/1000.0;
	R_Vel.TG_FY = target_iy/1000.0;
	R_Vel.TG_FW = target_iw/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_C.TG = R_Vel.TG_FX + R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_D.TG = R_Vel.TG_FX - R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	//设置电机PWM值
	AX_ROBOT_CommitMotorSpeeds(target_version,
		R_Wheel_A.PWM, -R_Wheel_B.PWM, R_Wheel_C.PWM, -R_Wheel_D.PWM);
	
	//printf("A%f B%f C%f D%f \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT, MOTOR_C.Wheel_RT, MOTOR_D.Wheel_RT  );
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}


/**
  * @简  述  机器人运动学处理-四轮差速
  * @参  数  无
  * @返回值  无
  */
void ROBOT_FWD_Kinematics(void)
{

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float)-((int16_t)AX_ENCODER_C_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float) ((int16_t)AX_ENCODER_D_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/(2*FWD_WB_SCALE*FWD_WHEEL_BASE))*1000;	
	
	//机器人目标速度限制
	if( target_ix > R_VX_LIMIT )    target_ix = R_VX_LIMIT;
	if( target_ix < (-R_VX_LIMIT))  target_ix = (-R_VX_LIMIT);
	if( target_iy > R_VY_LIMIT)     target_iy = R_VY_LIMIT;
	if( target_iy < (-R_VY_LIMIT))  target_iy = (-R_VY_LIMIT);
	if( target_iw > R_VW_LIMIT)     target_iw = R_VW_LIMIT;
	if( target_iw < (-R_VW_LIMIT))  target_iw = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = target_ix/1000.0;
	R_Vel.TG_FY = 0;
	R_Vel.TG_FW = target_iw/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_C.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_D.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	if(target_ix ==0 && target_iw ==0)
	{
		R_Wheel_A.PWM = 0;   
		R_Wheel_B.PWM = 0;  
		R_Wheel_C.PWM = 0;  
		R_Wheel_D.PWM = 0;  	
	}
	
	//设置电机PWM值
	AX_ROBOT_CommitMotorSpeeds(target_version,
		R_Wheel_A.PWM, -R_Wheel_B.PWM, R_Wheel_C.PWM, -R_Wheel_D.PWM);
	
	//输出调试信息
	//printf("@%f  %f  %f %f  \r\n",R_Wheel_A.RT,R_Wheel_B.RT, R_Wheel_C.RT ,R_Wheel_D.RT);
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}
#endif


