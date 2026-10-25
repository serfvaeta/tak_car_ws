/* Includes ------------------------------------------------------------------*/
#include "ax_control.h"
#include "ax_robot.h"
#include "ax_ps2_mode.h"
#include "ax_follow.h"
#include "ax_legacy_arm_input.h"
#include "openctr_features.h"

static void AX_CTL_Ps2StopChassis(void)
{
	AX_ROBOT_SetTargetIfActive(CTL_PS2, 0, 0, 0);
}

/**
  * @简  述  处理PS2手柄控制命令
  * @参  数  无
  * @返回值  无
  */
AX_PS2_InputAction AX_CTL_Ps2(JOYSTICK_TypeDef my_joystick)
{
#if !OPENCTR_ENABLE_LEGACY_ARM
    AX_PS2_InputAction action;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	action = AX_PS2_ModeUpdate(
		my_joystick.mode == 0x73,
		(my_joystick.btn1 & PS2_BT1_SELECT) != 0);
	__set_PRIMASK(primask);
	if(action == AX_PS2_INPUT_MODE_CHANGED)
		ax_beep_ring = BEEP_SHORT;
	if(action != AX_PS2_INPUT_FOLLOW)
		AX_FOLLOW_ModeReset();
	if(action == AX_PS2_INPUT_FOLLOW)
	{
		AX_FOLLOW_Step();
		return action;
	}
	if(action == AX_PS2_INPUT_SCAN ||
	   action == AX_PS2_INPUT_FOLLOW_AVOID ||
	   action == AX_PS2_INPUT_FOLLOW_AVOID2)
	{
		AX_CTL_Ps2StopChassis();
		return action;
	}
	if(action != AX_PS2_INPUT_DRIVE)
	{
		AX_CTL_Ps2StopChassis();
		return action;
	}
#endif

	static uint8_t btn_joyl_flag = 0;
	static uint8_t btn_joyr_flag = 0;
	static uint8_t speed = 4;  //底盘速度
	
	//红绿灯模式下，执行控制操作
	if(my_joystick.mode ==  0x73)
	{
		AX_ROBOT_SetTargetIfActive(CTL_PS2,
			(int16_t)(speed*(0x80 - my_joystick.RJoy_UD)),
			(int16_t)(speed*(0x80 - my_joystick.RJoy_LR)),
			(int16_t)(4*speed*(0x80 - my_joystick.LJoy_LR)));
		
#if OPENCTR_ENABLE_LEGACY_ARM
		AX_LEGACY_ARM_Ps2Step(my_joystick);
#endif
		
		//左摇杆按键，减速
		if(my_joystick.btn1 & PS2_BT1_JOY_L)
		{
			btn_joyl_flag = 1;
		}
		else
		{
			if(btn_joyl_flag)
			{
				//底盘运动速度减小
				if(speed > 1)
				{
					speed--;
				}
				else
				{
					speed = 1;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;
				}
				
				//复位标记
				btn_joyl_flag = 0;
			}
		}
		
		//右摇杆按键，加速
		if(my_joystick.btn1 & PS2_BT1_JOY_R)
		{
			btn_joyr_flag = 1;
		}
		else
		{
			if(btn_joyr_flag)
			{
				
				//底盘运动速度增加
				if(speed < 8)
				{
					speed++;
				}
				else
				{
					speed = 8;
					
					//蜂鸣器鸣叫提示
					ax_beep_ring = BEEP_SHORT;					
				}				
				
				//复位标记
				btn_joyr_flag = 0;
			}
		}
		return AX_PS2_INPUT_DRIVE;
	}
	else
	{
		AX_CTL_Ps2StopChassis();
		return AX_PS2_INPUT_INVALID;
	}
}	

/**
  * @简  述  处理手机APP控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_App(void)
{
	static uint8_t  comdata[16];
	
	//接收蓝牙APP串口数据
	if(AX_UART4_GetData(comdata))
	{
		//摇杆模式运动控制帧
		if((comdata[0] == ID_BLERX_YG))
		{
			AX_ROBOT_SetTargetIfActive(CTL_APP,
				(int16_t)(  6*(int8_t)comdata[4] ),
				(int16_t)( -6*(int8_t)comdata[3] ),
				(int16_t)(-20*(int8_t)comdata[1] ));
		}
		
		//手柄模式运动控制帧
		if((comdata[0] == ID_BLERX_SB))
		{
			AX_ROBOT_SetTargetIfActive(CTL_APP,
				(int16_t)(  6*(int8_t)comdata[4] ),
				(int16_t)( -6*(int8_t)comdata[3] ),
				(int16_t)(-20*(int8_t)comdata[1] ));			
		}
		
		//重力模式运动控制帧
		if(comdata[0] == ID_BLERX_ZL)
		{
			AX_ROBOT_SetTargetIfActive(CTL_APP,
				(int16_t)( -6*(int8_t)comdata[3] ),
				0,
				(int16_t)(-30*(int8_t)comdata[2] ));
		}
		
#if OPENCTR_ENABLE_LEGACY_ARM
		AX_LEGACY_ARM_AppFrame(comdata);
#endif
		//灯光效果控制帧
		if(comdata[0] == ID_BLERX_LG)
		{
			R_Light.M  = comdata[1];
			R_Light.S  = comdata[2];
			R_Light.T  = comdata[3];
			R_Light.R  = comdata[4];
			R_Light.G  = comdata[5];
			R_Light.B  = comdata[6];
		}
	}
	
#if OPENCTR_ENABLE_LEGACY_ARM
	AX_LEGACY_ARM_AppHeld(comdata);
#endif
}

/**
  * @简  述  处理SBUS航模遥控器控制命令
  * @参  数  无
  * @返回值  无
  */
void AX_CTL_RemoteSbus(void)
{
	static uint16_t sbusdata[8];
	static float speed;
	
	//获取SBUS解码数据
	if(AX_SBUS_GetRxData(sbusdata))
	{
		//SWD-8（通道7）拨杆到上方生效
		if(sbusdata[7] < 500)
		{
			//SWA-5拨杆设置速度
			if(sbusdata[4] < 500)          speed = 1.5;
			else if(sbusdata[4] < 1000)    speed = 1.0;
			else                           speed = 0.7;
			
			//运动控制
			AX_ROBOT_SetTargetIfActive(CTL_RMS,
				(int16_t)(  speed * (sbusdata[1] - 992)),
				(int16_t)( -speed * (sbusdata[0] - 992)),
				(int16_t)( -5.0 * speed * (sbusdata[3] - 992)));
		}
		
		//查看各个通道数值
		//printf("c1=%04d c2=%04d c3=%04d ch4=%04d ch5=%04d ch6=%04d ch7=%04d ch8=%04d \r\n",sbusdata[0],
	    //       sbusdata[1],sbusdata[2],sbusdata[3],sbusdata[4],sbusdata[5],sbusdata[6],sbusdata[7]);
	}
}
