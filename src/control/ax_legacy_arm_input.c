#include "ax_legacy_arm_input.h"
#include "ax_robot.h"
#include "openctr_features.h"

#if OPENCTR_ENABLE_LEGACY_ARM
void AX_LEGACY_ARM_Ps2Step(JOYSTICK_TypeDef my_joystick)
{
	static uint8_t btn1_up_flag = 0;
	static uint8_t btn1_down_flag = 0;	
	static uint8_t btn2_r1_flag = 0;	
	static uint8_t btn2_y_flag = 0;	
	static uint8_t btn2_b_flag = 0;	
	static uint8_t btn2_a_flag = 0;	
	static uint8_t btn2_x_flag = 0;	
	static uint16_t arm_spd = 15;  //机械臂速度
		//旧机械臂组合键：SELECT
		if(my_joystick.btn1 & PS2_BT1_SELECT)
		{	
			
			//R1按键，切换RGB灯效模式
			if(my_joystick.btn2 & PS2_BT2_R1)
			{
				btn2_r1_flag = 1;
			}
			else
			{
				if(btn2_r1_flag)
				{
					//灯光效果切换
					if(R_Light.M < LEFFECT6)
						R_Light.M++;
					else
						R_Light.M = LEFFECT1;
					
					//复位标记
					btn2_r1_flag = 0;
				}
			}
			
			//UP按键，机械臂加速
			if(my_joystick.btn1 & PS2_BT1_UP)
			{
				btn1_up_flag = 1;
			}
			else
			{
				if(btn1_up_flag)
				{
					//执行动作，
					if(arm_spd < 25)
					{
						arm_spd =  arm_spd + 5;
					}
					else
					{
						arm_spd = 25;
						
						//蜂鸣器鸣叫提示
						ax_beep_ring = BEEP_SHORT;					
					}
					
					//复位标记
					btn1_up_flag = 0;
				}
			}
			
			//DOWN按键，机械臂减速
			if(my_joystick.btn1 & PS2_BT1_DOWN)
			{
				btn1_down_flag = 1;
			}
			else
			{
				if(btn1_down_flag)
				{
					//执行动作
					if(arm_spd > 5)
					{
						arm_spd =  arm_spd -5;
					}
					else
					{
						arm_spd = 5;
						
						//蜂鸣器鸣叫提示
						ax_beep_ring = BEEP_SHORT;
					}
					
					//复位标记
					btn1_down_flag = 0;
				}
			}
			
			//Y按键(或三角按键)，动作1，行驶动作
			if(my_joystick.btn2 & PS2_BT2_Y)
			{
				btn2_y_flag = 1;
			}
			else
			{
				if(btn2_y_flag)
				{
					//执行动作
					ax_arm_angle[0] = ax_arm_set_angle[0][0];
					ax_arm_angle[1] = ax_arm_set_angle[0][1];
					ax_arm_angle[2] = ax_arm_set_angle[0][2];
					ax_arm_angle[3] = ax_arm_set_angle[0][3];
					ax_arm_angle[4] = ax_arm_set_angle[0][4];
					ax_arm_angle[5] = ax_arm_set_angle[0][5];	
					
					//复位标记
					btn2_y_flag = 0;
				}
			}			
			
			//B按键（或圆形按键），动作2，抓取地面物体动作
			if(my_joystick.btn2 & PS2_BT2_B)
			{
				btn2_b_flag = 1;
			}
			else
			{
				if(btn2_b_flag)
				{
					//执行动作
					ax_arm_angle[0] = ax_arm_set_angle[1][0];
					ax_arm_angle[1] = ax_arm_set_angle[1][1];
					ax_arm_angle[2] = ax_arm_set_angle[1][2];
					ax_arm_angle[3] = ax_arm_set_angle[1][3];
					ax_arm_angle[4] = ax_arm_set_angle[1][4];
					ax_arm_angle[5] = ax_arm_set_angle[1][5];	
					
					//复位标记
					btn2_b_flag = 0;
				}
			}
			
			//A按键（或叉号按键），动作3，后部放下动作
			if(my_joystick.btn2 & PS2_BT2_A)
			{
				btn2_a_flag = 1;
			}
			else
			{
				if(btn2_a_flag)
				{
					//执行动作
					ax_arm_angle[0] = ax_arm_set_angle[2][0];
					ax_arm_angle[1] = ax_arm_set_angle[2][1];
					ax_arm_angle[2] = ax_arm_set_angle[2][2];
					ax_arm_angle[3] = ax_arm_set_angle[2][3];
					ax_arm_angle[4] = ax_arm_set_angle[2][4];
					//ax_arm_angle[5] = ax_arm_set_angle[3][5];	
					
					//复位标记
					btn2_a_flag = 0;
				}
			}			
		
			//X按键（或方形按键），动作4，机械臂零位动作，机械臂直立向上
			if(my_joystick.btn2 & PS2_BT2_X)
			{
				btn2_x_flag = 1;
			}
			else
			{
				if(btn2_x_flag)
				{
					//执行动作
					ax_arm_angle[0] = ax_arm_set_angle[3][0];
					ax_arm_angle[1] = ax_arm_set_angle[3][1];
					ax_arm_angle[2] = ax_arm_set_angle[3][2];
					ax_arm_angle[3] = ax_arm_set_angle[3][3];
					ax_arm_angle[4] = ax_arm_set_angle[3][4];
					ax_arm_angle[5] = ax_arm_set_angle[3][5];		
					
					//复位标记
					btn2_x_flag = 0;
				}
			}	
		}
		else
		{
			//左右按键控制关节1
			if(my_joystick.btn1 & PS2_BT1_LEFT)
			{
				ax_arm_angle[0] += arm_spd; 
			}
			else if(my_joystick.btn1 & PS2_BT1_RIGHT)
			{
				ax_arm_angle[0] -= arm_spd; 
			}
			
			//上下按键控制关节2
			if(my_joystick.btn1 & PS2_BT1_DOWN)
			{
				ax_arm_angle[1] += arm_spd; 
			}
			else if(my_joystick.btn1 & PS2_BT1_UP)
			{
				ax_arm_angle[1] -= arm_spd; 
			}
			
			//L1 L2按键控制关节3
			if(my_joystick.btn2 & PS2_BT2_L1)
			{
				ax_arm_angle[2] += arm_spd; 
			}
			else if(my_joystick.btn2 & PS2_BT2_L2)
			{
				ax_arm_angle[2] -= arm_spd; 
			}
			
			//Y A按键控制关节4
			if(my_joystick.btn2 & PS2_BT2_Y)
			{
				ax_arm_angle[3] += arm_spd; 
			}
			else if(my_joystick.btn2 & PS2_BT2_A)
			{
				ax_arm_angle[3] -= arm_spd; 
			}
			
			//X B按键控制关节5
			if(my_joystick.btn2 & PS2_BT2_B)
			{
				ax_arm_angle[4] += arm_spd; 
			}
			else if(my_joystick.btn2 & PS2_BT2_X)
			{
				ax_arm_angle[4] -= arm_spd; 
			}
			
			//R1 R2按键控制关节6
			if(my_joystick.btn2 & PS2_BT2_R1)
			{
				ax_arm_angle[5] += arm_spd; 
			}
			else if(my_joystick.btn2 & PS2_BT2_R2)
			{
				ax_arm_angle[5] -= arm_spd; 
			}
		}			
}

void AX_LEGACY_ARM_AppFrame(const uint8_t comdata[16])
{
		//机臂模式控制帧
		if((comdata[0] == ID_BLERX_AM))
		{
			//控制机械臂
			ax_arm_angle[0] = (int16_t)(10.0*((int8_t)comdata[1]));
			ax_arm_angle[1] = (int16_t)(10.0*((int8_t)comdata[2]));
			ax_arm_angle[2] = (int16_t)(10.0*((int8_t)comdata[3]));
			ax_arm_angle[3] = (int16_t)(10.0*((int8_t)comdata[4]));
			ax_arm_angle[4] = (int16_t)(10.0*((int8_t)comdata[5]));
			ax_arm_angle[5] = (int16_t)(10.0*((int8_t)comdata[6]));
			
		}
		
}

void AX_LEGACY_ARM_AppHeld(const uint8_t comdata[16])
{
    uint16_t arm_spd;
	//手柄模式，按键控制机械臂
	if((comdata[0] == ID_BLERX_SB))
	{
		//通过K1-K4键调节速度，
		arm_spd = (comdata[6] & 0x0F) * 2;

		//左右按键控制关节1
		if(comdata[5] & APP_JOY1_LEFT)
		{
			ax_arm_angle[0] += arm_spd; 
		}
		else if(comdata[5] & APP_JOY1_RIGHT)
		{
			ax_arm_angle[0] -= arm_spd; 
		}
		
		//上下按键控制关节2
		if(comdata[5] & APP_JOY1_DOWN)
		{
			ax_arm_angle[1] += arm_spd; 
		}
		else if(comdata[5] & APP_JOY1_UP)
		{
			ax_arm_angle[1] -= arm_spd; 
		}
		
		//L1 L2按键控制关节3
		if(comdata[6] & APP_JOY2_L2)
		{
			ax_arm_angle[2] += arm_spd; 
		}
		else if(comdata[6] & APP_JOY2_L1)
		{
			ax_arm_angle[2] -= arm_spd; 
		}
		
		//Y A按键控制关节4
		if(comdata[5] & APP_JOY1_Y)
		{
			ax_arm_angle[3] += arm_spd; 
		}
		else if(comdata[5] & APP_JOY1_A)
		{
			ax_arm_angle[3] -= arm_spd; 
		}
		
		//X B按键控制关节5
		if(comdata[5] & APP_JOY1_B)
		{
			ax_arm_angle[4] += arm_spd; 
		}
		else if(comdata[5] & APP_JOY1_X)
		{
			ax_arm_angle[4] -= arm_spd; 
		}
		
		//R1 R2按键控制关节6
		if(comdata[6] & APP_JOY2_R1)
		{
			ax_arm_angle[5] += arm_spd; 
		}
		else if(comdata[6] & APP_JOY2_R2)
		{
			ax_arm_angle[5] -= arm_spd; 
		}		
	}	
}
#endif
