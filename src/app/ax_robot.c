/* Includes ------------------------------------------------------------------*/
#include "ax_robot.h"
#include "openctr_features.h"
#include "ax_range_pan.h"
#include "ax_light.h"
#include "ax_kinematics.h"
#include "ax_control.h"
#include "ax_legacy_arm.h"
#include "ax_ps2_mode.h"
#include "ax_follow_avoid.h"
#include "ax_follow_avoid2.h"
#include "ax_aoa.h"

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人轮子数据
ROBOT_Wheel  R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D;

//机器人IMU数据
ROBOT_Imu  R_Imu;

//机器人RGB数据
ROBOT_Light  R_Light;

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人电池电压数据
uint16_t R_Bat_Vol;  

//IMU数据
int16_t ax_imu_acc_data[3];  
int16_t ax_imu_gyro_data[3]; 
int16_t ax_imu_gyro_offset[3]; 

//电机PID控制参数
int16_t ax_motor_kp=800;      
int16_t ax_motor_kd=1000; 

//机器人型号
uint8_t ax_robot_type = ROBOT_MEC;

//机器人运动使能开关,默认打开状态
uint8_t ax_robot_move_enable = 1;

//灯光开关失能，默认打开状态
uint8_t ax_light_enable = 1;

//蜂鸣器鸣叫一声
uint8_t ax_beep_ring = 0;

//PS2手柄键值结构体
JOYSTICK_TypeDef my_joystick;  
#if OPENCTR_ENABLE_PS2_CONTROL
static TickType_t ps2_last_scan_tick;
#endif

//控制方式选择
volatile uint8_t ax_control_mode = CTL_ROS;
static uint32_t ax_target_version;

// 控制来源变更时清除上一来源的目标速度；可由任务或串口中断调用。
void AX_ROBOT_SelectControlSource(uint8_t source)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
		if(ax_control_mode != source)
		{
			if(ax_control_mode == CTL_PS2 || source == CTL_PS2)
			AX_PS2_ModeReset();
		R_Vel.TG_IX = 0;
		R_Vel.TG_IY = 0;
		R_Vel.TG_IW = 0;
		ax_control_mode = source;
		ax_target_version++;
		AX_MOTOR_A_SetSpeed(0);
		AX_MOTOR_B_SetSpeed(0);
		AX_MOTOR_C_SetSpeed(0);
		AX_MOTOR_D_SetSpeed(0);
	}
	__set_PRIMASK(primask);
}

// 仅当前来源仍有效时提交三轴目标，防止旧任务快照覆盖新来源。
void AX_ROBOT_SetTargetIfActive(uint8_t source, int16_t x, int16_t y, int16_t w)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if(ax_control_mode == source)
	{
		R_Vel.TG_IX = x;
		R_Vel.TG_IY = y;
		R_Vel.TG_IW = w;
		ax_target_version++;
	}
	__set_PRIMASK(primask);
}

void AX_ROBOT_ClearTarget(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	R_Vel.TG_IX = 0;
	R_Vel.TG_IY = 0;
	R_Vel.TG_IW = 0;
	ax_target_version++;
	__set_PRIMASK(primask);
}

uint32_t AX_ROBOT_GetTargetSnapshot(int16_t *x, int16_t *y, int16_t *w)
{
	uint32_t primask = __get_PRIMASK();
	uint32_t version;
	__disable_irq();
	*x = R_Vel.TG_IX;
	*y = R_Vel.TG_IY;
	*w = R_Vel.TG_IW;
	version = ax_target_version;
	__set_PRIMASK(primask);
	return version;
}

void AX_ROBOT_CommitMotorSpeeds(uint32_t version, int16_t a, int16_t b, int16_t c, int16_t d)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if(version != ax_target_version || ax_robot_move_enable == 0 ||
	   (ax_control_mode == CTL_PS2 && AX_PS2_ModeGet() == AX_PS2_MODE_SCAN))
		a = b = c = d = 0;
	AX_MOTOR_A_SetSpeed(a);
	AX_MOTOR_B_SetSpeed(b);
	AX_MOTOR_C_SetSpeed(c);
	AX_MOTOR_D_SetSpeed(d);
	__set_PRIMASK(primask);
}

//函数定义
void ROBOT_IMUHandle(void);     //IMU数据处理
void ROBOT_SendDataToRos(void);  //发送数据
	
/**
  * @简  述  机器人管理任务
  * @参  数  无
  * @返回值  无
  */
void Robot_Task(void* parameter)
{
	uint8_t control_source;
#if OPENCTR_ENABLE_PS2_CONTROL
	JOYSTICK_TypeDef ps2_snapshot;
	TickType_t ps2_snapshot_tick;
	AX_PS2_InputAction ps2_action;
	static uint8_t avoid_pan_was_enabled;
#endif

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement );
		
		taskENTER_CRITICAL();
		control_source = ax_control_mode;
#if OPENCTR_ENABLE_PS2_CONTROL
		ps2_action = AX_PS2_INPUT_INVALID;
		if(control_source == CTL_PS2)
		{
			ps2_snapshot = my_joystick;
			ps2_snapshot_tick = ps2_last_scan_tick;
		}
#endif
		taskEXIT_CRITICAL();
		
		//控制方式选择
#if OPENCTR_ENABLE_PS2_CONTROL || OPENCTR_ENABLE_APP_CONTROL || OPENCTR_ENABLE_SBUS_CONTROL
		//PS2 扫描任务若超过 200 ms 没有更新，按无效输入停车。
#if OPENCTR_ENABLE_PS2_CONTROL
		if(control_source == CTL_PS2 &&
		   (TickType_t)(xTaskGetTickCount() - ps2_snapshot_tick) > pdMS_TO_TICKS(200))
			ps2_snapshot.mode = 0;
#endif
		if(control_source != 0)
		{
#if OPENCTR_ENABLE_PS2_CONTROL
			if      (control_source == CTL_PS2)    ps2_action = AX_CTL_Ps2(ps2_snapshot); //PS2手柄控制
#endif
#if OPENCTR_ENABLE_APP_CONTROL
			if      (control_source == CTL_APP)    AX_CTL_App();    //APP控制
#endif
#if OPENCTR_ENABLE_SBUS_CONTROL
			if      (control_source == CTL_RMS)    AX_CTL_RemoteSbus();   //SBUS航模遥控器控制
#endif
		}
#endif
		
		AX_RANGE_PAN_SetManualAllowed(control_source != CTL_PS2);
#if OPENCTR_ENABLE_PS2_CONTROL
		{
			AX_PS2_Mode ps2_mode = AX_PS2_ModeGet();
			uint8_t avoid_pan_enabled = control_source == CTL_PS2 &&
				(ps2_mode == AX_PS2_MODE_FOLLOW_AVOID ||
				 ps2_mode == AX_PS2_MODE_FOLLOW_AVOID2);
			/* Force a fresh pan/range generation between the two avoid modes. */
			if(avoid_pan_enabled && avoid_pan_was_enabled &&
			   ps2_action == AX_PS2_INPUT_MODE_CHANGED)
				AX_RANGE_PAN_SetAvoidEnabled(0);
			AX_RANGE_PAN_SetAvoidEnabled(avoid_pan_enabled);
			avoid_pan_was_enabled = avoid_pan_enabled;
		}
#else
		AX_RANGE_PAN_SetAvoidEnabled(0);
#endif
		AX_RANGE_PAN_SetScanEnabled(control_source == CTL_PS2 &&
			AX_PS2_ModeGet() == AX_PS2_MODE_SCAN);
#if OPENCTR_ENABLE_PS2_CONTROL
		if(control_source == CTL_PS2)
			AX_RANGE_PAN_ScanButtons(
				(ps2_snapshot.btn2 & PS2_BT2_X) != 0,
				(ps2_snapshot.btn2 & PS2_BT2_B) != 0);
#endif
		AX_RANGE_PAN_TaskStep();
		AX_AOA_TaskStep();
		ROBOT_IMUHandle();
#if OPENCTR_ENABLE_PS2_CONTROL
		if(control_source == CTL_PS2 &&
		   ps2_action == AX_PS2_INPUT_FOLLOW_AVOID)
			AX_FOLLOW_AVOID_Step();
		else
			AX_FOLLOW_AVOID_Reset();
		if(control_source == CTL_PS2 &&
		   ps2_action == AX_PS2_INPUT_FOLLOW_AVOID2)
			AX_FOLLOW_AVOID2_Step();
		else
			AX_FOLLOW_AVOID2_Reset();
#endif

		//机器人运动学处理
		AX_ROBOT_Kinematics();

		//只有原有机械臂启用时才写 S1 和 TIM8 的四路舵机输出。
#if OPENCTR_ENABLE_LEGACY_ARM
		JOINT_Control();
#endif

		//数据发送
		ROBOT_SendDataToRos();
	}
}

/**
  * @简  述  机器人IMU数据处理
  * @参  数  无
  * @返回值  无
  */
void ROBOT_IMUHandle(void)
{
		
	//获取PMU6050加速度数据
	AX_MPU6050_GetAccData(ax_imu_acc_data);
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.ACC_X =  ax_imu_acc_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.ACC_Y = -ax_imu_acc_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.ACC_Z =  ax_imu_acc_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//获取PMU6050陀螺仪数据
	AX_MPU6050_GetGyroData(ax_imu_gyro_data);
	
	//陀螺仪加入零票校准数据
	ax_imu_gyro_data[0] += ax_imu_gyro_offset[0];
	ax_imu_gyro_data[1] += ax_imu_gyro_offset[1];
	ax_imu_gyro_data[2] += ax_imu_gyro_offset[2];
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.GYRO_X =  ax_imu_gyro_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.GYRO_Y = -ax_imu_gyro_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.GYRO_Z =  ax_imu_gyro_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//观察数据
    //printf("@ %d  \r\n",R_Imu.ACC_X);
}


/**
  * @简  述  机器人发送数据到ROS
  * @参  数  无
  * @返回值  无
  */
void ROBOT_SendDataToRos(void)
{
    //串口发送数据
	static uint8_t comdata[20]; 	

	//加速度 = (ax_acc/32768) * 2G  
	comdata[0] = (u8)( R_Imu.ACC_X >> 8 );  
	comdata[1] = (u8)( R_Imu.ACC_X );
	comdata[2] = (u8)( R_Imu.ACC_Y >> 8 );
	comdata[3] = (u8)( R_Imu.ACC_Y );
	comdata[4] = (u8)( R_Imu.ACC_Z >> 8 );
	comdata[5] = (u8)( R_Imu.ACC_Z );
	
	//陀螺仪角速度 = (ax_gyro/32768) * 500
	comdata[6] = (u8)( R_Imu.GYRO_X >> 8 );
	comdata[7] = (u8)( R_Imu.GYRO_X );
	comdata[8] = (u8)( R_Imu.GYRO_Y >> 8 );
	comdata[9] = (u8)( R_Imu.GYRO_Y );
	comdata[10] = (u8)( R_Imu.GYRO_Z	>> 8 );
	comdata[11] = (u8)( R_Imu.GYRO_Z );
	
	//机器人速度值 单位为m/s，放大1000倍
	comdata[12] = (u8)( R_Vel.RT_IX >> 8 );
	comdata[13] = (u8)( R_Vel.RT_IX );
	comdata[14] = (u8)( R_Vel.RT_IY >> 8 );
	comdata[15] = (u8)( R_Vel.RT_IY );
	comdata[16] = (u8)( R_Vel.RT_IW >> 8 );
	comdata[17] = (u8)( R_Vel.RT_IW );
	
	//电池电压
	comdata[18] = (u8)( R_Bat_Vol >> 8 );
	comdata[19] = (u8)( R_Bat_Vol );

	//单基站占用 USART2 时，只从 USB-COM 发送遥测，避免向基站 RX 注入旧协议。
#if !OPENCTR_ENABLE_AOA_RECEIVER
    AX_UART2_SendPacket(comdata, 20, ID_UTX_DATA);		
#endif
	
	//USB串口发送数据
    AX_UART3_SendPacket(comdata, 20, ID_UTX_DATA);		
	
	//CAN数据发送
	AX_CAN_SendPacket(ID_CANTX_MSG1, 6, &comdata[0]);
	AX_CAN_SendPacket(ID_CANTX_MSG2, 6, &comdata[6]);
	AX_CAN_SendPacket(ID_CANTX_MSG3, 8, &comdata[12]);
}

/**
  * @简  述  琐事管理任务
  * @参  数  无
  * @返回值  无
  */
void Trivia_Task(void* parameter)
{	
	//uint8_t i;	
	
	//计数变量
    static uint16_t ax_bat_vol_cnt = 0; 
	
	while (1)
	{	
		
		/*****电池管理***********************************/
		
		//采集电池电压
	    R_Bat_Vol = AX_VIN_GetVol_X100();
		
		//调试输出电池电压数据
        //printf("@ %d  \r\n",R_Bat_Vol);		
		
		//电量低于40%
		if(R_Bat_Vol < VBAT_40P)  
		{
			//红灯开始闪烁警示
			AX_LED_Red_Toggle();
			
			//电量低于20%
			if(R_Bat_Vol < VBAT_20P)
			{
				//红灯常亮
				AX_LED_Red_On();
				
				//电量低于10%，关闭系统进入保护状态
				if(R_Bat_Vol < VBAT_10P) 
				{
					//低压时间计数
					ax_bat_vol_cnt++;
					
					//超过10次，进入关闭状态
					if(ax_bat_vol_cnt > 10 )
					{
						AX_RANGE_PAN_SetManualAllowed(0);
						AX_RANGE_PAN_SetScanEnabled(0);
						//关闭绿灯，红灯常亮
						AX_LED_Green_Off();
						AX_LED_Red_On();
						
						//任务挂起
						vTaskSuspend(Robot_Task_Handle);
						vTaskSuspend(Disp_Task_Handle);
						
						//电机速度设置为0
						AX_MOTOR_A_SetSpeed(0);
						AX_MOTOR_B_SetSpeed(0);  
						AX_MOTOR_C_SetSpeed(0);
						AX_MOTOR_D_SetSpeed(0); 
						
						//清除OLED启动画面显示
						AX_OLED_ClearScreen();  //
						
						//蜂鸣器鸣叫报警
						while(1)
						{	
							//显示机器人停止信息
							AX_OLED_DispStr(0, 3, "     Low power      ", 0);	
							AX_OLED_DispStr(0, 5, "  Robot has stopped ", 0);	
							AX_BEEP_On();
							vTaskDelay(30);
							AX_BEEP_Off();
							
							vTaskDelay(1000);	
							AX_OLED_ClearScreen(); 
							vTaskDelay(1000);						
						}								
					}
				}
				else
				{
					ax_bat_vol_cnt = 0;
				}				
			}
		}
		else
		{
			//红灯关闭
			AX_LED_Red_Off();
		}
		
		/*****蜂鸣器鸣叫一声***********************************/
		if(ax_beep_ring != 0)
		{
			if(ax_beep_ring == BEEP_SHORT)
			{
				//短鸣叫一声
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;
			}
			else //
			{
				//长鸣叫一声
				AX_BEEP_On();
				vTaskDelay(1000); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;				
			}
		}
		
		/*****底盘类型检测***********************************/
		//通过拨码开关1获取机器人底盘类型
		if(AX_SW_GetS1() == 0)  //拨码在上方
		{
			//麦轮底盘
			ax_robot_type = ROBOT_MEC;
		}
		else
		{
			//四轮差速底盘
			ax_robot_type = ROBOT_FWD;
		}
		
		//LED系统心跳指示
		AX_LED_Green_Toggle();	
		
		
        //循环周期500ms
		vTaskDelay(500); 
	}			
}

/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Key_Task(void* parameter)
{	
	uint8_t  i;
	
	//支持短按，中按（3S），长按（10S）三种动作检测
	while (1)
	{		
		//按键扫描
		if(AX_KEY_Scan() != 0)
		{
			//软件延时
			vTaskDelay(50);  
			
			//确定按键按下
			if(AX_KEY_Scan() != 0)
			{
				//等待按键抬起
				for(i=0; i<200; i++)
				{

					vTaskDelay(50);
					
					if(AX_KEY_Scan() == 0)
					{
						break;
					}
					
					//按键时长3S时，给出提示音
					if(i == 60)
					{
						AX_BEEP_On();
						vTaskDelay(200);
						AX_BEEP_Off();
					}
				}

				//短按检测,小于1S
				if(i < 20)
				{
					//灯光效果切换
					if(R_Light.M < LEFFECT6)
						R_Light.M++;
					else
						R_Light.M = LEFFECT1;
				}
				
				//中按检测,大于3S，小于10S
				if(i>60 && i<200)
				{
					//暂无动作
				}				
				
					
				//长按检测，大于10S
				if(i == 200)
				{
					//暂无动作
									
				}
			}
		}
		
		//循环周期
		vTaskDelay(50);    
	}			
}


/**
  * @简  述  灯光显示任务
  * @参  数  无
  * @返回值  无
  */
void Light_Task(void* parameter)
{	
	
	while (1)
	{	
        //延时30ms，30HZ显示频率		
		vTaskDelay(30); 
		
		//判断前灯是否打开
		if(ax_light_enable != 0)
		{
			//正常显示
			AX_LIGHT_Show();
		}
		else
		{
			//关闭状态
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}


/**
  * @简  述  OLED屏幕显示任务
  * @参  数  无
  * @返回值  无
  */
void Disp_Task(void* parameter)
{	
	while (1)
	{	
		//延时	
		vTaskDelay(100); 
		
		//显示控制模式
		if(ax_control_mode)
		{
			if      (ax_control_mode == CTL_PS2)    AX_OLED_DispStr(90, 2, "PS2", 0);   //PS2手柄控制
			else if (ax_control_mode == CTL_APP)    AX_OLED_DispStr(90, 2, "APP", 0);   //APP控制
			else if (ax_control_mode == CTL_RMS)    AX_OLED_DispStr(90, 2, "RMS", 0);   //SBUS航模遥控器控制			
		}
		else
		{
			//ROS控制模式
			AX_OLED_DispStr(90, 2, "ROS", 0);
		}
		if(ax_control_mode == CTL_PS2)
		{
			if(AX_PS2_ModeGet() == AX_PS2_MODE_DRIVE)
				AX_OLED_DispStr(0, 4, "Mode: DRIVE          ", 0);
			else if(AX_PS2_ModeGet() == AX_PS2_MODE_FOLLOW)
				AX_OLED_DispStr(0, 4, "Mode: FOLLOW         ", 0);
			else if(AX_PS2_ModeGet() == AX_PS2_MODE_FOLLOW_AVOID)
				AX_OLED_DispStr(0, 4, "Mode: AVOID          ", 0);
			else if(AX_PS2_ModeGet() == AX_PS2_MODE_FOLLOW_AVOID2)
				AX_OLED_DispStr(0, 4, "Mode: AVOID2         ", 0);
			else
				AX_OLED_DispStr(0, 4, "Mode: SCAN           ", 0);
		}
		else AX_OLED_DispStr(0, 4, "---------------------", 0);
		
		//显示底盘类型
		switch(ax_robot_type)
		{
			case ROBOT_MEC: 
				AX_OLED_DispStr(78, 0, "MEC", 0);	
				break;
			case ROBOT_FWD: 
				AX_OLED_DispStr(78, 0, "FWD", 0);	
				break;
			default: 
				AX_OLED_DispStr(78, 0, "ERR", 0);	
				break;
		}
		
		//显示电池电压，陀螺仪Z轴数据,舵机偏角,
		AX_OLED_DispValue(30, 3, (R_Bat_Vol*0.1), 2, 1, 0);
		AX_OLED_DispValue(90, 3, (R_Imu.GYRO_Z ), 6, 0, 0);	
		
		vTaskDelay(100); 
		//显示轮子实时速度
		AX_OLED_DispValue(30, 6, (R_Wheel_A.RT*100), 2, 2, 0);
		AX_OLED_DispValue(90, 6, (R_Wheel_B.RT*100), 2, 2, 0);
		AX_OLED_DispValue(30, 7, (R_Wheel_C.RT*100), 2, 2, 0);
		AX_OLED_DispValue(90, 7, (R_Wheel_D.RT*100), 2, 2, 0);	
	}			
}

/**
  * @简  述  PS2数据获取任务
  * @参  数  无
  * @返回值  无
  */
#if OPENCTR_ENABLE_PS2_CONTROL
void Ps2_Task(void* parameter)
{
	JOYSTICK_TypeDef sample;

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime1;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement1 = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime1 = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime1, TimeIncrement1 );
		
		//读取PS2手柄键值
		AX_PS2_ScanKey(&sample);
		taskENTER_CRITICAL();
		my_joystick = sample;
		ps2_last_scan_tick = xTaskGetTickCount();
		taskEXIT_CRITICAL();
		
		//不在PS2控制模式下
		if(ax_control_mode != CTL_PS2)
		{
			//判断是否开启PS2手柄控制
			//START按键被按下后，左边摇杆上推，进入PS2控制模式
			if((sample.mode == 0x73) &&
			   (sample.btn1 == PS2_BT1_START) && (sample.LJoy_UD == 0x00))
			{
				//切换到PS2模式
				AX_ROBOT_SelectControlSource(CTL_PS2);	

				//执行蜂鸣器鸣叫提示
				ax_beep_ring = BEEP_SHORT;
			}
		}
		
//		//打印手柄键值
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		my_joystick.mode, my_joystick.btn1, my_joystick.btn2, 
//		my_joystick.RJoy_LR, my_joystick.RJoy_UD, my_joystick.LJoy_LR, my_joystick.LJoy_UD);
	}
}

#endif
