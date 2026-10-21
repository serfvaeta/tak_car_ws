 

#include "stm32f4xx.h"
#include <stdio.h>
#include "ax_robot.h"
#include "openctr_features.h"
#include "ax_sg90_pan.h"
#include "ax_xursc.h"

#include "ax_oled_chinese.h" //OLED汉字库
#include "ax_oled_picture.h" //OLED 图片库

//任务句柄
//启动任务
#define START_TASK_PRIO		1
#define START_STK_SIZE 		128  
TaskHandle_t StartTask_Handler = NULL;
void Start_Task(void *pvParameters);

//机器人运行主任务
#define ROBOT_TASK_PRIO		3     
#define ROBOT_STK_SIZE 		256 
TaskHandle_t Robot_Task_Handle = NULL;
void Robot_Task(void *pvParameters);

//按键处理任务
#define KEY_TASK_PRIO		4     
#define KEY_STK_SIZE 		128   
TaskHandle_t Key_Task_Handle = NULL;
void Key_Task(void *pvParameters);

//RGB灯效任务
#define LIGHT_TASK_PRIO		5     
#define LIGHT_STK_SIZE 		128   
TaskHandle_t Light_Task_Handle = NULL;       
void Light_Task(void *pvParameters);

//OLED显示任务
#define DISP_TASK_PRIO		6     
#define DISP_STK_SIZE 		128   
TaskHandle_t Disp_Task_Handle = NULL;
void Disp_Task(void *pvParameters);

//琐事管理任务
#define TRIVIA_TASK_PRIO		10     
#define TRIVIA_STK_SIZE 		128   
TaskHandle_t Trivia_Task_Handle = NULL;
void Trivia_Task(void *pvParameters);

//PS2数据获取任务
#define PS2_TASK_PRIO		11     
#define PS2_STK_SIZE 		128   
#if OPENCTR_ENABLE_PS2_CONTROL
TaskHandle_t Ps2_Task_Handle = NULL;
void Ps2_Task(void *pvParameters);
#endif


/**
  * @简  述  程序主函数
  * @参  数  无
  * @返回值  无
  */
int main(void)
{	
	//uint8_t temp = 0x55;
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
	
	//原有机械臂舵机与 XURSC 的 PC6/PC7/TIM8 接线互斥。
#if OPENCTR_ENABLE_LEGACY_ARM
	AX_SERVO_S1_Init();
	AX_SERVO_S2_Init();
	AX_SERVO_S3456_Init();
#else
	AX_XURSC_PinsSafe(); //已接线模块保持 PC6 输入、PC7 低电平
	AX_SG90_PanPinsSafe(); //PB8 保持低电平，不向已安装的 SG90 发脉冲
#endif
	
	//电机初始化，频率20KHZ
	AX_MOTOR_AB_Init();
	AX_MOTOR_CD_Init();
	
	//延时函数初始化
	AX_DELAY_Init();  
	//SG90 收到 USB-COM 角度指令后才开始输出 PWM。
#if OPENCTR_ENABLE_XURSC
	AX_XURSC_Init();
#endif
	
	//LED初始化
	AX_LED_Init();  
	
	//KEY按键检测初始化
	AX_KEY_Init();
	
	//电池电压检测初始化
	AX_VIN_Init();
	
	//蜂鸣器初始化
	AX_BEEP_Init();    
	
	//调试串口初始化
	AX_UART1_Init(230400);
	
	//EXP2 USART2：单基站接收使用 115200；其他配置保持原有速率。
#if OPENCTR_ENABLE_AOA_RECEIVER
	AX_UART2_Init(115200);
#else
	AX_UART2_Init(230400);
#endif
	
	//USB通信端口初始化
	AX_UART3_Init(230400);
	
	//蓝牙串口初始化
#if OPENCTR_ENABLE_APP_CONTROL
	AX_UART4_Init(115200);
#endif
	
	//SBUS航模遥控器串口初始化
#if OPENCTR_ENABLE_SBUS_CONTROL
	AX_SBUS_Init();
#endif
	
	//CAN初始化
	AX_CAN_Init();
	
	//PS2手柄初始化
#if OPENCTR_ENABLE_PS2_CONTROL
	AX_PS2_Init();
#endif
	
	//编码器初始化，
	AX_ENCODER_A_Init();  
	AX_ENCODER_B_Init();  
	AX_ENCODER_C_Init();  
	AX_ENCODER_D_Init(); 
	
	//RGB彩灯
	AX_RGB_Init();	
	AX_RGB_SetFullColor(0x3f, 0x3f, 0x3f);
	
	//OLED屏幕初始化
	AX_OLED_Init();	
	AX_OLED_DispPicture(0, 0, 128, 8, PIC64X128_XTARK, 0); 
	
	
	//初始化拨码开光
	AX_SW_Init();
	
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
	
	//开机提示信息
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();
	AX_Delayms(1900);
	
    //MPU6050初始化
	AX_MPU6050_Init();      
	AX_MPU6050_SetAccRange(AX_ACC_RANGE_2G);    //设置加速度量程
	AX_MPU6050_SetGyroRange(AX_GYRO_RANGE_500); //设置陀螺仪量程
	AX_MPU6050_SetGyroSmplRate(200);            //设置陀螺仪采样率
	AX_MPU6050_SetDLPF(AX_DLPF_ACC94_GYRO98);   //设置低通滤波器带宽
	
	//创建AppTaskCreate任务
	xTaskCreate((TaskFunction_t )Start_Task,  /* 任务入口函数 */
								 (const char*    )"Start_Task",/* 任务名字 */
								 (uint16_t       )START_STK_SIZE,  /* 任务栈大小 */
								 (void*          )NULL,/* 任务入口函数参数 */
								 (UBaseType_t    )START_TASK_PRIO, /* 任务的优先级 */
								 (TaskHandle_t*  )&StartTask_Handler);/* 任务控制块指针 */ 
							
	//启动任务，开启调度						 
	vTaskStartScheduler(); 

	//循环
	while (1);
								 
}


/**
  * @简  介  创建任务函数
  * @参  数  无
  * @返回值  无
  */
void Start_Task(void *pvParameters)
{
	uint8_t cnt;
	uint8_t temp = 0x55;
	
	//陀螺仪校准变量
	int16_t gyro_data[3]; 
	
	/******机器人启动流程************************************************/
	
	//默认灯效参数
	R_Light.M  = LEFFECT2;
	R_Light.S  = 0;
	R_Light.T  = 0;
	R_Light.R  = 0x00;
	R_Light.G  = 0xFF;
	R_Light.B  = 0xFF;				
		
	//陀螺仪零点校准
	for(int i=0; i<10; i++) 
	{
		//红灯闪烁，指示陀螺仪校准
		AX_LED_Green_On();
		vTaskDelay(30); 
		
		AX_LED_Green_Off();
		vTaskDelay(20); 

		//获取PMU6050陀螺仪数据
        AX_MPU6050_GetGyroData(gyro_data);
		
		ax_imu_gyro_offset[0] += gyro_data[0];
		ax_imu_gyro_offset[1] += gyro_data[1];
		ax_imu_gyro_offset[2] += gyro_data[2]; 		
	}
	
	//计算平均偏差值
	ax_imu_gyro_offset[0] = -ax_imu_gyro_offset[0]/10;
	ax_imu_gyro_offset[1] = -ax_imu_gyro_offset[1]/10;
	ax_imu_gyro_offset[2] = -ax_imu_gyro_offset[2]/10;	
	
	
	//机械臂关节初始化，行驶动作
	ax_arm_angle[0] = ax_arm_set_angle[0][0];
	ax_arm_angle[1] = ax_arm_set_angle[0][1];
	ax_arm_angle[2] = ax_arm_set_angle[0][2];
	ax_arm_angle[3] = ax_arm_set_angle[0][3];
	ax_arm_angle[4] = ax_arm_set_angle[0][4];
	ax_arm_angle[5] = ax_arm_set_angle[0][5];	

	//开机等待，展示等待灯效（树莓派等待35S，其它等待3S）
	for(int i=0; i<3; i++) 
	{
		//呼吸效果, 1S 
		for(cnt= 0; cnt<30; cnt++)
		{
			 temp = cnt*8;
			 AX_RGB_SetFullColor( temp, 0, 0 );
			 AX_LED_Green_On();
			 vTaskDelay(20); 
		}
		for(cnt= 20; cnt>0; cnt--)
		{
			 temp = cnt*12;
			 AX_RGB_SetFullColor( temp, 0, 0 );
			 AX_LED_Green_Off();
			  vTaskDelay(20); 
		}		
	}
	
	//关闭，进入工作灯效
	AX_RGB_SetFullColor(0x00, 0x00, 0x00);
	
	//开机启动完成，绿灯点亮，蜂鸣器提示
	AX_LED_Green_On();	
	AX_BEEP_On();
	AX_Delayms(100);	
	AX_BEEP_Off();	
	
	//显示主窗口界面
	AX_OLED_ClearScreen();  //清除OLED启动画面显示
	AX_OLED_DispStr(0, 0, "   * TARKBOT     *   ", 0);	
	AX_OLED_DispStr(0, 1, "---------------------", 0);	

	AX_OLED_DispStr(0, 2, " Ver:V0.00 Mod:ROS   ", 0);
	AX_OLED_DispStr(30, 2, ROBOT_FW_VER, 0);		
	AX_OLED_DispStr(0, 3, " Vol:12.2V Gyz:00000 ", 0);
	AX_OLED_DispStr(0, 4, "---------------------", 0);	
	AX_OLED_DispStr(0, 5, "---------------------", 0);	
	AX_OLED_DispStr(0, 6, " MTA:00.00 MTB:-0.00 ", 0);
	AX_OLED_DispStr(0, 7, " MTC:00.00 MTD:-0.00 ", 0);
	
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
	
	//进入临界区
	taskENTER_CRITICAL();   

	//创建机器人控制任务
	xTaskCreate((TaskFunction_t )Robot_Task, /* 任务入口函数 */
			 (const char*    )"Robot_Task",/* 任务名字 */
			 (uint16_t       )ROBOT_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )ROBOT_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Robot_Task_Handle);/* 任务控制块指针 */
			 	 								 
	//创建按键处理任务
	xTaskCreate((TaskFunction_t )Key_Task, /* 任务入口函数 */
			 (const char*    )"Key_Task",/* 任务名字 */
			 (uint16_t       )KEY_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )KEY_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Key_Task_Handle);/* 任务控制块指针 */
			 
	//RGB灯效任务
	xTaskCreate((TaskFunction_t )Light_Task, /* 任务入口函数 */
			 (const char*    )"Light_Task",/* 任务名字 */
			 (uint16_t       )LIGHT_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )LIGHT_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Light_Task_Handle);/* 任务控制块指针 */			

	//OLED屏显示任务
	xTaskCreate((TaskFunction_t )Disp_Task, /* 任务入口函数 */
			 (const char*    )"Disp_Task",/* 任务名字 */
			 (uint16_t       )DISP_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )DISP_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Disp_Task_Handle);/* 任务控制块指针 */	
			 
	//琐事管理任务
	xTaskCreate((TaskFunction_t )Trivia_Task, /* 任务入口函数 */
			 (const char*    )"Trivia_Task",/* 任务名字 */
			 (uint16_t       )TRIVIA_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )TRIVIA_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Trivia_Task_Handle);/* 任务控制块指针 */	

	//PS2手柄数据读取任务
#if OPENCTR_ENABLE_PS2_CONTROL
	xTaskCreate((TaskFunction_t )Ps2_Task, /* 任务入口函数 */
			 (const char*    )"Ps2_Task",/* 任务名字 */
			 (uint16_t       )PS2_STK_SIZE,   /* 任务栈大小 */
			 (void*          )NULL,	/* 任务入口函数参数 */
			 (UBaseType_t    )PS2_TASK_PRIO,	    /* 任务的优先级 */
			 (TaskHandle_t*  )&Ps2_Task_Handle);/* 任务控制块指针 */				 
#endif
						  
	//删除AppTaskCreate任务				
	vTaskDelete(StartTask_Handler); 

	//退出临界区
	taskEXIT_CRITICAL();           
						 							
}


