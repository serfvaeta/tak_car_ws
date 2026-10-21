#include "ax_ps2.h"
#include "ax_delay.h"
#include "ax_sys.h"


//PS2手柄的输入输出口
#define DI()     PEin(12)      //数据输入引脚

#define CMD_H()  PEout(10)=1   //命令位高
#define CMD_L()  PEout(10)=0   //命令位低

#define CS_H()   PEout(8)=1    //CS拉高(别名ATT)
#define CS_L()   PEout(8)=0    //CS拉低(别名ATT)

#define CLK_H()  PEout(7)=1    //时钟拉高
#define CLK_L()  PEout(7)=0    //时钟拉低


const  uint8_t PS2_cmnd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //请求获取数据命令                         
static uint8_t PS2_data[9] = {0};  //接收的数据


/**
  * @简  述  PS2初始化
  * @参  数  无
  * @返回值  无
  */
void AX_PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
	
	//DATA 信号从手柄到主机   输入口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//COMMMAND 信号从主机到手柄  输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//CS 手柄出发信号 信号在通信期间处于低电平	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//CLK 信号从主机到手柄   输出口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);	
	
	//关闭PS2手柄使能
	CS_H();
}


/**
  * @简  述  PS2数据读写函数
  * @参  数  cmd:要写入的命令
  * @返回值  读出数据
  */
static uint8_t PS2_ReadWriteData(uint8_t cmd)
{
	volatile uint8_t res = 0;
	volatile uint8_t ref;
	
	//写入命令，并读取一个1字节数据
	for(ref = 0x01; ref > 0x00; ref <<= 1)
	{
		////输出一位数据
		if(ref&cmd)
			CMD_H();
		else
			CMD_L();
		
		CLK_L();
		AX_Delayus(16);
		
		//读取一位数据
		if(DI())
			res |= ref; 
		CLK_H();
		AX_Delayus(16);		
	}

	//返回读出数据
    return res;	
}

/**
  * @简  述  PS2获取按键及摇杆数值。
  * @参  数  *JoystickStruct 手柄键值结构体
  * @返回值  无
  */
void AX_PS2_ScanKey(JOYSTICK_TypeDef *JoystickStruct)
{
	uint8_t i;
	
	//使能手柄
	CS_L();
	
	//读取PS2数据
	for(i=0; i<9; i++)
	{
		PS2_data[i] = PS2_ReadWriteData(PS2_cmnd[i]);
	}
	
	//关闭使能
	CS_H();

	//数值传递
	JoystickStruct->mode = PS2_data[1];
	JoystickStruct->btn1 = ~PS2_data[3];
	JoystickStruct->btn2 = ~PS2_data[4];
	JoystickStruct->RJoy_LR = PS2_data[5];
	JoystickStruct->RJoy_UD = PS2_data[6];
	JoystickStruct->LJoy_LR = PS2_data[7];
	JoystickStruct->LJoy_UD = PS2_data[8];
}


