#include "ax_uart3.h"
#include <stdio.h>

#include "ax_robot.h"
#include "ax_range_pan.h"
#include "openctr_features.h"

static uint8_t uart3_rx_con=0;       //接收计数器
static uint8_t uart3_rx_checksum;    //帧头部分校验和
static uint8_t uart3_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte
static uint8_t uart3_tx_buf[40];     //发送缓冲

/**
  * @简  述  UART   串口初始化
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
void AX_UART3_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;


	/* 串口USART配置 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	//USART对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 

	//USART 端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 

	//USART参数配置
	USART_InitStructure.USART_BaudRate = baud;    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	//USART使能
	USART_Cmd(USART3, ENABLE); 
	
	
	//开启串口接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	
    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
	
}

/**
  * @简  述  UART 串口中断服务函数
  * @参  数  无 
  * @返回值  无
  */
void USART3_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART3);	
		
		if(uart3_rx_con < 3)    //==接收帧头 + 长度
		{
			if(uart3_rx_con == 0)  //接收帧头1 0xAA
			{
				if(Res == 0xAA)
				{
					uart3_rx_buf[0] = Res;
					uart3_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart3_rx_con == 1) //接收帧头2 0x55
			{
				if(Res == 0x55)
				{
					uart3_rx_buf[1] = Res;
					uart3_rx_con = 2;
				}
				else
				{
					uart3_rx_con = 0;						
				}				
			}
			else  //接收数据长度
			{
				if(Res >= 5 && Res <= sizeof(uart3_rx_buf))
				{
					uart3_rx_buf[2] = Res;
					uart3_rx_con = 3;
					uart3_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
				}
				else uart3_rx_con = 0;
			}
		}
		else    //==接收数据
		{
			if(uart3_rx_con < (uart3_rx_buf[2]-1) )
			{
				uart3_rx_buf[uart3_rx_con] = Res;
				uart3_rx_con++;
				uart3_rx_checksum = uart3_rx_checksum + Res;					
			}
			else    //判断最后1位
			{
				//接收完成，恢复初始状态
				uart3_rx_con = 0;	
				
				//数据校验
				if( Res == uart3_rx_checksum )  //校验正确
				{	
					
					//独立 SG90 指令不切换底盘控制模式。
#if OPENCTR_ENABLE_SG90_PAN
					if(uart3_rx_buf[2] == 7 && uart3_rx_buf[3] == ID_URX_SG90_PAN)
					{
						AX_RANGE_PAN_RequestAngle((int16_t)((uart3_rx_buf[4]<<8) | uart3_rx_buf[5]));
					}
					else
#endif
					//速度控制帧
					if(uart3_rx_buf[2] == 11 && uart3_rx_buf[3] == ID_URX_VEL)
					{
						AX_ROBOT_SelectControlSource(CTL_ROS);
						AX_ROBOT_SetTargetIfActive(CTL_ROS,
							(int16_t)((uart3_rx_buf[4]<<8) | uart3_rx_buf[5]),
							(int16_t)((uart3_rx_buf[6]<<8) | uart3_rx_buf[7]),
							(int16_t)((uart3_rx_buf[8]<<8) | uart3_rx_buf[9]));
					}
					else
					{
						//旧机械臂角度帧仅在机械臂功能启用时处理。
#if OPENCTR_ENABLE_LEGACY_ARM
						if(uart3_rx_buf[2] == 17 && uart3_rx_buf[3] == ID_URX_ANGLE)
						{
							AX_ROBOT_SelectControlSource(CTL_ROS);
							ax_arm_angle[0] = (int16_t)((uart3_rx_buf[4]<<8) | uart3_rx_buf[5]);
							ax_arm_angle[1] = (int16_t)((uart3_rx_buf[6]<<8) | uart3_rx_buf[7]);
							ax_arm_angle[2] = (int16_t)((uart3_rx_buf[8]<<8) | uart3_rx_buf[9]);
							ax_arm_angle[3] = (int16_t)((uart3_rx_buf[10]<<8) | uart3_rx_buf[11]);
							ax_arm_angle[4] = (int16_t)((uart3_rx_buf[12]<<8) | uart3_rx_buf[13]);
							ax_arm_angle[5] = (int16_t)((uart3_rx_buf[14]<<8) | uart3_rx_buf[15]);
						}	
#endif
					}	
				}
			}
		}
		
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	} 
}

/**
  * @简  述  UART 发送数据（X-Protocol协议）
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤27 (32-5)
  *          num：帧号，帧编码
  * @返回值	 无
  */
void AX_UART3_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
  uint8_t tx_checksum = 0;//发送校验和
	
	if(len <= 32)
	{
		/******获取数据******/
		uart3_tx_buf[0] = 0xAA;    //帧头
		uart3_tx_buf[1] = 0x55;    //
		uart3_tx_buf[2] = len+5;  //根据输出长度计算帧长度
		uart3_tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			uart3_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******计算校验和******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart3_tx_buf[i];
		}
		uart3_tx_buf[i] = tx_checksum;
		
		
		/******发送数据******/	
		cnt = 5+len;
		
		//查询传输方式
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART3, uart3_tx_buf[i]);
			while(USART_GetFlagStatus(USART3,USART_FLAG_TC) != SET);
		}
	}
}



