#include "ax_uart2.h"
#include <stdio.h>

#include "ax_robot.h"
#include "ax_aoa.h"
#include "openctr_features.h"

#if OPENCTR_ENABLE_TTL_CONTROL
static uint8_t uart2_rx_con=0;       //接收计数器
static uint8_t uart2_rx_checksum;    //帧头部分校验和
static uint8_t uart2_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte
#endif
static uint8_t uart2_tx_buf[40];     //发送缓冲

/**
  * @简  述  UART   串口初始化
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
void AX_UART2_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
#if OPENCTR_ENABLE_TTL_CONTROL || OPENCTR_ENABLE_AOA_RECEIVER
	NVIC_InitTypeDef  NVIC_InitStructure;
#endif


	/* 串口USART配置 */
#if OPENCTR_ENABLE_AOA_RECEIVER
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
#else
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
#endif
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//USART对应引脚复用映射
#if OPENCTR_ENABLE_AOA_RECEIVER
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
#else
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
#endif

	//USART 端口配置
#if OPENCTR_ENABLE_AOA_RECEIVER
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
#if OPENCTR_ENABLE_AOA_RECEIVER
	GPIO_Init(GPIOD,&GPIO_InitStructure);
#else
	GPIO_Init(GPIOA,&GPIO_InitStructure);
#endif

#if OPENCTR_ENABLE_AOA_RECEIVER
	// PD5 已接基站 RX；保持空闲高电平，不发送旧底盘遥测。
	GPIO_SetBits(GPIOD, GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

	//USART参数配置
	USART_InitStructure.USART_BaudRate = baud;    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
#if OPENCTR_ENABLE_AOA_RECEIVER
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
#elif OPENCTR_ENABLE_TTL_CONTROL
	USART_InitStructure.USART_Mode |= USART_Mode_Rx;
#endif
	USART_Init(USART2, &USART_InitStructure);

	//USART使能
	USART_Cmd(USART2, ENABLE); 
	
	//开启串口接收中断
#if OPENCTR_ENABLE_TTL_CONTROL || OPENCTR_ENABLE_AOA_RECEIVER
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
	
    //USART2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
#endif
	
}

/**
  * @简  述  UART 串口中断服务函数
  * @参  数  无 
  * @返回值  无
  */
#if OPENCTR_ENABLE_AOA_RECEIVER
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		AX_AOA_OnRxByte((uint8_t)USART_ReceiveData(USART2));
	}
}
#elif OPENCTR_ENABLE_TTL_CONTROL
void USART2_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART2);	
		
		if(uart2_rx_con < 3)    //==接收帧头 + 长度
		{
			if(uart2_rx_con == 0)  //接收帧头1 0xAA
			{
				if(Res == 0xAA)
				{
					uart2_rx_buf[0] = Res;
					uart2_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart2_rx_con == 1) //接收帧头2 0x55
			{
				if(Res == 0x55)
				{
					uart2_rx_buf[1] = Res;
					uart2_rx_con = 2;
				}
				else
				{
					uart2_rx_con = 0;						
				}				
			}
			else  //接收数据长度
			{
				uart2_rx_buf[2] = Res;
				uart2_rx_con = 3;
				uart2_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
			}
		}
		else    //==接收数据
		{
			if(uart2_rx_con < (uart2_rx_buf[2]-1) )
			{
				uart2_rx_buf[uart2_rx_con] = Res;
				uart2_rx_con++;
				uart2_rx_checksum = uart2_rx_checksum + Res;					
			}
			else    //判断最后1位
			{
				//接收完成，恢复初始状态
				uart2_rx_con = 0;	
				
				//数据校验
				if( Res == uart2_rx_checksum )  //校验正确
				{	
					
					//设置为ROS控制模式
					AX_ROBOT_SelectControlSource(CTL_ROS);
					
					//速度控制帧
					if(uart2_rx_buf[3] == ID_URX_VEL)
					{
						AX_ROBOT_SetTargetIfActive(CTL_ROS,
							(int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]),
							(int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]),
							(int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]));
					}
					else
					{
						//舵机角度控制帧
						if(uart2_rx_buf[3] == ID_URX_ANGLE)
						{
							ax_arm_angle[0] = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
							ax_arm_angle[1] = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
							ax_arm_angle[2] = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
							ax_arm_angle[3] = (int16_t)((uart2_rx_buf[10]<<8) | uart2_rx_buf[11]);
							ax_arm_angle[4] = (int16_t)((uart2_rx_buf[12]<<8) | uart2_rx_buf[13]);
							ax_arm_angle[5] = (int16_t)((uart2_rx_buf[14]<<8) | uart2_rx_buf[15]);
						}
					}									
				}
			}
		}
		
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	} 
}


#endif
/**
  * @简  述  UART 发送数据（X-Protocol协议）
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤27 (32-5)
  *          num：帧号，帧编码
  * @返回值	 无
  */
void AX_UART2_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
	
	if(len <= 32)
	{
		/******获取数据******/
		uart2_tx_buf[0] = 0xAA;    //帧头
		uart2_tx_buf[1] = 0x55;    //
		uart2_tx_buf[2] = len+5;  //根据输出长度计算帧长度
		uart2_tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			uart2_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******计算校验和******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart2_tx_buf[i];
		}
		uart2_tx_buf[i] = tx_checksum;
		
		
		/******发送数据******/	
		cnt = 5+len;
		
		//查询传输方式
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART2, uart2_tx_buf[i]);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
		}
	}
}



