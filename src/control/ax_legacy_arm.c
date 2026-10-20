#include "ax_legacy_arm.h"
#include "ax_robot.h"

//机械臂关节角度
int16_t ax_arm_angle[6] = {0};   

//机械臂预设动作，单位角度放大10倍
const int16_t ax_arm_set_angle[4][6]=	  
{
	{   0,-385,-860,-570,   0,   0},   //动作1 行驶保护动作
	{   0, 300,-640,-570,   0,-600},   //动作2 抓取地面物体动作
	{   0, 300, 670, 700,   0,-600},   //动作3 向后放下物体动作
	{   0,   0,   0,   0,   0,   0}    //动作4 直立向上动作
};

/**
  * @简  述  舵机云台机械臂控制函数
  * @参  数  无
  * @返回值  无
  */
void JOINT_Control(void)
{
	//幅度限制保护
	if(ax_arm_angle[0] > JOINTA_UP_LIMIT)	 ax_arm_angle[0] = JOINTA_UP_LIMIT;
	if(ax_arm_angle[0] < JOINTA_LOW_LIMIT)   ax_arm_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_arm_angle[1] > JOINTB_UP_LIMIT)	 ax_arm_angle[1] = JOINTB_UP_LIMIT;
	if(ax_arm_angle[1] < JOINTB_LOW_LIMIT)   ax_arm_angle[1] = JOINTB_LOW_LIMIT;
	
	if(ax_arm_angle[2] > JOINTC_UP_LIMIT)	 ax_arm_angle[2] = JOINTC_UP_LIMIT;
	if(ax_arm_angle[2] < JOINTC_LOW_LIMIT)   ax_arm_angle[2] = JOINTC_LOW_LIMIT;

	if(ax_arm_angle[3] > JOINTD_UP_LIMIT)	 ax_arm_angle[3] = JOINTD_UP_LIMIT;
	if(ax_arm_angle[3] < JOINTD_LOW_LIMIT)   ax_arm_angle[3] = JOINTD_LOW_LIMIT;

	if(ax_arm_angle[4] > JOINTE_UP_LIMIT)	 ax_arm_angle[4] = JOINTE_UP_LIMIT;
	if(ax_arm_angle[4] < JOINTE_LOW_LIMIT)   ax_arm_angle[4] = JOINTE_LOW_LIMIT;

	if(ax_arm_angle[5] > JOINTF_UP_LIMIT)	 ax_arm_angle[5] = JOINTF_UP_LIMIT;
	if(ax_arm_angle[5] < JOINTF_LOW_LIMIT)   ax_arm_angle[5] = JOINTF_LOW_LIMIT;	
	
	//查看舵机角度
	//printf("A%d  \r\n ", ax_arm_angle[0] );
	
	//设置舵机角度
	AX_SERVO_S1_SetAngle( ax_arm_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle(-ax_arm_angle[1] - JOINTB_ANGLE_OFFSET);
	AX_SERVO_S3_SetAngle( ax_arm_angle[2] + JOINTC_ANGLE_OFFSET);
	AX_SERVO_S4_SetAngle( ax_arm_angle[3] + JOINTD_ANGLE_OFFSET);	
	AX_SERVO_S5_SetAngle( ax_arm_angle[4] + JOINTE_ANGLE_OFFSET);
	AX_SERVO_S6_SetAngle( ax_arm_angle[5] + JOINTF_ANGLE_OFFSET);		
}

