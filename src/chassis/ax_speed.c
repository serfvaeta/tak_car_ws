/* Includes ------------------------------------------------------------------*/
#include "ax_speed.h"
#include "ax_robot.h"


/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	// 零目标时清除累计 PWM，避免离开扫描模式后恢复旧输出。
	if(spd_target == 0.0f)
	{
		motor_pwm_out = 0;
		bias = 0.0f;
		bias_last = 0.0f;
		return 0;
	}

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}	

/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	// 零目标时清除累计 PWM，避免离开扫描模式后恢复旧输出。
	if(spd_target == 0.0f)
	{
		motor_pwm_out = 0;
		bias = 0.0f;
		bias_last = 0.0f;
		return 0;
	}

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}

/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlC(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	// 零目标时清除累计 PWM，避免离开扫描模式后恢复旧输出。
	if(spd_target == 0.0f)
	{
		motor_pwm_out = 0;
		bias = 0.0f;
		bias_last = 0.0f;
		return 0;
	}

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}

/**
  * @简  述  电机PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_SPEED_PidCtlD(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	// 零目标时清除累计 PWM，避免离开扫描模式后恢复旧输出。
	if(spd_target == 0.0f)
	{
		motor_pwm_out = 0;
		bias = 0.0f;
		bias_last = 0.0f;
		return 0;
	}

	//获得偏差值
	bias = spd_target - spd_current;
	
	//PID计算电机输出PWM值
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//返回PWM控制值
	return motor_pwm_out;
}




