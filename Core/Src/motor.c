#include "motor.h"

Robot_t robot;
int abs(int x)
{
    if (x < 0)
        return -x;
    else
        return x;
}
static void Set_Pwm(int m_a,int m_b,int m_c,int m_d)
{
	if(m_a>0 && m_b>0 && m_c>0&& m_d>0)
	
	
  PWMA = abs(m_a);
  PWMB = abs(m_b);
  PWMC = abs(m_c);
  PWMD = abs(m_d);
}
static void pid_motor ()
{

}



void motor_contorl()
{
	//调用pi函数 进行pwm值输出
	
	

	
	
Set_Pwm(robot.MOTOR_A.Output.speed , robot.MOTOR_B.Output .speed,robot.MOTOR_A.Output.speed , robot.MOTOR_B.Output.speed );
}