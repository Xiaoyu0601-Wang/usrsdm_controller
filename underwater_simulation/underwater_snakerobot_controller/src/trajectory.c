/************************************************************ 
Copyright (C), 1988-2013, Huawei Tech. Co., Ltd.
ProjectName: Omni-directional Mobile Robot 
FileName: trajectory.c
Author: Wang XiaoYu      Version: 1.0V         Date: 17/5/14
Description: 
		Transformation from World Coordinate to Robot Coordinate
Version: 1.0V       
Function  List: 
	1.void limitCheck(wheelSpeedType *output,int maxIn);
	2.float smc_calc(smcObj *smc, float s);
History:       
      <author>  <time>   <version>   <desc> 
	      Wang    17/5/14     1.0		        
***********************************************************/ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <trajectory.h>
#include <mapan.h>
#include <racing.h>
#include <pid.h>
#include <ipd.h>

trajectoryObj AxisXTraj;
trajectoryObj AxisYTraj;
trajectoryObj YawTraj;

float theta = 0;
unsigned char Track_FLAG = 0;
/****************Circle********************/
void CircleTracking(void)
{
	
	if(theta==360)
	{
//		theta=0;
		IpdAxisX.setpoint = 0;
		IpdAxisY.setpoint = 0;
	}
	else
	{
		theta=theta+1;
		IpdAxisX.setpoint = 300*cos(theta/57.2957795f);
		IpdAxisY.setpoint = 300*sin(theta/57.2957795f);
	}
}
/****************Square********************/
void SquareTracking(void)
{
	if((Total.x>505)&&(Total.y<0))
	{Track_FLAG = 1;}
	if((Total.x>505)&&(Total.y>505))
	{Track_FLAG = 2;}
	if((Total.x<0)&&(Total.y>505))
	{Track_FLAG = 3;}

	switch(Track_FLAG)
	{
		case 0:	IpdAxisX.setpoint = 510;IpdAxisY.setpoint = 0;break;
		case 1:	IpdAxisX.setpoint = 510;IpdAxisY.setpoint = 510;break;
		case 2:	IpdAxisX.setpoint = 0;IpdAxisY.setpoint = 510;break;
		case 3:	IpdAxisX.setpoint = 0;IpdAxisY.setpoint = 0;break;
	}
}
/****************SignalTracking********************/
void SignalTracking(void)
{
		theta=theta+1;
		PidAxisX.setpoint = 300*sin(theta/57.2957795f)+400*sin(theta/2/57.2957795f)+100*sin(theta/4/57.2957795f);
}
/****************GoBack********************/
void GoBack(void)
{
	if((Total.x>505)&&(Total.y<0))
	{Track_FLAG = 1;}
	if((Total.theta>178.5f))
	{Track_FLAG = 2;}

	switch(Track_FLAG)
	{
		case 0:	IpdAxisX.setpoint = 510;IpdAxisY.setpoint = 0;break;
		case 1:	IpdAngle.setpoint = 180;break;
		case 2:	IpdAxisX.setpoint = 0;IpdAxisY.setpoint = 0;break;
	}
}
/*************************************************
Function:
Description: 
Calls:
Called By:
Table Accessed:
Table Updated:
Input: Null
Output: Null
Return: Null
Others:
*************************************************/
void PolynomialTrajectory(trajectoryObj *traj)
{
    float temp;

    traj->a0 = traj->PInitial;
    traj->a1 = traj->VInitial;
    temp = 3 * (traj->PFinal - traj->PInitial)
           - (2*traj->VInitial + traj->VFinal) * traj->T;
    traj->a2 = temp / (traj->T * traj->T);
    temp = 2 * (traj->PInitial - traj->PFinal)
           + (traj->VInitial + traj->VFinal) * traj->T;
    traj->a3 = temp / (traj->T * traj->T * traj->T);
}
void SetPosition(trajectoryObj *traj, 
                 float nowPosition, float finalPosition,
                 float nowVelocity, float finalVelocity,
                 float time)
{
    		traj->PInitial = nowPosition;
        traj->VInitial = nowVelocity;
        traj->PFinal = finalPosition;
        traj->VFinal = finalVelocity;
        traj->T = time;
        traj->t = 0;
        PolynomialTrajectory(traj);
        traj->flag = 1;
}

void TimeUpdate(trajectoryObj *traj)
{
    if (traj->flag == 1)
	  {
		    if (traj->t <= traj->T)
        {
            traj->t += 0.001f;
        }
	    	else
		    {
            traj->flag = 0;
				    traj->CmdPosition = traj->PFinal;
            traj->CmdVelocity = traj->VFinal;    /* /S */
		    }
    }
}

void Trajectory_Cal(trajectoryObj *traj)
{
    float temp;

	  temp = traj->a0
         + traj->a1 * traj->t
         + traj->a2 * traj->t * traj->t
         + traj->a3 * traj->t * traj->t * traj->t;
    traj->CmdPosition = temp;

	  temp = traj->a1
         + 2 * traj->a2 * traj->t
         + 3 * traj->a3 * traj->t * traj->t;
    traj->CmdVelocity = temp;    /* /S */
}

//void CenterRotation_Cal(trajectoryObj *traj)
//{
//    float temp;

//	  temp = traj->a0
//         + traj->a1 * traj->t
//         + traj->a2 * traj->t * traj->t
//         + traj->a3 * traj->t * traj->t * traj->t;
//    traj->CmdPosition = temp;

//	  temp = traj->a1
//         + 2 * traj->a2 * traj->t
//         + 3 * traj->a3 * traj->t * traj->t;
//    temp /= 400;         /* degree/2.5mS */
//}

/*************************************End_File*********************************/
///****************Square********************/


//void SquareTracking(void)
//{
//	if (Track_FLAG!=4)
//	{theta = theta+10;}
//	if((Total.x>500)&&(Total.y<10))
//	{Track_FLAG = 1;theta = 0;}
//	if((Total.x<500)&&(Total.y>500))
//	{Track_FLAG = 2;theta = 0;}
//	if((Total.x<10)&&(Total.y>500))
//	{Track_FLAG = 3;theta = 0;}
//	if((Total.y<-10)&&(Total.x<10))
//	{Track_FLAG = 4;theta = 0;}
//	
//	switch(Track_FLAG)
//	{
//		case 0:	IpdAxisX.setpoint = theta;IpdAxisY.setpoint = 0;break;
//		case 1:	IpdAxisX.setpoint = 500;IpdAxisY.setpoint = theta;break;
//		case 2:	IpdAxisX.setpoint = 500-theta;IpdAxisY.setpoint = 500;break;
//		case 3:	IpdAxisX.setpoint = 0;IpdAxisY.setpoint = 500-theta;break;
//		default: break;
//	}
//}
