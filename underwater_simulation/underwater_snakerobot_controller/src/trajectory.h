#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

typedef struct _TRAJECTORY
{
	float a0;   //设定目标
	float a1;   //比例常数 p 
	float a2;   //积分常数 I 
	float a3;   //微分常数 D 
	float T;
	float t;
	float PInitial;
	float PFinal;
	float VInitial;
	float VFinal;
	float CmdPosition;
	float CmdVelocity;
	float flag;
}trajectoryObj;

extern trajectoryObj AxisXTraj;
extern trajectoryObj AxisYTraj;
extern trajectoryObj YawTraj;

extern float theta;
extern unsigned char Track_FLAG;


extern void CircleTracking(void);
extern void SquareTracking(void);
extern void GoBack(void);
extern void SignalTracking(void);

extern void TimeUpdate(trajectoryObj *traj);
extern void Trajectory_Cal(trajectoryObj *traj);
extern void SetPosition(trajectoryObj *traj,
                        float nowPosition, float finalPosition,
                        float nowVelocity, float finalVelocity,
                        float time);

#endif
