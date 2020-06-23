#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

class trajectory
{
public:
    void reset()
    {
        // m_integral = 0;
        // m_previousError = 0;
        // m_previousTime = ros::Time::now();
    }

  void PolynomialTrajectory(void)
  {
    float temp;

    a0 = PInitial;
    a1 = VInitial;
    temp = 3 * (PFinal - PInitial)
           - (2*VInitial + VFinal) * T;
    a2 = temp / (T * T);
    temp = 2 * (PInitial - PFinal)
           + (VInitial + VFinal) * T;
    a3 = temp / (T * T * T);
  }

  void SetTarget(float nowPosition, float finalPosition,
                 float nowVelocity, float finalVelocity,
                 float taskTime)
  {
		PInitial = nowPosition;
    VInitial = nowVelocity;
    PFinal = finalPosition;
    VFinal = finalVelocity;
    T = taskTime;
    t = 0;
    PolynomialTrajectory();
    flag = true;
    m_previousTime = ros::Time::now();
  }

  void update()
  {
    float temp;

    ros::Time time = ros::Time::now();
    float dt = time.toSec() - m_previousTime.toSec();

    if (flag == true)
	  {
		    if (t < T)
        {
            t += dt;
            temp = a0 + a1 * t + a2 * pow(t,2) + a3 * pow(t,3);
            CmdPosition = temp;

            temp = a1
                 + 2 * a2 * t
                 + 3 * a3 * t * t;
            CmdVelocity = temp;    /* /S */
        }
	    	else
		    {
            flag = false;
				    CmdPosition = PFinal;
            CmdVelocity = VFinal;    /* /S */
		    }
    }
    m_previousTime = ros::Time::now();
  }



  void Trajectory_Cal(float s)
  {
      float temp;

  	  temp = a0 + a1 * s + a2 * pow(s,2) + a3 * pow(s,3);
      CmdPosition = temp;

  	  temp = a1
           + 2 * a2 * s
           + 3 * a3 * s * s;
      CmdVelocity = temp;    /* /S */
  }

  float CmdPosition;
  float CmdVelocity;

private:
  float a0;
	float a1;
	float a2;
	float a3;
	float T;
	float t;
	float PInitial;
	float PFinal;
	float VInitial;
	float VFinal;
	bool flag;
  ros::Time m_previousTime;
};
