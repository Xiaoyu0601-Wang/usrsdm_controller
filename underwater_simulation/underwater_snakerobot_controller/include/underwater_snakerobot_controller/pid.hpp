#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <math.h>

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
    CmdPosition = 0.0;
    CmdVelocity = 0.0;
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
				    CmdPosition = PFinal-0.000001;
            CmdVelocity = VFinal;  /* /S */
		    }
    }
    m_previousTime = ros::Time::now();
  }

  float Trajectory_Cal_Posi(float s)
  {
      float temp;

  	  temp = a0 + a1 * s + a2 * pow(s,2) + a3 * pow(s,3);

      return temp;
  }

  float Trajectory_Cal_Vel(float s)
  {
      float temp;

      temp = a1 + 2 * a2 * s + 3 * a3 * s * s;

      return temp;
  }

  float Trajectory_Cal_Acc(float s)
  {
      float temp;

      temp = 2 * a2 + 6 * a3 * s;

      return temp;
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

  float time_cal(float position)
  {
    double f, g, h, value;
    double i, j, k, l, m, n, p, po;
    double r, s, t, u;
    double x1, x2, x2re, x2im, x3re, x3im, x3;

    float delta0, delta1;
    float a,b,c,d;
    a=a3;b=a2;c=a1;d=a0-position;

    f = ((3*c/a)-((b*b)/(a*a)))/3;
    g = ((2*(b*b*b)/(a*a*a))-(9*b*c/(a*a))+(27*d/a))/27;
    h = ((g*g)/4)+((f*f*f)/27);

 if(f==0 && g==0 && h==0){     // all roots are real and equal
    x1 = pow((d/a),0.33333333333333333333333333333333);
    x2 = pow((d/a),0.33333333333333333333333333333333);
    x3 = pow((d/a),0.33333333333333333333333333333333);
    return x1;
    }
 else if(h<=0){         // all 3 roots are real
    i = pow((((g*g)/4)-h),0.5);
    j = pow(i,0.33333333333333333333333333333333);
    k = acos((g/(2*i))*-1);
    l = j * -1;
    m = cos(k/3);
    n = sqrt(3) * sin(k/3);
    p = (b/(3*a))*-1;
    x1 = (2*j)*m-(b/(3*a));
    x2 = l * (m+n) + p;
    x3 = l * (m-n) + p;
    return x3;
    }
 else if(h>0){        // only 1 root is real
    r = ((g/2)*-1)+pow(h,0.5);
    s = pow(r,0.33333333333333333333333333333333);
    t = ((g/2)*-1)-pow(h,0.5);
    u = pow((t),0.33333333333333333333333333333333);
    x1 = (s+u) - (b/(3*a));
    return x1;
    }

    // delta0 = -b*b*b/27/a/a/a+b*c/6/a/a-d/2/a;
    // delta1 = pow(delta0,2)+pow(c/3/a-b*b/9/a/a,3);
    //
    // return cbrt(delta0+sqrt(delta1))
    //       +cbrt(delta0-sqrt(delta1))
    //       -b/3/a;
    // delta0 = b*b-3*a*c;
    // delta1 = 2*b*b*b-9*a*b*c+27*a*a*d;
    //
    // return cbrt(delta1+sqrt(delta1*delta1-4*delta0*delta0*delta0)/2);
    return 0;
  }

  float CmdPosition;
  float CmdVelocity;
	float t;
	float T;
  double length;
  float PInitial;
  float PFinal;
  float VInitial;
  float VFinal;
	bool flag;

private:
  float a0;
	float a1;
	float a2;
	float a3;
  ros::Time m_previousTime;
};
