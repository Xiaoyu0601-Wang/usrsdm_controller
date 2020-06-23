/*
 * Copath_s_yright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fstream>

//ROS
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
//Matrix
#include <Eigen/Dense>
#include <Eigen/Geometry>
//Dynamixel
// #include <dynamixel_workbench_msgs/JointCommand.h>
//Mylib
// #include "screwDriveController.hpp"
// #include "serialPort.hpp"
#include "pid.hpp"
#include "LowPassFilter.hpp"
extern "C" {
#include "dubins.h"
#include "firstOrderFilter.h"
}
#include "reeds_shepp.h"
// #include "reeds_shepp.hpp"
//Message
#include "underwater_snakerobot_controller/HardwareCommand.h"
#include "underwater_snakerobot_controller/RobotState.h"

// #define M_PI           3.14159265358979323846  /* pi */

using namespace std;
using namespace Eigen;
// using namespace ReedsSheppStateSpace;
// ReedsSheppStateSpace reeds_shepp;

Vector3d U_kinematic(0.0,0.0,0.0);	//v_t, w, s_dot

// LowPassFilter lpf_screwVel1(14.0, 0.01);
// LowPassFilter lpf_screwVel2(14.0, 0.01);
// LowPassFilter lpf_jointPos1(14.0, 0.01);

double get(const ros::NodeHandle& n, const std::string& name)
{
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:
	Controller(const std::string& trajectory, const std::string& outputDataFile, const ros::NodeHandle& n)
    :ctrlInput_pub()
    ,hardware_pub()
		,robotState_sub()
    ,moCap_sub()
    ,screwJointInput()
		,ctrl_frequency(get(n, "ctrl_frequency"))
		,ctrl_period(1.0/ctrl_frequency)
    ,actuator_number(get(n, "robot_spec/actuator_number"))
		,link_length(get(n, "robot_spec/link_length"))
		,turning_radius(get(n, "robot_spec/turning_radius"))
		,max_vel(get(n, "robot_spec/max_vel"))
		,min_vel(get(n, "robot_spec/min_vel"))
    ,max_jointTorque(get(n, "robot_spec/max_jointTorque"))
    ,min_jointTorque(get(n, "robot_spec/min_jointTorque"))
    ,max_thrustForce(get(n, "robot_spec/max_thrustForce"))
    ,min_thrustForce(get(n, "robot_spec/min_thrustForce"))
		,average_vel(get(n, "robot_spec/average_vel"))
		,max_turningRate(get(n, "robot_spec/max_turningRate"))
		,min_turningRate(get(n, "robot_spec/min_turningRate"))
    ,max_screwRPM(get(n, "robot_spec/max_screwRPM"))
    ,min_screwRPM(get(n, "robot_spec/min_screwRPM"))
    ,max_jointAngle(get(n, "robot_spec/max_jointAngle"))
    ,min_jointAngle(get(n, "robot_spec/min_jointAngle"))
    ,lpf_screwVel1(3.0, 0.01)
    ,lpf_screwVel2(3.0, 0.01)
    ,lpf_jointPos1(9.0, 0.01)
    ,m_trajectory(trajectory)
    ,m_outputDataFile(outputDataFile)
		,eta(get(n, "kinematic_controller/eta"))
		,k_et(get(n, "kinematic_controller/k_et"))
		,k_en(get(n, "kinematic_controller/k_en"))
		,k_theta(get(n, "kinematic_controller/k_theta"))
		,k_s(get(n, "kinematic_controller/k_s"))
		,theta_a(get(n, "kinematic_controller/theta_a"))
    ,lastCurrentRobot(0.0,0.0,0.0)
    ,argmin_index(0)
    ,last_argmin_index(0)
    ,ReedsSheppTraj(0.5)
	{
		ros::NodeHandle _nh;
		ROS_INFO("Initialize ROS message");
		std::cout << "ctrl_frequency: " << ctrl_frequency << std::endl;
    std::cout << "ctrl_period: " << ctrl_period << std::endl;
    std::cout << "eta: " << eta << std::endl;
		target_pub = _nh.advertise<nav_msgs::Odometry>("/target_path",1);
		// position_pub = _nh.advertise<nav_msgs::Odometry>("/dataNavigator",1);  //Only give position command
		ctrlInput_pub = _nh.advertise<std_msgs::Float64MultiArray>("/g500/thrusters_input",1);
		hardware_pub = _nh.advertise<underwater_snakerobot_controller::HardwareCommand>("/hardware_command",1);
		robotState_sub = _nh.subscribe("/g500/pose", 1, &Controller::updateRobotState, this);
    moCap_sub = _nh.subscribe("/RigidBody/odom", 1, &Controller::updateMoCap, this);

		ROS_INFO("Initialize output file");
    char m_outputDataFile_char[m_outputDataFile.size() + 1];
    strcpy(m_outputDataFile_char, m_outputDataFile.c_str());
    cout << m_outputDataFile_char << endl;
		outFile_robotState.open(m_outputDataFile_char);//("/home/michael/Documents/Octave/USR/12_2.txt");//DeltaTheta

    ROS_INFO("Initialize robot");
		screwJointInput.data.resize(actuator_number);

		ROS_INFO("Initialize trajectory");
    char m_trajectory_char[m_trajectory.size() + 1];
    strcpy(m_trajectory_char, m_trajectory.c_str());
    cout << m_trajectory_char << endl;
		inFile_trajectory.open(m_trajectory_char);//"/home/michael/catkin_ws/src/underwater_simulation/underwater_snakerobot_controller/trajectory/wayPoints.txt");
		string line;
		getline(inFile_trajectory, line);
		cout << line << endl;
		s.flag = false;
		/* Read first target point */
		double wayPoint[4];
		for (int i = 0; i < 4; i++)
		{
			if (!(inFile_trajectory>>wayPoint[i]))
			{
				break;
			}
		}
		path_start[0]=0.0; path_start[1]=0.0; path_start[2]=0.0;
		path_end[0]=wayPoint[0]; path_end[1]=wayPoint[1]; path_end[2]=wayPoint[2];s.VFinal=wayPoint[3];
		cout << wayPoint[0] << wayPoint[1] << wayPoint[2] << wayPoint[3] << endl;

    moCap.pose.pose.orientation.x = 0;
    moCap.pose.pose.orientation.y = 0;
    moCap.pose.pose.orientation.z = 0;
    moCap.pose.pose.orientation.w = 1;

		ROS_INFO("Initialization finished");

    ros::Duration(2.0).sleep();

/////////////////////////////////////////////////////////////////////////////////
    sway_test_path_start[0]=0.0; sway_test_path_start[1]=0.0; sway_test_path_start[2]=0.0;
    sway_test_path_end[0]=0.0; sway_test_path_end[1]=0.0; sway_test_path_end[2]=0.0;
////////////////////////////////////////////////////////////////////////////////

		Controller::run(ctrl_frequency);
    // testFlag = false;
	}
    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        std::cout << "Controller started" << std::endl;
        ros::spin();
    }
    ~Controller()
    {
    	outFile_robotState.close();
    	inFile_trajectory.close();
      std::cout << "Controller stop" << std::endl;
    }
private:
    double integral(double x_dot, double x, double t)
    {
	    /* Computes the integral o x dt */
	    return ((x_dot * t) + x);
    }

	void updateRobotState(const underwater_snakerobot_controller::RobotState::ConstPtr& msg)
	{
		robotState = *msg;
		// std::cout << "RobotState: " << robotState << std::endl;
	}
  void updateMoCap(const nav_msgs::Odometry::ConstPtr& msg)
  {
    moCap = *msg;
    // std::cout << "RobotState: " << robotState << std::endl;
  }
  double fmodr( double x, double y)
  {
      return x - y*floor(x/y);
  }

  double mod2pi( double theta )
  {
      return fmodr( theta, 2 * M_PI );
  }
  double thetaErrorCal(double desired, double current)
  {
    double temp, thetaE;
    temp = desired - current;
    if (temp > M_PI) {
      thetaE = temp - 2 * M_PI;
    }
    else if (temp < -M_PI) {
      thetaE = 2 * M_PI - temp;
    }
    else {
      thetaE = temp;
    }

    return thetaE;
  }
	void iteration(const ros::TimerEvent& e)
	{
    double qnew[3];
    ReedsSheppTraj.reedsShepp(sway_test_path_start, sway_test_path_end, ReedsSheppTraj);
    sway_test_path_end[1]+=0.01;
    // ReedsSheppTraj.sample(ReedsSheppTraj, 0.2, qnew);
    // while(testFlag==true);
    // testFlag = true;
		/* Read new waypoint */
		if (s.flag == false)
		{
			double wayPoint[4];
			bool temp_flag = true;
			for (int i = 0; i < 4; i++)
			{
				if (!(inFile_trajectory>>wayPoint[i]))
				{
					temp_flag=false;
					// cout << "targetWorld finished" << endl;
					break;
				}
			}
			if (temp_flag == true)
			{
				path_start[0] = path_end[0];path_start[1] = path_end[1];
				path_start[2] = path_end[2];
				path_end[0] = wayPoint[0];path_end[1] = wayPoint[1];
				path_end[2] = wayPoint[2];
				cout << "targetWorld finished" << endl;
				dubins_init(path_start, path_end, turning_radius, &path);
///////////////////////////////////////////////////////////////////////////////
        // ReedsSheppTraj.reedsShepp(path_start, path_end, ReedsSheppTraj);
///////////////////////////////////////////////////////////////////////////////
				s.length = dubins_path_length(&path);
				s.SetTarget(0.0, s.length, s.VFinal, wayPoint[3], s.length/average_vel);
				cout << wayPoint[0] << wayPoint[1]
					   << wayPoint[2] << wayPoint[3] << endl;
				// cout << s.flag << endl;

        argmin_index = 0;
        last_argmin_index = 0;
			}
			// else
			// {
			// 	path_start[0] = robotState.positionW.x;path_start[1] = robotState.positionW.y;
			// 	path_start[2] = robotState.rotationE.z;
			// 	cout << "targetWorld finished" << endl;
			// 	dubins_init(path_start, path_end, turning_radius, &path);
			// 	s.length = dubins_path_length(&path);
			// 	s.SetTarget(0.0, s.length, 0.0, 0.0, s.length/0.3);
			// }
		}
		/**************************************
		        Kinematic controller
		**************************************/
		/* Generate trajectory */
		s.update();
		double posi_temp[3];
		dubins_path_sample(&path, s.CmdPosition, posi_temp);
////////////////////////////////////////////////////////////////////////////////
    // ReedsSheppTraj.sample(ReedsSheppTraj, s.CmdPosition, qnew);
////////////////////////////////////////////////////////////////////////////////
    posi_temp[2] = mod2pi(posi_temp[2]);
    targetWorld=Map<Vector3d>(posi_temp,3);
    // qnew[2] = mod2pi(qnew[2]);
    // targetWorld=Map<Vector3d>(qnew,3);
    cout << "targetWorld:" << targetWorld << endl;
    /* Trajectory plot */
    transform.setEulerYPR(targetWorld(2), 0.0, 0.0);
		transform.getRotation(q_tf);
		nav_msgs::Odometry odom_path;
		odom_path.pose.pose.position.x=targetWorld(0);
		odom_path.pose.pose.position.y=targetWorld(1);
		odom_path.pose.pose.position.z=0.3;
		odom_path.pose.pose.orientation.x=q_tf.getX();
		odom_path.pose.pose.orientation.y=q_tf.getY();
		odom_path.pose.pose.orientation.z=q_tf.getZ();
		odom_path.pose.pose.orientation.w=q_tf.getW();
		odom_path.header.stamp = ros::Time::now();
		target_pub.publish(odom_path);

    Vector3d currentWorld;
    Vector3d velB;
    /* Read robot state (Simulation) */
    currentWorld[0] = robotState.positionW.x;
    currentWorld[1] = robotState.positionW.y;
    currentWorld[2] = mod2pi(robotState.rotationE.z);//fmod(robotState.rotationE.z, 2 * M_PI ); //robotState.rotationE.z;
    velB(0) = robotState.velocityB.x;
    velB(1) = robotState.velocityB.y;
    velB(2) = robotState.angular_vel.z;
    double joint_angle = robotState.joint_angle;
    Vector3d hydro(robotState.hydroDynamics[0],
                   robotState.hydroDynamics[1],
                   robotState.hydroDynamics[2]);
    Vector3d innerForce(robotState.internalForce[0],
                        robotState.internalForce[1],
                        robotState.internalForce[2]);
    double joint_innerForce = robotState.internalForce[3];

    /* Read robot state (Experiment) */
    // tfScalar roll, pitch, yaw;
    // tf::Matrix3x3(
    //   tf::Quaternion(
    //       moCap.pose.pose.orientation.x,
    //       moCap.pose.pose.orientation.y,
    //       moCap.pose.pose.orientation.z,
    //       moCap.pose.pose.orientation.w
    //   )).getRPY(roll, pitch, yaw);
    // currentWorld[0] = moCap.pose.pose.position.x;
    // currentWorld[1] = -moCap.pose.pose.position.y;
    // currentWorld[2] = mod2pi(-(yaw+0.1));//fmod(robotState.rotationE.z, 2 * M_PI ); //robotState.rotationE.z;
    // velB(0) = moCap.twist.twist.linear.x;
    // velB(1) = -moCap.twist.twist.linear.y;
    // velB(2) = -moCap.twist.twist.angular.z;

    cout << "||x: " << currentWorld[0] << "  ||y: " << currentWorld[1] << " ||yaw: " << currentWorld[2] << endl;
    cout << "||VelX: " << velB[0] << "  ||VelY: " << velB[1] << " ||VelYaw: " << velB[2] << endl;
		/***********************************/
    // cout << s.CmdPosition << " length " << s.length << endl;
    Vector3d currentError = (targetWorld - currentWorld);
    eta = 0.5*tanh(abs(currentError(0))-2*0.5*abs(currentError(1)))+0.5;//0.5*(tanh(sqrt(pow(currentRobot(0),2)+pow(0.8*currentRobot(1),2))-1.0)+1.0);
    eta = 1;
		/* Calculate the cost function for searching best point
		   on path for approaching */
    double temp_argmin;
    double s_argmin = 10000;
    last_argmin_index = argmin_index;
    double i_start, i_end;

    double closest_distance=10000;
    double temp_distance, distance_index;
    double dis_search[3];
    for (double i=0;i<s.length;i=i+0.02)
    {
      dubins_path_sample(&path, i, dis_search);
      temp_distance = pow(dis_search[0]-currentWorld(0),2)
                    + pow(dis_search[1]-currentWorld(1),2);
      if (temp_distance < closest_distance) {
        closest_distance = temp_distance;
        distance_index = i;
      }
    }
    if ((s.CmdPosition-0.3)<0) {
      // if (s.CmdPosition-0.3>-0.3) {
        i_start = s.CmdPosition;
      // } else {
      //   i_start = 0.0;
      // }
      // i_start = 0.0;
    }
    else
    {
      i_start = s.CmdPosition-0.3;
    }
    if ((s.CmdPosition+0.0)>s.length) {
      i_end = s.length;
    }
    else
    {
      i_end = s.CmdPosition+0.0;
    }

		for (double i=i_start;i<=i_end;i=i+0.02)
		{
			dubins_path_sample(&path, i, posi_temp);
			temp_argmin=(1-eta)*(pow(posi_temp[0]-currentWorld(0),2)
			                    +pow(posi_temp[1]-currentWorld(1),2)
			                    +pow(thetaErrorCal(posi_temp[2], currentWorld(2)),2))
			             +eta*pow(i-s.CmdPosition,2);
			if (temp_argmin < s_argmin) {
				s_argmin = temp_argmin;
				argmin_index = i;
			}
      double posi_s_argmin[3];
      dubins_path_sample(&path, argmin_index, posi_s_argmin); // s.CmdPosition
      pathWorld = Map<Vector3d>(posi_s_argmin,3);
			// cout << " currentWorld: " << posi_temp[2] << endl;
			// cout << " temp_argmin: " << temp_argmin << endl;
			// cout << " argmin_index: " << argmin_index << endl;
		}
////////////////////////////////////////////////////////////////////////////////////////
		/* Calculate the curvature C_c */
    double path_posi_temp[3],path_posi_temp1[3],time_temp1,time_temp2;
		// dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(last_argmin_index)),
		//                    path_posi_temp);
		// double C_c = thetaErrorCal(pathWorld(2), path_posi_temp[2])/ctrl_period; //thetaErrorCal
    time_temp1 = s.time_cal(argmin_index)-0.01;
    time_temp2 = s.time_cal(argmin_index)+0.01;
    if (time_temp1 < 0)
    {
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp2), path_posi_temp1);
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(argmin_index)), path_posi_temp);
    }
    else if (time_temp2 > s.T)
    {
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(argmin_index)), path_posi_temp1);
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp1), path_posi_temp);
    }
    else
    {
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(argmin_index)), path_posi_temp1);
      dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp1), path_posi_temp);
    }
    // cout << "path_posi_temp1[2]: " << path_posi_temp1[2] << endl;
    // cout << "path_posi_temp[2]: " << path_posi_temp[2] << endl;
    double C_c = thetaErrorCal(path_posi_temp1[2], path_posi_temp[2])/0.005;//0.001
    /* avoid the infinite value */
    if (C_c != C_c) {
      C_c = 0;
    }
    cout << "C_c: " << C_c << endl;

    // double s_dot_v,velx,vely,accx,accy,s_dot_v1,velx1,vely1;
    // s_dot_v = s.Trajectory_Cal_Vel(s.time_cal(argmin_index));
    // velx = s_dot_v *cos(pathWorld(2));
    // vely = s_dot_v *sin(pathWorld(2));
    // s_dot_v1 = s.Trajectory_Cal_Vel(s.Trajectory_Cal_Posi(s.time_cal(argmin_index)-0.01));
    // velx1 = s_dot_v1 *cos(path_posi_temp[2]);
    // vely1 = s_dot_v1 *sin(path_posi_temp[2]);
    // accx = (velx-velx1)/0.01;
    // accy = (vely-vely1)/0.01;
    // C_c = fabs(accy*velx-vely*accx)/sqrt(pow(velx*velx+vely*vely,3));

		/* Calculate kinematic control input */
		Matrix3d rotWorldToRobot;
		rotWorldToRobot << cos(currentWorld(2)), sin(currentWorld(2)), 0,
						          -sin(currentWorld(2)), cos(currentWorld(2)), 0,
						           0,                    0,                    1;
		Vector3d currentRobot = rotWorldToRobot * (pathWorld - currentWorld);
    currentRobot(2) = thetaErrorCal(pathWorld(2), currentWorld(2));
		// double e_s = s.CmdPosition - argmin_index;
		double s_dot = abs(s.CmdVelocity);//(s.Trajectory_Cal_Posi(s.time_cal(last_argmin_index))-s.Trajectory_Cal_Posi(s.time_cal(argmin_index)))/ctrl_period;// + (1-eta)*k_s*tanh(e_s);///s.Trajectory_Cal_Vel(s.time_cal(argmin_index));// + (1-eta)*k_s*tanh(e_s);//s.CmdVelocity + (1-eta)*k_s*tanh(e_s);
    dot_currentRobot(1) = -velB(2)*currentRobot(0) + s_dot*sin(currentRobot(2));
    // dot_currentRobot(1) = U_kinematic(1)*currentRobot(0) + s_dot*sin(currentRobot(2));

    double alpha_en = theta_a * tanh(k_en*currentRobot(1));
    double alpha_en_dot = theta_a * dot_currentRobot(1)*4
                          *k_en*theta_a*exp(2*k_en*currentRobot(1))
                          /pow(1+exp(2*k_en*currentRobot(1)),2);

    // dot_currentRobot = (currentRobot - lastCurrentRobot)/ctrl_period;
    // lastCurrentRobot = currentRobot;
    double bar_e_theta = currentRobot(2) + alpha_en;
//////////////////////////////////////////////////////////////
///////////////////////////eta controller///////////////////////////////////
    // alpha_en = 0;
    // alpha_en_dot = 0;
////////////////////////////////////////////////////////////////////////////////////
///////////////////////////traditional controller///////////////////////////////////
// /* Calculate the curvature C_c */
// // dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(s.CmdPosition)-0.01),
// //                    posi_temp);
// // C_c = -(posi_temp[2]-targetWorld(2))/ctrl_period;
// // double path_posi_temp[3],path_posi_temp1[3],time_temp1,time_temp2;
// time_temp1 = s.time_cal(s.CmdPosition)-0.01;
// time_temp2 = s.time_cal(s.CmdPosition)+0.01;
// if (time_temp1 <= 0)
// {
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp2),
//                      path_posi_temp1);
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(s.CmdPosition)),
//                      path_posi_temp);
// }
// else if (time_temp2 >= s.T)
// {
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(s.CmdPosition)),
//                      path_posi_temp1);
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp1),
//                      path_posi_temp);
// }
// else
// {
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(s.time_cal(s.CmdPosition)),
//                      path_posi_temp1);
//   dubins_path_sample(&path, s.Trajectory_Cal_Posi(time_temp1),
//                      path_posi_temp);
// }
// C_c = thetaErrorCal(path_posi_temp1[2], path_posi_temp[2])/0.01; //thetaErrorCal
// if (C_c != C_c) {
//   C_c = 0;
// }
// /* Calculate kinematic control input */
// rotWorldToRobot << cos(currentWorld(2)), sin(currentWorld(2)), 0,
//                   -sin(currentWorld(2)), cos(currentWorld(2)), 0,
//                    0,                    0,                    1;
// currentRobot = rotWorldToRobot * (targetWorld - currentWorld);
// s_dot = s.CmdVelocity;//(s.Trajectory_Cal_Posi(s.time_cal(last_argmin_index))-s.Trajectory_Cal_Posi(s.time_cal(argmin_index)))/ctrl_period;// + (1-eta)*k_s*tanh(e_s);///s.Trajectory_Cal_Vel(s.time_cal(argmin_index));// + (1-eta)*k_s*tanh(e_s);//s.CmdVelocity + (1-eta)*k_s*tanh(e_s);
// alpha_en = theta_a * tanh(k_en*currentRobot(1));
// dot_currentRobot(1) = velB(2)*currentRobot(0) + s_dot*sin(currentRobot(2));
//////////////////////////////////////Print///////////////////////////////////
    // cout << "targetWorld: " << targetWorld << endl;
    cout << "eta: " << eta << endl;
    cout << " pathWorld: " << pathWorld(0) << "; " << pathWorld(1)
         << "; "<< pathWorld(2) << endl;
    // cout << " currentWorld: " << currentWorld(0) << "; " << currentWorld(1)
    //      << "; "<< currentWorld(2) << endl;
    // cout << " argmin_index: " << argmin_index << endl;
    // cout << " s.CmdPosition: " << s.CmdPosition << endl;
    cout << " C_c: " << C_c << endl;
    cout << " s_dot: " << C_c << endl;
    cout << " bar_e_theta: " << C_c << endl;

////////////////////////////////////////////////////////////////////////////////
    /* plot */
		// cout << "C_c: " << (posi_temp[2]-pathWorld(2))/ctrl_period << endl;
		// cout << "s_dot: " << s_dot << endl;

		if (bar_e_theta!=0)
		{
			// U_kinematic(1) = k_theta*bar_e_theta;
      U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
										 + alpha_en_dot*dot_currentRobot(1)
										 + s_dot*currentRobot(1)*(sin(bar_e_theta-alpha_en)+sin(alpha_en))/bar_e_theta;


										 //- dot_alpha_en
                     //+ 20*s_dot*sin(currentRobot(2))*currentRobot(1)/currentRobot(2)
										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
										   //(1+pow(k_en*currentRobot(1),2));
		}

		else
		{
			U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
										 + alpha_en_dot*dot_currentRobot(1);

       // U_kinematic(1) = C_c*s_dot + k_theta*e_theta;

										 //- s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
                     //+ s_dot*sin(currentRobot(2))
										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
										   //(1+pow(k_en*currentRobot(1),2));
		}
		U_kinematic(0) = k_et*currentRobot(0) + s_dot*cos(currentRobot(2));//s.Trajectory_Cal_Vel(s.time_cal(argmin_index))*cos(currentRobot(2));//+ 0.3*U_kinematic(1);
		U_kinematic(0) = std::max(std::min(U_kinematic(0), max_vel),
								              min_vel);
		U_kinematic(1) = std::max(std::min(U_kinematic(1), max_turningRate),
									            min_turningRate);
    cout << " currentRobot: " << currentRobot << endl;
		/**************************************
		        Dynamic controller in Simulation
		**************************************/
		/* Hydrodynamics, collis and innerForce */
		cout << "innerForce " << innerForce << endl;
		cout << "hydro: " << hydro << endl;
  		/* Velocity error in {B} frame */
		Vector3d targetVelB;
		targetVelB(0) = U_kinematic(0)*cos(currentWorld[2]); //Zero represents Beta in the paper
		targetVelB(1) = U_kinematic(0)*sin(currentWorld[2]);
		targetVelB(2) = -U_kinematic(1);
		Vector3d sigma = targetVelB - velB;
		// cout << "sigma: " << sigma << endl;
		// /* Gain */
		// Vector3d k_1(0.6, 0.6, 0.6);
		// Vector3d k_2(2.0, 2.0, 4.0);
		// Vector3d k_3(0.0, 0.0, 0.0);
		// Vector3d k_4(0.0, 0.0, 0.0);
		// /* Super-twisting algorithm */
		// Vector3d z;
		// Vector3d nu;
		// z(0) = integral(k_3(0)*tanh(sigma(0))+k_4(0)*sigma(0), z(0), ctrl_period);
		// nu(0) = k_1(0)*tanh(sigma(0))+k_2(0)*sigma(0)+z(0);
		// z(1) = integral(k_3(1)*tanh(sigma(1))+k_4(1)*sigma(1), z(1), ctrl_period);
		// nu(1) = k_1(1)*tanh(sigma(1))+k_2(1)*sigma(1)+z(1);
		// z(2) = integral(k_3(2)*tanh(sigma(2))+k_4(2)*sigma(2), z(2), ctrl_period);
		// nu(2) = k_1(2)*tanh(sigma(2))+k_2(2)*sigma(2)+z(2);
		// nu(2) = nu(2) +joint_innerForce;
		// /* Feedback lineariation */
		// Matrix3d M;
		// M << 2.089,   0,    0,
		//          0, 2.6,    0,
		//          0,   0, 0.0149;
		// Matrix3d B;
		// // joint_angle = 0.8;
		// // double angle_link2 = currentWorld[2] + joint_angle;
		// // B << cos(currentWorld[2]), cos(angle_link2), 0,
		// //      sin(currentWorld[2]), sin(angle_link2), 0,
		// //      0                   , 0               , 1;
		// // nu(0)=0;nu(1)=0;nu(2)=0;

		// if (joint_angle == 0)
		// {
		// 	// if (U_kinematic(1)>=0)
		// 	// {
		// 	// 	joint_angle = 0.01;
		// 	// }
		// 	// else if (U_kinematic(1)<0)
		// 	// {
		// 	// 	joint_angle = -0.01;
		// 	// }
		// 	/* Kinematic input */
		// 	screwJointInput.data[0] = U_kinematic(0)*6;
		// 	screwJointInput.data[1] = -U_kinematic(1)*1.047;
		// 	screwJointInput.data[2] = U_kinematic(0)*6;
		// }
		// else
		// {
		// 	B << 1,                cos(joint_angle),  0,
		// 	     0,                sin(joint_angle),  0,
		// 	     0, -link_length/2*sin(joint_angle), -1;
		// 	Vector3d input_dyn;
		// 	input_dyn = B.inverse()*(M*nu-hydro-innerForce);
		// 	cout << "nu: " << nu << endl;
		// 	input_dyn(0) = std::max(std::min(input_dyn(0), 2.0),
		// 							     -2.0);
		// 	input_dyn(1) = std::max(std::min(input_dyn(1), 2.0),
		// 								 -2.0);
		// 	input_dyn(2) = std::max(std::min(input_dyn(2), 3.0),
		// 							     -3.0);
		// 	cout << "input_dyn: " << input_dyn << endl;
		// 	screwJointInput.data[0] = input_dyn(0);
		// 	screwJointInput.data[1] = -input_dyn(2);
		// 	screwJointInput.data[2] = input_dyn(1);
		// }
		// // B << 1,                cos(joint_angle),  0,
		// //      0,                sin(joint_angle),  0,
		// //      0, -link_length/2*sin(joint_angle), -1;
		// // Vector3d input_dyn;
		// // input_dyn = B.inverse()*(M*nu-hydro-innerForce);
		// // cout << "nu: " << nu << endl;
		/**************************************
		        Control input
		**************************************/
		cout << "U0: " << U_kinematic(0) << endl;
		cout  << " U1: " << U_kinematic(1) << endl;

		/* Kinematic input */
		screwJointInput.data[0] = U_kinematic(0)*10.0;//*10.0;
		screwJointInput.data[1] = -U_kinematic(1)*5.0;//*5.0;//*1.047;
		screwJointInput.data[2] = U_kinematic(0)*10.0;//*10.0;

    screwJointInput.data[0] = lpf_screwVel1.update(screwJointInput.data[0]);
    screwJointInput.data[1] = lpf_jointPos1.update(screwJointInput.data[1]);
    screwJointInput.data[2] = lpf_screwVel2.update(screwJointInput.data[2]);
    // screwJointInput.data[0] = firstOrderFilter((float)screwJointInput.data[0]
    //                                           ,&firstOrderFilters[0]);
    // screwJointInput.data[1] = firstOrderFilter((float)screwJointInput.data[1]
    //                                           ,&firstOrderFilters[1]);
    // screwJointInput.data[2] = firstOrderFilter((float)screwJointInput.data[2]
    //                                           ,&firstOrderFilters[2]);
    screwJointInput.data[0] = std::max(std::min(screwJointInput.data[0], max_thrustForce),
									            min_thrustForce);
    screwJointInput.data[1] = std::max(std::min(screwJointInput.data[1], max_jointTorque),
									            min_jointTorque);
    screwJointInput.data[2] = std::max(std::min(screwJointInput.data[2], max_thrustForce),
									            min_thrustForce);
		/* Dynamic input */
		// input_dyn(0) = std::max(std::min(input_dyn(0), 2.0),
		// 						     -2.0);
		// input_dyn(1) = std::max(std::min(input_dyn(1), 2.0),
		// 							 -2.0);
		// input_dyn(2) = std::max(std::min(input_dyn(2), 3.0),
		// 						     -3.0);
		// cout << "input_dyn: " << input_dyn << endl;
		// screwJointInput.data[0] = input_dyn(0);
		// screwJointInput.data[1] = input_dyn(2);
		// screwJointInput.data[2] = input_dyn(1);
///////////////////////For simulation//////////////////////////////////////////
		cout << "screwJointInput: " << screwJointInput.data[0]
							<< "; " << screwJointInput.data[1]
							<< "; " << screwJointInput.data[2] << endl;
		ctrlInput_pub.publish(screwJointInput);
///////////////////////For real robot//////////////////////////////////////////
    hardware_command.screwVel1	=	screwJointInput.data[0]/3*max_screwRPM;
    hardware_command.screwVel2	=	screwJointInput.data[2]/3*max_screwRPM;
    hardware_command.jointPos1 =	screwJointInput.data[1]/5*650+2050;
    cout << "hardware_command: " << hardware_command.screwVel1
              << "; " << hardware_command.jointPos1
              << "; " << hardware_command.screwVel2 << endl;
    hardware_pub.publish(hardware_command);

		// outFile_robotState << " " << currentWorld(0) << " " << currentWorld(1) << " " << currentWorld(2)
	 	// 				 << " " << targetWorld(0) << " " << targetWorld(1) << " " << targetWorld(2)
		// 				 << " " << U_kinematic(0) << " " << U_kinematic(1) //7,8
		// 				 << " " << s_dot*cos(currentWorld(2)) << " " << s_dot*sin(currentWorld(2)) //9,10
		// 				 << " " << screwJointInput.data[0]<< " " << screwJointInput.data[1] << " "  << eta //11,12,13
		// 				 << endl;

            outFile_robotState << " " << ReedsSheppTraj.distance(ReedsSheppTraj) << endl;
		cout << "*************************************" << endl;
}
private:
	/* Ros message */
	ros::Publisher target_pub;
	// ros::Publisher position_pub;
	ros::Publisher ctrlInput_pub;
	ros::Publisher hardware_pub;
	ros::Subscriber robotState_sub;
  ros::Subscriber moCap_sub;

  std::string m_trajectory;
  std::string m_outputDataFile;

	std_msgs::Float64MultiArray screwJointInput;
  underwater_snakerobot_controller::HardwareCommand hardware_command;
	underwater_snakerobot_controller::RobotState robotState;
  nav_msgs::Odometry moCap;
	/* File of all variables for plot */
	ofstream outFile_robotState;
	ifstream inFile_trajectory;
	/* Robot specifications */
  int actuator_number;
	double link_length;
	double turning_radius;
	double max_vel;
	double min_vel;
	double average_vel;
	double max_turningRate;
	double min_turningRate;
  double max_jointTorque;
  double min_jointTorque;
  double max_thrustForce;
  double min_thrustForce;
  double max_screwRPM;
  double min_screwRPM;
  double max_jointAngle;
  double min_jointAngle;
  LowPassFilter lpf_screwVel1;
  LowPassFilter lpf_screwVel2;
  LowPassFilter lpf_jointPos1;
	/* Controller parameters */
	double ctrl_frequency;
	double ctrl_period;
	double eta;
	double k_et;
	double k_en;
	double k_theta;
	double k_s;
	double theta_a;

	Vector3d dot_currentRobot;
	Vector3d pathWorld;
	Vector3d targetWorld;

  trajectory s;
	DubinsPath path;
  ReedsSheppStateSpace ReedsSheppTraj;
  // ReedsSheppStateSpace::ReedsSheppPath ReedsSheppPath;
	double path_start[3];
	double path_end[3];
	tf::Matrix3x3 transform;
	tf::Quaternion q_tf;
  Vector3d lastCurrentRobot;
  double argmin_index;
  double last_argmin_index;

	/* Dynamic controller */
	Vector3d k_1;
	Vector3d k_2;
	Vector3d k_3;
	Vector3d k_4;
	Vector3d nu;
	Vector3d z;

  double sway_test_path_start[3];
	double sway_test_path_end[3];

  //ReedsSheppPath
  // ReedsSheppStateSpace reeds_shepp;
  // bool testFlag;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathTrackingControl");
	ros::NodeHandle nh("~");

  std::string trajectory, outputDataFile;
  nh.getParam("trajectory",trajectory);//param<std::string>("trajectory", trajectory);
  nh.getParam("outputDataFile",outputDataFile);
  // std::cout << trajectory << std::endl;
	/****************** Test ******************/
	std::cout << "Start: " << std::endl;
	Controller controller(trajectory, outputDataFile, nh);
	std::cout << "End" << std::endl;

	return 0;
}
