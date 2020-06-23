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
extern "C" {
#include "dubins.h"
}
//Message
#include "underwater_snakerobot_controller/HardwareCommand.h"
#include "underwater_snakerobot_controller/RobotState.h"

using namespace std;
using namespace Eigen;

Vector3d U_kinematic(0.0,0.0,0.0);	//v_t, w, s_dot

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:
	Controller(const ros::NodeHandle& n)
	    :ctrlInput_pub()
	    ,hardware_pub()
		,robotState_sub()
		,ctrl_frequency(get(n, "ctrl_frequency"))
		,ctrl_period(1/ctrl_frequency)
		,turning_radius(get(n, "robot_spec/turning_radius"))
		,max_vel(get(n, "robot_spec/max_vel"))
		,min_vel(get(n, "robot_spec/min_vel"))
		,max_turningRate(get(n, "robot_spec/max_turningRate"))
		,min_turningRate(get(n, "robot_spec/min_turningRate"))
		,eta(get(n, "kinematic_controller/eta"))
		,k_et(get(n, "kinematic_controller/k_et"))
		,k_en(get(n, "kinematic_controller/k_en"))
		,k_theta(get(n, "kinematic_controller/k_theta"))
		,k_s(get(n, "kinematic_controller/k_s"))
		,theta_a(get(n, "kinematic_controller/theta_a"))

	{
		ros::NodeHandle _nh("~");
		ROS_INFO("Initialize ROS message");
		std::cout << "RobotState: " << ctrl_period << std::endl;
		target_pub = _nh.advertise<nav_msgs::Odometry>("/target_path",1);
		// position_pub = _nh.advertise<nav_msgs::Odometry>("/dataNavigator",1);
		ctrlInput_pub = _nh.advertise<std_msgs::Float64MultiArray>("/g500/thrusters_input",1);
		hardware_pub = _nh.advertise<underwater_snakerobot_controller::HardwareCommand>("/hardware_command",1);
		robotState_sub = _nh.subscribe("/g500/pose", 1, &Controller::updateRobotState, this);

		ROS_INFO("Initialize output file");
		outFile_robotState.open("/home/michael/catkin_ws/66666.txt");

		ROS_INFO("Initialize trajectory");
		inFile_trajectory.open("/home/michael/catkin_ws/src/underwater_simulation/underwater_snakerobot_controller/trajectory/wayPoints.txt");
		string line;
		getline(inFile_trajectory, line);
		cout << line << endl;
		s.flag = false;
		// if (!(inFile_trajectory>>path_start[0]))
		// {
		// 	ROS_INFO("**********************************");
		// }
		double wayPoint[4];
		for (int i = 0; i < 4; i++)
		{
			if (!(inFile_trajectory>>wayPoint[i]))
			{
				break;
			}
		}
		path_start[0]=0.0; path_start[1]=0.0; path_start[2]=0.0;
		path_end[0]=wayPoint[0]; path_end[1]=wayPoint[1]; path_end[2]=wayPoint[2];
		cout << wayPoint[0] << wayPoint[1] 
			 << wayPoint[2] << wayPoint[3] << endl;

		ROS_INFO("Initialization finished");
	
		Controller::run(ctrl_period);
	}
    void run(double period)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(period), &Controller::iteration, this);
        std::cout << "Controller started" << std::endl;
        ros::spin();
    }
    ~Controller()
    {
    	outFile_robotState.close();
    	inFile_trajectory.close();
    }
private:
    double integral(double x_dot, double x, double t)
    {
	    /* Computes the integral o x dt */
	    return ((x_dot * t) + x);
    }

	void updateRobotState(
		 const underwater_snakerobot_controller::RobotState::ConstPtr& msg)
	{
		robotState = *msg;
		// std::cout << "RobotState: " << robotState << std::endl;
	}

	void iteration(const ros::TimerEvent& e)
	{
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
				s.length = dubins_path_length(&path);
				s.SetTarget(0.0, s.length, s.VFinal, wayPoint[3], s.length/0.3);
				cout << wayPoint[0] << wayPoint[1] 
					 << wayPoint[2] << wayPoint[3] << endl;
				cout << s.flag << endl;
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
		// ROS_INFO("Kinematic controller");
		s.update();
		double posi_temp[3];
		dubins_path_sample(&path, s.CmdPosition, posi_temp);
      	targetWorld=Map<Vector3d>(posi_temp,3);
      	/***************** Plot ******************/
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
		/***********************************/
      	// cout << s.CmdPosition << " length " << s.length << endl;
      	cout << targetWorld << endl;
      	/* Read robot state */
      	Vector3d currentWorld;
		currentWorld[0] = robotState.positionW.x;
  		currentWorld[1] = robotState.positionW.y;
  		currentWorld[2] = robotState.rotationE.z;
  		// currentWorld[0] = 0.0;
  		// currentWorld[1] = 0.0;
  		// currentWorld[2] = 0.0;
		/* Calculate the cost function for searching best point
		   on path for approaching */
      	double temp_argmin;
      	double s_argmin = 10000;
      	double argmin_index;
		for (double i=0.01;i<s.length;i=i+0.02)
		{
			dubins_path_sample(&path, i, posi_temp);
			temp_argmin=(1-eta)*(pow(posi_temp[0]-currentWorld(0),2)
			                    +pow(posi_temp[1]-currentWorld(1),2)
			                    +pow(posi_temp[2]-currentWorld(2),2))
			             +eta*pow(i-s.CmdPosition,2);
			if (temp_argmin < s_argmin) {
				s_argmin = temp_argmin;
				pathWorld = Map<Vector3d>(posi_temp,3);
				argmin_index = i;
			}
			// cout << " currentWorld: " << posi_temp[2] << endl;
			// cout << " temp_argmin: " << temp_argmin << endl;
			// cout << " argmin_index: " << argmin_index << endl;
		}
		cout << " pathWorld: " << pathWorld(0) << "; " << pathWorld(1) 
		     << "; "<< pathWorld(2) << endl;
		cout << " currentWorld: " << currentWorld(0) << "; " << currentWorld(1) 
		     << "; "<< currentWorld(2) << endl;
		cout << " argmin_index: " << argmin_index << endl;
		cout << " s.CmdPosition: " << s.CmdPosition << endl;
		cout << "*************************************" << endl;
		/* Calculate the curvature C_c */
		dubins_path_sample(&path,
		                 s.Trajectory_Cal_Posi(s.time_cal(argmin_index)+0.01),
		                 posi_temp);
		double C_c = (posi_temp[2]-pathWorld(2))/ctrl_period;
		/* Calculate kinematic control input */
  		Matrix3d rotWorldToRobot;
  		rotWorldToRobot << cos(currentWorld(2)), sin(currentWorld(2)), 0,
  						  -sin(currentWorld(2)), cos(currentWorld(2)), 0,
  						   0,                    0,                    1;
  		Vector3d currentRobot = rotWorldToRobot * (pathWorld - currentWorld);
		double e_s = s.CmdPosition - argmin_index;
		double s_dot = s.CmdVelocity + (1-eta)*k_s*tanh(e_s);
		double alpha_en = theta_a * tanh(k_en*currentRobot(1));
		dot_currentRobot(1) = U_kinematic(1)*currentRobot(0) + s_dot*sin(currentRobot(2));
		double alpha_en_dot = dot_currentRobot(1)*4
		                      *k_en*theta_a*exp(2*k_en*currentRobot(1))
					          /pow(1+exp(2*k_en*currentRobot(1)),2);
		double bar_e_theta = currentRobot(2) + alpha_en;

		if (bar_e_theta!=0)
		{
			U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
										 //- dot_alpha_en
										 + s_dot*currentRobot(1)*sin(currentRobot(2))/bar_e_theta
										 + alpha_en_dot*dot_currentRobot(1);
                     //+ 20*s_dot*sin(currentRobot(2))*currentRobot(1)/currentRobot(2)
										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
										   //(1+pow(k_en*currentRobot(1),2));
		}
		else
		{
			U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
										 - alpha_en_dot;
										 //- s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
                     //+ s_dot*sin(currentRobot(2))
										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
										   //(1+pow(k_en*currentRobot(1),2));
		}
		U_kinematic(0) = (1-eta)*k_et*currentRobot(0) + s.Trajectory_Cal_Vel(s.time_cal(argmin_index))*cos(currentRobot(2));//+ 0.3*U_kinematic(1);
		U_kinematic(0) = std::max(std::min(U_kinematic(0), max_vel),
								     min_vel);
		U_kinematic(1) = std::max(std::min(U_kinematic(1), max_turningRate),
									 min_turningRate);
		/**************************************
		        Dynamic controller
		**************************************/
		// Vector3d velB;
		// velB(0) = robotState.velocityB.x;
  // 		velB(1) = robotState.velocityB.y;
  // 		velB(2) = robotState.angular_vel.z;
		// Vector3d targetVelB;
		// targetVelB(0) = U_kinematic(0)*cos(0);
  // 		targetVelB(1) = U_kinematic(0)*sin(0);
  // 		targetVelB(2) = U_kinematic(1);

		// Vector3d sigma = targetVelB - velB;

		// nu(0) = k_1(0)*sigma(0)+k_2(0)*sigma(0)+z(0);
		// z(0) = integral(k_3(0)*tanh(sigma(0)), z(0), ctrl_period);
		
		cout << "U0: " << U_kinematic(0) << " U1: " << U_kinematic(1) << endl;
		screwJointInput.data.resize(5);
		screwJointInput.data[0] = U_kinematic(0)*6;
		screwJointInput.data[1] = -U_kinematic(1)*1.047;
		screwJointInput.data[2] = U_kinematic(0)*6;
		ctrlInput_pub.publish(screwJointInput);
	}
	// bool readWaypointPosition(double start[3], double end[3])
	// {
	// 	start = end;

	// 	for (int i = 0; i < 3; i++)
	// 	{
	// 		if (!(inFile_trajectory>>end[i]))
	// 		{
	// 			return false;
	// 		}
	// 	}
		
	// 	return true;
	// }
	// bool readWaypointVelocity(double* start, double* end)
	// {
	// 	start = end;

	// 	// if (!(inFile_trajectory>>end))
	// 	// {
	// 	// 	return false;
	// 	// }
	// 	end =2.5;
	// 	return true;
	// }
private:
	/* Ros message */
	ros::Publisher target_pub;
	// ros::Publisher position_pub;
	ros::Publisher ctrlInput_pub;
	ros::Publisher hardware_pub;
	ros::Subscriber robotState_sub;	
	std_msgs::Float64MultiArray screwJointInput;
	underwater_snakerobot_controller::RobotState robotState;
	/* File of all variables for plot */
	ofstream outFile_robotState;
	ifstream inFile_trajectory;
	/* Robot specifications */
	double turning_radius;
	double max_vel;
	double min_vel;
	double max_turningRate;
	double min_turningRate;
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
	double path_start[3];
	double path_end[3];
	tf::Matrix3x3 transform;
	tf::Quaternion q_tf;

	/* Dynamic controller */
	Vector3d k_1;
	Vector3d k_2;
	Vector3d k_3;
	Vector3d k_4;
	Vector3d nu;
	Vector3d z;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setVehicleVelocity");
	ros::NodeHandle nh("~");
	/****************** Test ******************/
	std::cout << "Start: " << std::endl;
	Controller controller(nh);
	std::cout << "End" << std::endl;

// 	double eta = 0.5;
// 	double k_et = 3.6;
// 	double k_en = 3.0;
// 	double k_theta = 10.0;
// 	double k_s = 1.9;
// 	double theta_a = 1.0;
// 	double alpha_en;
// 	double dot_alpha_en;
// 	double bar_e_theta;
// 	double control_period = 100;
// 	double deltaT = 1/control_period;

// 	Vector3d currentRobot(0.0, 0.0, 0.0);  //e_t, e_n, e_z
// 	Vector3d dot_currentRobot(0.0, 0.0, 0.0);
// 	Vector3d currentWorld(0.0, 1.0, 0.0);
// 	Vector3d currentAtti(0.0, 0.0, 0.0);  //roll, pitch, yaw
// 	Vector3d currentRobotDot(0.0,0.0,0.0);
// 	Vector3d pathWorld(0.0,0.0,0.0);
// 	Vector3d pathAtti(0.0, 0.0, 0.0);  //roll, pitch, yaw
// 	Vector3d targetWorld(0.0,0.0,0.0);
// 	Matrix3d rotWorldToRobot;
// 	double C_c;

// 	tf::Matrix3x3 transform;
// 	tf::Quaternion q_tf;

//   	trajectory s;
// 	DubinsPath path;
// 	double s_initial[] = {0, 0, 0};
// 	double s_final[] = {0.0, 2.0, 3.142};
// 	double q_t[3];
//   	double q_p[3];
// 	double turning_radius = 1.0;
// 	double s_dot = 0.0;
//   	double s_argmin = 1000;
//   	double e_s;
//   	double argmin_index;

// 	dubins_init(s_initial, s_final, turning_radius, &path);
// 	double s_length = dubins_path_length(&path);
// 	s.SetTarget(0.0, s_length, 0.0, 0.0, s_length/0.6);

// 	ros::Rate r(control_period);
// 	while (ros::ok()) {
//   		s.update();
//   		dubins_path_sample(&path, s.CmdPosition, q_t);
//       	targetWorld=Map<Vector3d>(q_t,3);

// 		double t_temp = s.time_cal(s.CmdPosition);

// 		for (float i=0;i<=s_length;i=i+deltaT) {
// 			dubins_path_sample(&path, i, q_p);
// 			double temp_argmin;
// 			temp_argmin=(1-eta)*(pow(q_p[0]-currentWorld(0),2)
// 			                    +pow(q_p[1]-currentWorld(1),2)
// 			                    +pow(q_p[2]-currentWorld(2),2))
// 			           +eta*pow(t_temp-s.time_cal(i),2);
// 			if (temp_argmin<s_argmin) {
// 				s_argmin = temp_argmin;
// 				pathWorld = Map<Vector3d>(q_p,3);
// 				argmin_index = i;
// 			}
// 		}
// 		double argmin_q_temp[3];
// 		dubins_path_sample(&path,
// 		                   s.Trajectory_Cal_Posi(s.time_cal(argmin_index)+0.01),
// 		                   argmin_q_temp);
// 		C_c = (argmin_q_temp[2]-pathWorld(2))/deltaT;
// 		// std::cout << "Xposition: " << s.t << " " << s.time_cal(s.CmdPosition) << std::endl;
// 		// std::cout << "C_c: " << pathWorld << std::endl;
// 		// currentWorld(2) = currentAtti(2);
//   		rotWorldToRobot << cos(currentWorld(2)),  sin(currentWorld(2)), 0,
//   											 -sin(currentWorld(2)), cos(currentWorld(2)), 0,
//   											 0,                     0,                    1;
//   		currentRobot = rotWorldToRobot * (pathWorld - currentWorld);
// 		e_s = s.CmdPosition - argmin_index;
// 		s_dot = s.CmdVelocity + (1-eta)*k_s*tanh(e_s);
// 		alpha_en = theta_a * tanh(k_en*currentRobot(1));
// 		dot_currentRobot(1) = U_kinematic(1)*currentRobot(0) + s_dot*sin(currentRobot(2));
// 		dot_alpha_en = dot_currentRobot(1)
// 							*4*k_en*theta_a*exp(2*k_en*currentRobot(1))
// 							/pow(1+exp(2*k_en*currentRobot(1)),2);
// 		bar_e_theta = currentRobot(2) + alpha_en;

// 		double asdf = s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
// 		// std::cout << "s_dot: " << dot_alpha_en << std::endl;

// 		if (bar_e_theta!=0) {
// 			U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
// 										 //- dot_alpha_en
// 										 + s_dot*currentRobot(1)*sin(currentRobot(2))/bar_e_theta
// 										 + dot_alpha_en*dot_currentRobot(1);
//                      //+ 20*s_dot*sin(currentRobot(2))*currentRobot(1)/currentRobot(2)
// 										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
// 										   //(1+pow(k_en*currentRobot(1),2));
// 		}
// 		else {
// 			U_kinematic(1) = C_c*s_dot + k_theta*bar_e_theta
// 										 - dot_alpha_en;
// 										 //- s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
//                      //+ s_dot*sin(currentRobot(2))
// 										 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
// 										   //(1+pow(k_en*currentRobot(1),2));
// 		}
// 		U_kinematic(0) = (1-eta)*k_et*currentRobot(0) + s.Trajectory_Cal_Vel(s.time_cal(argmin_index))*cos(currentRobot(2));//+ 0.3*U_kinematic(1);
// 		if (abs(U_kinematic(1))>=1.0) {
// 			if (U_kinematic(1)>0) {
// 				U_kinematic(1) = 1.0;
// 			}
// 			else {
// 				U_kinematic(1) = -1.0;
// 			}
// 		}
// ************************************************************************************
// 			// dubins_path_sample(&path,
// 			// 						 s.Trajectory_Cal_Posi(s.time_cal(s.CmdPosition)+deltaT),
// 			// 						 argmin_q_temp);
// 			// C_c = (argmin_q_temp[2]-q_p[2])/deltaT;
// 			//
// 			// U_kinematic(0) = k_et*currentRobot(0) + s.CmdVelocity*cos(currentRobot(2));
// 			// if (bar_e_theta!=0) {
// 			// 	U_kinematic(1) = C_c*s.CmdVelocity + k_theta*bar_e_theta
// 			// 								 + s_dot*currentRobot(1)*sin(currentRobot(2))/bar_e_theta
// 			// 								 + dot_alpha_en*dot_currentRobot(1);
// 			// }
// 			// else {
// 			// 	U_kinematic(1) = C_c*s.CmdVelocity + k_theta*bar_e_theta
// 			// 								 - dot_alpha_en;
// 			// }
// 			// if (abs(U_kinematic(1))>=1.0) {
// 			// 	if (U_kinematic(1)>0) {
// 			// 		U_kinematic(1) = 1.0;
// 			// 	}
// 			// 	else {
// 			// 		U_kinematic(1) = -1.0;
// 			// 	}
// 			// }
// 			// if (abs(U_kinematic(0))>=0.8) {
// 			// 	if (U_kinematic(0)>0) {
// 			// 		U_kinematic(0) = 0.8;
// 			// 	}
// 			// 	else {
// 			// 		U_kinematic(0) = -0.8;
// 			// 	}
// 			// }
// /***************************************************************************************/
// 		// transform.setEulerYPR(currentWorld(2), 0.0, 0.0);
// 		// transform.getRotation(q_tf);
//   		// nav_msgs::Odometry odom;
//   		// odom.pose.pose.position.x=currentWorld(0);
//   		// odom.pose.pose.position.y=currentWorld(1);
//   		// odom.pose.pose.position.z=0.05;
//   		// odom.pose.pose.orientation.x=q_tf.getX();
//   		// odom.pose.pose.orientation.y=q_tf.getY();
//   		// odom.pose.pose.orientation.z=q_tf.getZ();
//   		// odom.pose.pose.orientation.w=q_tf.getW();

// 		// transform.setEulerYPR(targetWorld(2), 0.0, 0.0);
// 		// transform.getRotation(q_tf);
// 		// nav_msgs::Odometry odom_path;
// 		// odom_path.pose.pose.position.x=targetWorld(0);
// 		// odom_path.pose.pose.position.y=targetWorld(1);
// 		// odom_path.pose.pose.position.z=0.05;
// 		// odom_path.pose.pose.orientation.x=q_tf.getX();
// 		// odom_path.pose.pose.orientation.y=q_tf.getY();
// 		// odom_path.pose.pose.orientation.z=q_tf.getZ();
// 		// odom_path.pose.pose.orientation.w=q_tf.getW();

//   		// odom.twist.twist.linear.x=2*s.CmdVelocity*cos(targetWorld(2));//(1-eta)*k_et*e_t;
//   		// odom.twist.twist.linear.y=2*s.CmdVelocity*sin(targetWorld(2));//path_s_y;
//   		// odom.twist.twist.linear.z=0;//z;
//   		// odom.twist.twist.angular.x=0;//roll;
//   		// odom.twist.twist.angular.y=0;//pitch;
//   		// odom.twist.twist.angular.z=0;//(theta);//+3.14159265/2);
//   // 		for (int i=0; i<36; i++) {
//   // 			odom.twist.covariance[i]=0;
//   // 			odom.pose.covariance[i]=0;
//   // 		}
//   // 		odom.header.stamp = ros::Time::now();
// 		// odom_path.header.stamp = ros::Time::now();
// 		// target_pub.publish(odom_path);
//   		// position_pub.publish(odom);

// 		// msg_thruster.data.resize(5);
// 		// msg_thruster.data[0] = U_kinematic(0);
// 		// msg_thruster.data[1] = -U_kinematic(1)/2*1.047*3;
// 		// msg_thruster.data[2] = U_kinematic(0);
// 		// controller.ctrlInput_pub.publish(msg_thruster);
// //*********************Update Current pose of robot in Inertial frame
// 		// currentWorld(0) += U_kinematic(0)*cos(currentWorld(2))*deltaT;
// 		// currentWorld(1) += U_kinematic(0)*sin(currentWorld(2))*deltaT+0.1*deltaT;
// 		// currentWorld(2) += U_kinematic(1)*deltaT;
// 		// outputFile << " " << currentWorld(0) << " " << currentWorld(1) << " " << currentWorld(2)
// 		//  					 << " " << targetWorld(0) << " " << targetWorld(1) << " " << targetWorld(2)
// 		// 					 << " " << U_kinematic(0)*cos(currentWorld(2)) << " " << U_kinematic(0)*sin(currentWorld(2))
// 		// 					 << " " << s_dot*cos(currentWorld(2)) << " " << s_dot*sin(currentWorld(2))
// 		// 					 << " " << U_kinematic(1)<< " " << bar_e_theta
// 		// 					 << endl;
// 		// cout << "X: " << currentWorld(0) << " Y: " << currentWorld(1) << " Yaw: " << currentWorld(2) << " Omega: " << -U_kinematic(1) << " v: " << U_kinematic(0) << endl;

// 		// underwater_snakerobot_controller::HardwareCommand msg_hardware;
// 		// cin >> msg_hardware.jointPos1;
// 		// msg_hardware.jointPos1 = 1500;
// 		// hardware_pub.publish(msg_hardware);
// //***************resest*********
// 		s_argmin = 5000;
// //**************************
// 		ros::spinOnce();
// 		r.sleep();
// 	}
	return 0;
}
