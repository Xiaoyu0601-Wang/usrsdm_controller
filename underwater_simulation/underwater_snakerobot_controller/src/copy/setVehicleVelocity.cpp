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
//Mylib
#include "pid.hpp"
#include "serialPort.hpp"
extern "C" {
#include "dubins.h"
}

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
	ofstream outputFile;
	outputFile.open("1234.txt");

	SerialPort usbCom;
	usbCom.Open("/dev/ttyUSB0");

	double eta = 0.5;
	double k_et = 3.6;
	double k_en = 3.0;
	double k_theta = 10.0;
	double k_s = 1.9;
	double theta_a = 1.0;
	double alpha_en;
	double dot_alpha_en;
	double bar_e_theta;
	double control_period = 100;
	double deltaT = 1/control_period;
  std_msgs::Float64MultiArray msg_thruster;
  double screws[3]= {2,1,2};

	Vector3d currentRobot(0.0, 0.0, 0.0);  //e_t, e_n, e_z
	Vector3d dot_currentRobot(0.0, 0.0, 0.0);
	Vector3d currentWorld(0.0, 1, 0.0);
	Vector3d currentAtti(0.0, 0.0, 0.0);  //roll, pitch, yaw
	Vector3d currentRobotDot(0.0,0.0,0.0);
	Vector3d pathWorld(0.0,0.0,0.0);
	Vector3d pathAtti(0.0, 0.0, 0.0);  //roll, pitch, yaw
	Vector3d targetWorld(0.0,0.0,0.0);
	Vector3d U_kinematic(0.0,0.0,0.0);	//v_t, w, s_dot
	Matrix3d rotWorldToRobot;
  double C_c;

  tf::Matrix3x3 transform;
  tf::Quaternion q_tf;

	ros::init(argc, argv, "setVehicleVelocity");
	ros::NodeHandle nh;
	ros::Publisher position_pub;
  ros::Publisher target_pub;
	ros::Publisher screw_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>("/dataNavigator",1);
	target_pub=nh.advertise<nav_msgs::Odometry>("/target_path",1);
	screw_pub=nh.advertise<std_msgs::Float64MultiArray>("/g500/thrusters_input",1);

  trajectory s;
	DubinsPath path;
	double s_initial[] = {0, 0, 0};
	double s_final[] = {2, 2, 3.142};
	double q_t[3];
  double q_p[3];
	double turning_radius = 1.0;
	double s_dot = 0.0;
  double s_argmin = 1000;
  double e_s;
  double argmin_index;

	dubins_init(s_initial, s_final, turning_radius, &path);
	double s_length = dubins_path_length(&path);
	s.SetTarget(0.0, s_length, 0.0, 0.0, s_length/0.4);

	ros::Rate r(control_period);
	while (ros::ok()) {
    // while(s.flag)
		{
  		s.update();
  		dubins_path_sample(&path, s.CmdPosition, q_t);
      targetWorld=Map<Vector3d>(q_t,3);

			double t_temp = s.time_cal(s.CmdPosition);

      for (float i=0;i<=s_length;i=i+deltaT) {
        dubins_path_sample(&path, i, q_p);
        double temp_argmin;
        temp_argmin=(1-eta)*(pow(q_p[0]-currentWorld(0),2)
                            +pow(q_p[1]-currentWorld(1),2)
                            +pow(q_p[2]-currentWorld(2),2))
                   +eta*pow(t_temp-s.time_cal(i),2);
        if (temp_argmin<s_argmin) {
          s_argmin = temp_argmin;
          pathWorld = Map<Vector3d>(q_p,3);
          argmin_index = i;
        }
      }
			// std::cout << "C_c: " << s_argmin << std::endl;
      double argmin_q_temp[3];
      dubins_path_sample(&path,
                         s.Trajectory_Cal_Posi(s.time_cal(argmin_index)+0.01),
                         argmin_q_temp);
      C_c = (argmin_q_temp[2]-pathWorld(2))/deltaT;
      // std::cout << "Xposition: " << s.t << " " << s.time_cal(s.CmdPosition) << std::endl;
      // std::cout << "C_c: " << pathWorld << std::endl;
      // currentWorld(2) = currentAtti(2);
  		rotWorldToRobot << cos(currentWorld(2)),  sin(currentWorld(2)), 0,
  											 -sin(currentWorld(2)), cos(currentWorld(2)), 0,
  											 0,                     0,                    1;
  		currentRobot = rotWorldToRobot * (pathWorld - currentWorld);
      e_s = s.CmdPosition - argmin_index;
      s_dot = s.CmdVelocity + (1-eta)*k_s*tanh(e_s);
			alpha_en = theta_a * tanh(k_en*currentRobot(1));
			dot_currentRobot(1) = U_kinematic(1)*currentRobot(0) + s_dot*sin(currentRobot(2));
			dot_alpha_en = dot_currentRobot(1)
									 * 4*k_en*theta_a*exp(2*k_en*currentRobot(1))
									   /pow(1+exp(2*k_en*currentRobot(1)),2);
		  bar_e_theta = currentRobot(2) + alpha_en;

			double asdf = s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
			// std::cout << "s_dot: " << dot_alpha_en << std::endl;

			if (bar_e_theta!=0) {
				U_kinematic(1) = C_c*s_dot + (1-eta)*k_theta*bar_e_theta
											 - dot_alpha_en
											 //- s_dot*currentRobot(1)*sin(currentRobot(2))/bar_e_theta
											 -dot_alpha_en*dot_currentRobot(1);
	                     //+ 20*s_dot*sin(currentRobot(2))*currentRobot(1)/currentRobot(2)
											 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
											   //(1+pow(k_en*currentRobot(1),2));
			}
			else {
				U_kinematic(1) = C_c*s_dot + (1-eta)*k_theta*bar_e_theta
											 - dot_alpha_en
											 ;//- s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
	                     //+ s_dot*sin(currentRobot(2))
											 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
											   //(1+pow(k_en*currentRobot(1),2));
			}
			// bar_e_theta = 1+dot_alpha_en*currentRobot(0)*s_dot;
			// if (bar_e_theta!=0) {
			// 	U_kinematic(1) = (C_c*s_dot + (1-eta)*k_theta*bar_e_theta
			// 								 - dot_alpha_en(s_dot*s_dot*sin(currentRobot(2))+currentRobot(1)*s_dot))/bar_e_theta;
	    //                  //+ 20*s_dot*sin(currentRobot(2))*currentRobot(1)/currentRobot(2)
			// 								 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
			// 								   //(1+pow(k_en*currentRobot(1),2));
			// }
			// else {
			// 	U_kinematic(1) = C_c*s_dot + (1-eta)*k_theta*bar_e_theta
			// 								 - dot_alpha_en
			// 								 ;//- s_dot*currentRobot(1)*(sin(currentRobot(2))+sin(alpha_en))/bar_e_theta;
	    //                  //+ s_dot*sin(currentRobot(2))
			// 								 //+ k_en*(U_kinematic(1)*currentRobot(0)-s_dot*sin(currentRobot(2)))
			// 								   //(1+pow(k_en*currentRobot(1),2));
			// }

  		U_kinematic(0) = (1-eta)*k_et*currentRobot(0) + s_dot*cos(currentRobot(2));//+ 0.3*U_kinematic(1);
			if (abs(U_kinematic(1))>=1.5) {
				if (U_kinematic(1)>0) {
					U_kinematic(1) = 1.5;
				}
				else {
					U_kinematic(1) = -1.5;
				}
			}

  		// std::cout << "U1: " << U_kinematic(0) << "  U2: " << U_kinematic(1) << std::endl;

      transform.setEulerYPR(currentWorld(2), 0.0, 0.0);
      transform.getRotation(q_tf);
  		nav_msgs::Odometry odom;
  		odom.pose.pose.position.x=currentWorld(0);
  		odom.pose.pose.position.y=currentWorld(1);
  		odom.pose.pose.position.z=0.5;
  		odom.pose.pose.orientation.x=q_tf.getX();
  		odom.pose.pose.orientation.y=q_tf.getY();
  		odom.pose.pose.orientation.z=q_tf.getZ();
  		odom.pose.pose.orientation.w=q_tf.getW();

			transform.setEulerYPR(targetWorld(2), 0.0, 0.0);
			transform.getRotation(q_tf);
			nav_msgs::Odometry odom_path;
			odom_path.pose.pose.position.x=targetWorld(0);
			odom_path.pose.pose.position.y=targetWorld(1);
			odom_path.pose.pose.position.z=0.5;
			odom_path.pose.pose.orientation.x=q_tf.getX();
			odom_path.pose.pose.orientation.y=q_tf.getY();
			odom_path.pose.pose.orientation.z=q_tf.getZ();
			odom_path.pose.pose.orientation.w=q_tf.getW();

  		// odom.twist.twist.linear.x=2*s.CmdVelocity*cos(targetWorld(2));//(1-eta)*k_et*e_t;
  		// odom.twist.twist.linear.y=2*s.CmdVelocity*sin(targetWorld(2));//path_s_y;
  		// odom.twist.twist.linear.z=0;//z;
  		// odom.twist.twist.angular.x=0;//roll;
  		// odom.twist.twist.angular.y=0;//pitch;
  		// odom.twist.twist.angular.z=0;//(theta);//+3.14159265/2);
  		for (int i=0; i<36; i++) {
  			odom.twist.covariance[i]=0;
  			odom.pose.covariance[i]=0;
  		}
  		odom.header.stamp = ros::Time::now();
			odom_path.header.stamp = ros::Time::now();
			target_pub.publish(odom_path);
  		position_pub.publish(odom);

      msg_thruster.data.resize(5);
      msg_thruster.data[0] = U_kinematic(0);
      msg_thruster.data[1] = -U_kinematic(1)/2*1.047;
      msg_thruster.data[2] = U_kinematic(0);
      screw_pub.publish(msg_thruster);

//*********************Update Current pose of robot in Inertial frame
      currentWorld(0) += U_kinematic(0)*cos(currentWorld(2))*deltaT;
      currentWorld(1) += U_kinematic(0)*sin(currentWorld(2))*deltaT;
      currentWorld(2) += U_kinematic(1)*deltaT;
			outputFile << " " << currentWorld(0) << " " << currentWorld(1) << " " << currentWorld(2)
			 					 << " " << targetWorld(0) << " " << targetWorld(1) << " " << targetWorld(2)
								 << " " << U_kinematic(0)*cos(currentWorld(2)) << " " << U_kinematic(0)*sin(currentWorld(2))
								 << " " << s_dot*cos(currentWorld(2)) << " " << s_dot*sin(currentWorld(2))
								 << " " << U_kinematic(1)<< " " << bar_e_theta
								 << endl;
			// cout << "X: " << currentWorld(0) << " Y: " << currentWorld(1) << " Yaw: " << currentWorld(2) << endl;
//*************************
//***************resest*********
      s_argmin = 5000;
//**************************
    }
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
