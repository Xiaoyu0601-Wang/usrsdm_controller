/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <underwater_snakerobot_controller/HardwareCommand.h>

#include "screwDriveController.hpp"
// #include "serialPort.hpp"

using namespace std;

double get(const ros::NodeHandle& n, const std::string& name)
{
    double value;
    n.getParam(name, value);
    return value;
}

class HardwareCtrl
{
public:
  HardwareCtrl(const ros::NodeHandle& n)
  :hardware_command_sub()
  ,joy_subscriber()
  ,joint_command_client()
  ,hardwareCtrl_frequency(get(n, "hardwareCtrl_frequency"))
  ,max_screwRPM(get(n, "robot_spec/max_screwRPM"))
  ,min_screwRPM(get(n, "robot_spec/min_screwRPM"))
  ,max_jointAngle(get(n, "robot_spec/max_jointAngle"))
  ,min_jointAngle(get(n, "robot_spec/min_jointAngle"))
  ,ScrewDrive()
  ,joint_command()
  ,command_flag(false)
  {
    ros::NodeHandle _nh;

    control_mode = 0;
    ros::NodeHandle params("~");
    params.param<int>("x_axis", axes_.x.axis, 2);
    params.param<int>("y_axis", axes_.y.axis, 3);
    params.param<int>("z_axis", axes_.z.axis, 4);
    params.param<int>("yaw_axis", axes_.yaw.axis, 1);
    params.param<double>("yaw_velocity_max", axes_.yaw.max, 1.0);
    params.param<int>("slow_button", buttons_.slow.button, 1);
    params.param<double>("slow_factor", slow_factor_, 0.2);

    joy_subscriber = _nh.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&HardwareCtrl::joyCallback, this, _1));

    hardware_command_sub = _nh.subscribe("hardware_command", 1, &HardwareCtrl::HardwareCtrlCallback,this);
    joint_command_client = _nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");
    joint_command.request.unit = "raw";

    HardwareCtrl::run(hardwareCtrl_frequency);
  }
  void run(double frequency)
  {
      ros::NodeHandle _node;
      ros::Timer timer = _node.createTimer(ros::Duration(1.0/frequency), &HardwareCtrl::iteration, this);
      printf("Hardware control started\n");
      printf("hardwareCtrl_frequency: %f\n", hardwareCtrl_frequency);
      printf("max_screwRPM: %f\n", max_screwRPM);
      printf("min_screwRPM: %f\n", min_screwRPM);
      printf("max_jointAngle: %f\n", max_jointAngle);
      printf("min_jointAngle: %f\n", min_jointAngle);
      ros::spin();
  }
  ~HardwareCtrl()
  {
    std::cout << "Hardware command stopped" << std::endl;
  }

// private:
  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, int axis)
  {
    // if (axis.axis == 0)
    // {return 0;}
    // sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    // if (axis.axis < 0)
    // {
    //   sign = -1.0;
    //   axis.axis = -axis.axis;
    // }
    // if ((size_t) axis.axis > joy->axes.size())
    // {return 0;}
    // return sign * joy->axes[axis.axis - 1] * axis.max;
    return joy->axes[axis] ;
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0)
    {return 0;}
    if ((size_t) button > joy->buttons.size())
    {return 0;}
    return joy->buttons[button - 1];
  }
  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    /*     Real Robot             */
    // cout << "1111: " << hardware_command.screwVel1 << endl;

    if (getButton(joy, 15)&&(control_mode!=4))
    {
      control_mode = 4;
      printf("Stop Mode\n");
    }
    if (getButton(joy, 14)&&(control_mode!=1))
    {
      control_mode = 1;
      printf("Moving Mode\n");
    }
    if (getButton(joy, 5)&&getButton(joy, 13))
    {
      max_screwRPM += 50;
      min_screwRPM -= 50;
      printf("+50 RPM range\n");
    }
    if (getButton(joy, 7)&&getButton(joy, 13))
    {
      max_screwRPM -= 50;
      min_screwRPM += 50;
      printf("+50 RPM range\n");
    }
  }
  void HardwareCtrlCallback(const underwater_snakerobot_controller::HardwareCommand& msg)
  {
    // cout << msg.jointPos1 << endl;
    hardware_command = msg;
    command_flag = true;
  }

  void iteration(const ros::TimerEvent& e)
  {
    if ((control_mode == 1)&&(command_flag == true))
    {
      /*Check saturation*/
      hardware_command.screwVel1 = std::max(std::min(hardware_command.screwVel1, max_screwRPM),
                                   min_screwRPM);
      hardware_command.screwVel2 = std::max(std::min(hardware_command.screwVel2, max_screwRPM),
                                   min_screwRPM);
      hardware_command.jointPos1 = std::max(std::min(hardware_command.jointPos1, max_jointAngle),
                                   min_jointAngle);
      hardware_command.jointPos2 = std::max(std::min(hardware_command.jointPos2, max_jointAngle),
                                   min_jointAngle);

      joint_command.request.id = 1;
      joint_command.request.goal_position = hardware_command.jointPos1;
      if (joint_command_client.call(joint_command)) {
        if (joint_command.response.result) {
          // ROS_INFO("Succeed to write goal_position");
        }
        else
          ROS_WARN("Failed to write goal_position");
      }
      else {
        ROS_ERROR("Failed to call service /joint_command");
      }

      // joint_command.request.id = 2;
      // joint_command.request.goal_position = hardware_command.jointPos2;
      // if (joint_command_client.call(joint_command)) {
      //   if (joint_command.response.result) {
      //     // ROS_INFO("Succeed to write goal_position");
      //   }
      //   else
      //     ROS_WARN("Failed to write goal_position");
      // }
      // else {
      //   ROS_ERROR("Failed to call service /joint_command");
      // }

      ScrewDrive.SetVelocity(1, -hardware_command.screwVel1);
      // ros::Duration(0.001).sleep();
      ScrewDrive.SetVelocity(2, hardware_command.screwVel2);
      // ros::Duration(0.001).sleep();
      // cout << "joint: " << hardware_command.jointPos1 << "joint: " << hardware_command.jointPos2 << endl;
    }

    if ((control_mode == 4)&&(command_flag == true))
    {
      hardware_command.screwVel1 = std::max(std::min(hardware_command.screwVel1, max_screwRPM),
                                   min_screwRPM);
      hardware_command.screwVel2 = std::max(std::min(hardware_command.screwVel2, max_screwRPM),
                                   min_screwRPM);
      hardware_command.jointPos1 = std::max(std::min(hardware_command.jointPos1, max_jointAngle),
                                   min_jointAngle);
      hardware_command.jointPos2 = std::max(std::min(hardware_command.jointPos2, max_jointAngle),
                                   min_jointAngle);

      joint_command.request.id = 1;
      joint_command.request.goal_position = 2050;
      if (joint_command_client.call(joint_command)) {
        if (joint_command.response.result) {
          // ROS_INFO("Succeed to write goal_position");
        }
        else
          ROS_WARN("Failed to write goal_position");
      }
      else {
        ROS_ERROR("Failed to call service /joint_command");
      }

      // ScrewDrive.SetVelocity(1, -50);
      // ScrewDrive.SetVelocity(2, 50);
      // ros::Duration(0.5).sleep();
      ScrewDrive.SetVelocity(1, 0);
      ScrewDrive.SetVelocity(2, 0);
    }

    command_flag = false;
  }

  private:
    struct Axis
    {
      int axis;
      double max;
    };

    struct Button
    {
      int button;
    };

    struct
    {
      Axis x;
      Axis y;
      Axis z;
      Axis yaw;
    } axes_;

    struct
    {
      Button slow;
    } buttons_;

    double slow_factor_;

    unsigned char control_mode;

    bool command_flag;
    double hardwareCtrl_frequency;
    double max_screwRPM;
    double min_screwRPM;
    double max_jointAngle;
    double min_jointAngle;

    underwater_snakerobot_controller::HardwareCommand hardware_command;
    dynamixel_workbench_msgs::JointCommand joint_command;
    ros::ServiceClient joint_command_client;
    ros::Subscriber hardware_command_sub;
    ros::Subscriber joy_subscriber;

    ScrewDriveController ScrewDrive;
};

///////////////////////////////Main////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hardware_command");

  ros::NodeHandle nh("~");
  HardwareCtrl hardwareCtrl(nh);

  return 0;
}
