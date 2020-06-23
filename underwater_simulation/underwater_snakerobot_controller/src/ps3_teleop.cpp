//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#include "LowPassFilter.hpp"
#include "underwater_snakerobot_controller/HardwareCommand.h"

using namespace std;

unsigned char control_mode = 1;

namespace hector_quadrotor
{

class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;

  // unsigned char control_mode;

  ros::Publisher hardwareCommand_publisher_;
  ros::Publisher screw_pub_;
  underwater_snakerobot_controller::HardwareCommand hardware_command;
  std_msgs::Float64MultiArray msg_thruster;

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
//=============================================================
// Control Mode
// 0: Two screws move with the same velocity
// 1:
//=============================================================
  unsigned char control_mode;

  LowPassFilter lpf_screwVel1;
  LowPassFilter lpf_screwVel2;
  LowPassFilter lpf_jointPos1;
public:
  Teleop()
  :lpf_screwVel1(12.0, 0.01)
  ,lpf_screwVel2(12.0, 0.01)
  ,lpf_jointPos1(12.0, 0.01)
  {
    // axes_.x.axis=2;
    // axes_.y.axis=3;
    // axes_.z.axis=4;
    // axes_.yaw.axis=1;
    // axes_.yaw.max=1.0;
    // buttons_.slow.button=1;
    // slow_factor_=0.2;
    control_mode = 0;
    ros::NodeHandle params("~");

    params.param<int>("x_axis", axes_.x.axis, 2);
    params.param<int>("y_axis", axes_.y.axis, 3);
    params.param<int>("z_axis", axes_.z.axis, 4);
    params.param<int>("yaw_axis", axes_.yaw.axis, 1);

    params.param<double>("yaw_velocity_max", axes_.yaw.max, 1.0);
    params.param<int>("slow_button", buttons_.slow.button, 1);
    params.param<double>("slow_factor", slow_factor_, 0.2);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyCallback, this, _1));
    hardwareCommand_publisher_ = node_handle_.advertise<underwater_snakerobot_controller::HardwareCommand>("hardware_command", 1);
    screw_pub_=node_handle_.advertise<std_msgs::Float64MultiArray>("/g500/thrusters_input",1);

    control_mode = 1;
  }

  ~Teleop()
  {
    stop();
  }

  void joyCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    if (getButton(joy, 1)&&(control_mode!=1))
    {
      control_mode = 1;
      printf("AUV Mode\n");
    }
    if (getButton(joy, 4)&&(control_mode!=2))
    {
      control_mode = 2;
      printf("Seperate Mode\n");
    }

    if (control_mode == 1)
    {
      /*     Real Robot             */
      hardware_command.screwVel1 = getAxis(joy, 1)*150.0;//*50.0;
      hardware_command.screwVel2 = getAxis(joy, 1)*150.0;//*50.0;
      hardware_command.jointPos1 = getAxis(joy, 2)*650.0+2050.0;//*1150.0+2050.0;
      hardware_command.jointPos2 = getAxis(joy, 3)*650.0+2050.0;//*1150.0+2050.0;
      // hardware_command.screwVel1 = lpf_screwVel1.update(hardware_command.screwVel1);
      // hardware_command.jointPos1 = lpf_jointPos1.update(hardware_command.jointPos1);
      // hardware_command.screwVel2 = lpf_screwVel2.update(hardware_command.screwVel2);
      // cout << "1111: " << hardware_command.screwVel1 << endl;
      // cout << "2222: " << hardware_command.screwVel2 << endl;
      // cout << "3333: " << hardware_command.jointPos1 << endl;
      // cout << "4444: " << hardware_command.jointPos2 << endl;

      // if (getButton(joy, buttons_.slow.button))
      // {
      //   velocity_.linear.x *= slow_factor_;
      //   velocity_.linear.y *= slow_factor_;
      //   velocity_.linear.z *= slow_factor_;
      //   velocity_.angular.z *= slow_factor_;
      // }
      hardwareCommand_publisher_.publish(hardware_command);

      /*     Simulation             */
      msg_thruster.data.resize(5);
      msg_thruster.data[0] = getAxis(joy, 1)*2.0;
      msg_thruster.data[1] = (getAxis(joy, 13)-getAxis(joy, 12))*3/2;
      msg_thruster.data[2] = getAxis(joy, 3)*2.0;
      screw_pub_.publish(msg_thruster);
    }

    if (control_mode == 2)
    {
      /*     Real Robot      */
      hardware_command.screwVel1 = getAxis(joy, 1)*150.0;//*50.0;
      hardware_command.screwVel2 = getAxis(joy, 3)*150.0;//*50.0;
      hardware_command.jointPos1 = (getAxis(joy, 13)-getAxis(joy, 12))*650.0+2050.0;//*1150.0+2050.0;
      // hardware_command.jointPos2 = getAxis(joy, 3)*650.0+2050.0;//*1150.0+2050.0;
      // hardware_command.screwVel1 = lpf_screwVel1.update(hardware_command.screwVel1);
      // hardware_command.jointPos1 = lpf_jointPos1.update(hardware_command.jointPos1);
      // hardware_command.screwVel2 = lpf_screwVel2.update(hardware_command.screwVel2);
      // cout << "1111: " << hardware_command.screwVel1 << endl;
      // cout << "2222: " << hardware_command.screwVel2 << endl;
      // cout << "3333: " << hardware_command.jointPos1 << endl;
      // cout << "4444: " << hardware_command.jointPos2 << endl;

      // if (getButton(joy, buttons_.slow.button))
      // {
      //   velocity_.linear.x *= slow_factor_;
      //   velocity_.linear.y *= slow_factor_;
      //   velocity_.linear.z *= slow_factor_;
      //   velocity_.angular.z *= slow_factor_;
      // }
      hardwareCommand_publisher_.publish(hardware_command);

      /*     Simulation             */
      msg_thruster.data.resize(5);
      msg_thruster.data[0] = getAxis(joy, 1)*2.0;
      msg_thruster.data[1] = (getAxis(joy, 13)-getAxis(joy, 12))*3/2;
      msg_thruster.data[2] = getAxis(joy, 3)*2.0;
      screw_pub_.publish(msg_thruster);
    }
  }

  // void sendCommand(void)
  // {
  //   hardwareCommand_publisher_.publish(hardware_command);
  // }

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

  void stop()
  {

  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
