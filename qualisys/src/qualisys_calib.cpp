#include <ros/ros.h>
#include <ros/node_handle.h>
#include <qualisys/QualisysCalib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qualisys_calibrate");

  ros::NodeHandle nh("~");
  qualisys::QualisysCalib calib(nh);

  if (!calib.init()) {
    ROS_INFO("Calibration initialization failed!");
    return -1;
  }

  return 0;
}
