#include <ros/ros.h>
#include <qualisys/QualisysDriver.h>

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "qualisys");
  ros::NodeHandle nh("~");

  qualisys::QualisysDriver qualisys_driver(nh);
  if(!qualisys_driver.init()) {
    ROS_INFO("Initialization of the qualisys driver failed!");
    return -1;
  }

  while(ros::ok())
  {
    qualisys_driver.run();
    ros::spinOnce();
  }

  ROS_INFO("Shutting down");
  qualisys_driver.disconnect();

  return 0;
}
