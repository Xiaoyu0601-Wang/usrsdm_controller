#include <ros/ros.h>
#include <ros/node_handle.h>
#include <qualisys/QualisysOdom.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qualisys_odom");
  ros::NodeHandle nh("~");

  try
  {
    qualisys::QualisysOdom qualisys_odom(nh);
    qualisys_odom.init();
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}
