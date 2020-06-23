/*
 * Copyright 2015 Kartik Mohta
 * Author: Kartik Mohta <kartikmohta@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sstream>
#include <math.h>
#include <string>

// Including ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <qualisys/Subject.h>
#include <qualisys/RTProtocol.h>

int main(int argc, char *argv[])
{
  const float deg2rad = M_PI / 180.;

  // Defining global variables
  float x, y, z, roll, pitch, yaw;
  uint bodyCount;
  uint frameNumber;

  CRTPacket *pRTPacket;
  CRTPacket::EPacketType eType;

  ros::init(argc, argv, "Qualisys2Ros");
  ros::NodeHandle nh("~");
  tf::TransformBroadcaster publisher;

  std::string serverAddress;
  int basePort;
  nh.param("server_address", serverAddress, std::string("192.168.254.1"));
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  nh.param("server_base_port", basePort, 22222);

  bool publishers_initialized_(false);
  std::vector<ros::Publisher> pub_vicon_subject_array;

  // Defining a protocol that connects to the Qualisys
  CRTProtocol poRTProtocol;

  // Connecting to the server
  ROS_INFO_STREAM(
      "Connecting to the Qualisys Motion Tracking system specified at: "
      << serverAddress << ":" << basePort);

  if(!poRTProtocol.Connect((char *)serverAddress.data(), basePort, 0, 1, 7))
  {
    ROS_FATAL_STREAM("Could not find the Qualisys Motion Tracking system at: "
                     << serverAddress << ":" << basePort);
    return 0;
  }

  ROS_INFO_STREAM("Connected to " << serverAddress << ":" << basePort);
  ROS_INFO_STREAM("Entering the measurement streaming loop...");

  int createBodyBufferFlag = 0;
  int createMarkerBufferFlag = 0;
  int printOnce = 0;

  poRTProtocol.Read6DOFSettings();

  // Infinite Measurement Loop
  while(ros::ok())
  {
    pRTPacket = poRTProtocol.GetRTPacket();
    frameNumber = pRTPacket->GetFrameNumber();

    poRTProtocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

    if(poRTProtocol.ReceiveRTPacket(eType, true))
    {
      switch(eType)
      {
        // Case 1 - sHeader.nType 0 indicates an error
        case CRTPacket::PacketError:
          ROS_ERROR_STREAM_THROTTLE(
              1, "Error when streaming frames: "
                     << poRTProtocol.GetRTPacket()->GetErrorString());
          break;
        case CRTPacket::PacketNoMoreData: // No more data
          ROS_WARN_STREAM_THROTTLE(1, "No more data");
          break;

        // Case 2 - Data received
        case CRTPacket::PacketData:
          bodyCount = pRTPacket->Get6DOFEulerBodyCount();
          if(bodyCount <= 0)
          {
            ROS_WARN_THROTTLE(1, "No Bodies Found");
          }
          else
          {
            if(!publishers_initialized_)
            {
              for(unsigned int i = 0; i < bodyCount; i++)
              {
                // Create publishers
                std::stringstream name;
                // name << "Q" << i;
                name << poRTProtocol.Get6DOFBodyName(i);
                ros::Publisher pub_subject =
                    nh.advertise<qualisys::Subject>(name.str(), 10);
                pub_vicon_subject_array.push_back(pub_subject);
                publishers_initialized_ = true;
              }
            }

            for(unsigned int i = 0; i < bodyCount; i++)
            {
              pRTPacket->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);
              if(isnan(x) || isnan(y) || isnan(z) || isnan(roll) ||
                 isnan(pitch) || isnan(yaw))
              {
                ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " << i + 1 << "/"
                                                          << bodyCount
                                                          << " not detected");
              }
              else
              {
                // ROTATION: GLOBAL (FIXED) X Y Z (R P Y)
                std::stringstream name;
                // name << "Q" << i;
                name << poRTProtocol.Get6DOFBodyName(i);
                // Qualisys sometimes flips 180 degrees around the x axis
                if(roll > 90)
                {
                  roll -= 180;
                }
                else if(roll < -90)
                {
                  roll += 180;
                }
                tf::StampedTransform stamped_transform = tf::StampedTransform(
                    tf::Transform(
                        tf::createQuaternionFromRPY(
                            roll * deg2rad, pitch * deg2rad, yaw * deg2rad),
                        tf::Vector3(x, y, z) / 1000.),
                    ros::Time::now(), "Qualisys", name.str());
                publisher.sendTransform(stamped_transform);
                geometry_msgs::TransformStamped geom_stamped_transform;
                tf::transformStampedTFToMsg(stamped_transform,
                                            geom_stamped_transform);

                qualisys::Subject subject_msg;
                subject_msg.header = geom_stamped_transform.header;
                subject_msg.name = name.str();
                subject_msg.position.x =
                    geom_stamped_transform.transform.translation.x;
                subject_msg.position.y =
                    geom_stamped_transform.transform.translation.y;
                subject_msg.position.z =
                    geom_stamped_transform.transform.translation.z;
                subject_msg.orientation =
                    geom_stamped_transform.transform.rotation;
                pub_vicon_subject_array[i].publish(subject_msg);
              }
            }
          }
          break;

        default:
          ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");
      }
    }
    ros::spinOnce();
  }

  ROS_INFO("Shutting down");
  poRTProtocol.StreamFramesStop();
  poRTProtocol.Disconnect();
  return 0;
}
