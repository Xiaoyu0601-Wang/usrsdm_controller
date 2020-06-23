/*
 * Copyright 2012 Kartik Mohta <kartikmohta@gmail.com>
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

#ifndef QUALISYS_CALIB_H
#define QUALISYS_CALIB_H

#include <map>
#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

#include <qualisys/Subject.h>
//#include <qualisys/SetPose.h>

namespace qualisys {

  /*
   * @brief QulisysCalib Perform the calibration process for a
   *  new model.
   */
  class QualisysCalib{
    public:
      /*
       * @brief Constructor and Destructor
       */
      QualisysCalib(const ros::NodeHandle& n);
      ~QualisysCalib() {}

      /*
       * @brief init Initialize the object
       * @param verbose If true, print various info
       * @return If the init sequence is successful
       */
      bool init();

    private:
      ros::NodeHandle nh;

      std::string calib_marker_pos_file;
      std::string zero_pose_dir;
      std::string model_name;

      std::map<std::string, Eigen::Vector3d> marker_pos_map;
      std::vector<Eigen::Vector3d> calib_ref_points;
      std::vector<Eigen::Vector3d> calib_actual_points;
      Eigen::Affine3d calib_transform;

      //bool enable_calibration = false;
      //bool calib_stand_ready = false;
      bool enable_calibration;
      bool calib_stand_ready;

      ros::Publisher zero_pose_pub;


      // Disable copy constructor and assign operator
      QualisysCalib(const QualisysCalib&);
      QualisysCalib& operator=(const QualisysCalib&);

      // Load zero pose from file
      bool loadZeroPoseFromFile(const std::string &filename,
          Eigen::Affine3d &zero_pose);

      // Save zero pose to file
      bool saveZeroPoseToFile(const Eigen::Affine3d &zero_pose,
          const std::string &filename);

      // Load markers' positions of the calib stand from file
      bool loadCalibMarkerPos(const std::string &filename,
          std::map<std::string, Eigen::Vector3d> &marker_pos_map);

      // Get SE3 transfrom between reference points and acutal points
      bool getTransform(const std::vector<Eigen::Vector3d> &reference_points,
          const std::vector<Eigen::Vector3d> &actual_points,
          Eigen::Affine3d &transform);

      // Callback for receiving markers' positions of calib stand
      void calibStandCallback(const qualisys::Subject::ConstPtr &msg);

      // Callback for receiving markers' positions of the model
      void subjectCallback(const qualisys::Subject::ConstPtr &msg);

      // Callback for toggling calibration
      bool toggleCalibCallback(std_srvs::Empty::Request &req,
          std_srvs::Empty::Response &res);
  };
}

#endif
