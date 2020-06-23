#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <qualisys/QualisysCalib.h>

using namespace std;
using namespace ros;
using namespace Eigen;

// Support for new yaml-cpp, taken from
// https://github.com/ros-planning/moveit_setup_assistant/commit/c9238ca#diff-1
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
static void operator>>(const YAML::Node &node, T &i)
{
  i = node.as<T>();
}
#endif

// yaml-cpp 0.5 also changed how you load the YAML document.  This
// function hides the changes.
static void loadYaml(std::istream &in_stream, YAML::Node &doc_out)
{
#ifdef HAVE_NEW_YAMLCPP
  doc_out = YAML::Load(in_stream);
#else
  YAML::Parser parser(in_stream);
  parser.GetNextDocument(doc_out);
#endif
}

namespace qualisys
{

QualisysCalib::QualisysCalib(const ros::NodeHandle& n):
  nh(n),
  enable_calibration(false),
  calib_stand_ready(false){
  return;
}

bool QualisysCalib::init() {

  // Load the name of files
  nh.param("calib_marker_pos_file", calib_marker_pos_file,
      std::string("QuadrotorCalib.yaml"));
  nh.param("zero_pose_dir", zero_pose_dir,
      std::string("calib"));

  // Load the positon of markers on the calib stand
  if(!loadCalibMarkerPos(calib_marker_pos_file, marker_pos_map)) {
    ROS_ERROR("Error loading calib marker positions from file: %s",
              calib_marker_pos_file.c_str());
    return false;
  }

  // Resize the number of markers on the stand accordingly.
  calib_ref_points.resize(marker_pos_map.size());
  calib_actual_points.resize(marker_pos_map.size());

  // Create publishers, subscribers, and services
  zero_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("zero_pose", 10);
  Subscriber calib_sub = nh.subscribe("qualisys_calib", 10,
      &QualisysCalib::calibStandCallback, this,
      ros::TransportHints().tcp().tcpNoDelay());
  Subscriber subject_sub = nh.subscribe("qualisys_subject", 10,
      &QualisysCalib::subjectCallback, this,
      ros::TransportHints().tcp().tcpNoDelay());
  ServiceServer toggle_calib_srv = nh.advertiseService("toggle_calibration",
      &QualisysCalib::toggleCalibCallback, this);

  return true;
}

bool QualisysCalib::loadZeroPoseFromFile(const std::string &filename,
    Eigen::Affine3d &zero_pose)
{
  zero_pose = Eigen::Affine3d::Identity();

  std::ifstream fin(filename.c_str());
  if(!fin.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening file: " << filename
              << ", setting zero pose to identity" << std::endl;
#endif
    return false;
  }

  bool ret = false;

  try
  {
    YAML::Node doc;
    loadYaml(fin, doc);

    try
    {
      Eigen::Vector3d v;
      Eigen::Quaterniond q;
      doc["translation"]["x"] >> v(0);
      doc["translation"]["y"] >> v(1);
      doc["translation"]["z"] >> v(2);
      doc["rotation"]["x"] >> q.x();
      doc["rotation"]["y"] >> q.y();
      doc["rotation"]["z"] >> q.z();
      doc["rotation"]["w"] >> q.w();
      zero_pose.translate(v);
      zero_pose.rotate(q);
      ret = true;
    }
    catch(YAML::KeyNotFound &e)
    {
      ret = false;
    }
  }
  catch(YAML::ParserException &e)
  {
    ret = false;
  }
  fin.close();

  if(!ret)
  {
#if DEBUG
    std::cerr << "Error parsing calib file: " << filename
              << ", setting zero_pose to Identity" << std::endl;
#endif
  }
  return ret;
}

bool QualisysCalib::saveZeroPoseToFile(const Eigen::Affine3d &zero_pose,
    const std::string &filename)
{
  YAML::Emitter out;

  Eigen::Vector3d v(zero_pose.translation());
  Eigen::Quaterniond q(zero_pose.rotation());

  out << YAML::BeginMap;
  out << YAML::Key << "translation";
  out << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value
      << v.x() << YAML::Key << "y" << YAML::Value << v.y() << YAML::Key << "z"
      << YAML::Value << v.z() << YAML::EndMap;
  out << YAML::Key << "rotation";
  out << YAML::Value << YAML::BeginMap << YAML::Key << "x" << YAML::Value
      << q.x() << YAML::Key << "y" << YAML::Value << q.y() << YAML::Key << "z"
      << YAML::Value << q.z() << YAML::Key << "w" << YAML::Value << q.w()
      << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream fout(filename.c_str(),
                     std::ios_base::out | std::ios_base::trunc);
  if(!fout.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening file: " << filename << std::endl;
#endif
    return false;
  }

  bool ret = true;

  fout << out.c_str() << std::endl;
  if(!fout.good())
  {
#ifdef DEBUG
    std::cerr << "Error writing to file: " << filename << std::endl;
#endif
    ret = false;
  }

  fout.close();

  return ret;
}

bool QualisysCalib::getTransform(const std::vector<Eigen::Vector3d> &reference_points,
    const std::vector<Eigen::Vector3d> &actual_points,
    Eigen::Affine3d &transform)
{
  transform = Eigen::Affine3d::Identity();

  if(reference_points.size() != actual_points.size())
  {
    return false;
  }

  // Algorithm from http://dx.doi.org/10.1016/0021-9290(94)00116-L
  const size_t num_points = reference_points.size();
  Eigen::Vector3d reference_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d actual_mean = Eigen::Vector3d::Zero();
  for(size_t i = 0; i < num_points; i++)
  {
    reference_mean += reference_points[i];
    actual_mean += actual_points[i];
  }
  reference_mean /= num_points;
  actual_mean /= num_points;

  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for(size_t i = 0; i < num_points; i++)
  {
    C += (actual_points[i] - actual_mean) *
         (reference_points[i] - reference_mean).transpose();
  }
  C /= num_points;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      C, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d V = svd.matrixV();
  if(U.determinant() * V.determinant() < 0)
  {
    S(2, 2) = -1;
  }

  const Eigen::Matrix3d rotation = U * S * V.transpose();

  const Eigen::Vector3d translation = actual_mean - rotation * reference_mean;

  transform.translate(translation);
  transform.rotate(rotation);

  return true;
}

bool QualisysCalib::loadCalibMarkerPos(const std::string &filename,
    std::map<std::string, Eigen::Vector3d> &marker_pos_map)
{
  std::ifstream fin(filename.c_str());
  if(!fin.is_open())
  {
#ifdef DEBUG
    std::cerr << "Error opening calib marker position file: " << filename
              << std::endl;
#endif
    return false;
  }

  bool ret = false;

  try
  {
    YAML::Node doc;
    loadYaml(fin, doc);

    try
    {
      const YAML::Node &markers = doc["markers"];

      for(unsigned int i = 0; i < markers.size(); i++)
      {
        std::string name;
        markers[i]["name"] >> name;
        const YAML::Node &pos = markers[i]["position"];
        Eigen::Vector3d position;
        pos[0] >> position.x();
        pos[1] >> position.y();
        pos[2] >> position.z();
        // Use a simple way to add a pair to the map
        marker_pos_map[name] = position;
        //marker_pos_map.insert(
        //    std::make_pair<std::string, Eigen::Vector3d>(name, position));
      }
      ret = true;
    }
    catch(YAML::KeyNotFound &e)
    {
      ret = false;
    }
  }
  catch(YAML::ParserException &e)
  {
    ret = false;
  }
  fin.close();

  if(!ret)
  {
#if DEBUG
    std::cerr << "Error parsing calib marker position file: " << filename
              << std::endl;
#endif
    marker_pos_map.clear();
  }
  return ret;
}

void QualisysCalib::calibStandCallback(
    const qualisys::Subject::ConstPtr &msg)
{
  calib_ref_points.clear();
  calib_actual_points.clear();

  std::map<std::string, Eigen::Vector3d>::iterator it;
  for(size_t i = 0; i < msg->markers.size(); i++)
  {
    if(!msg->markers[i].occluded)
    {
      it = marker_pos_map.find(msg->markers[i].name);
      if(it != marker_pos_map.end())
      {
        calib_ref_points.push_back(it->second);
        calib_actual_points.push_back(Eigen::Vector3d(
            msg->markers[i].position.x, msg->markers[i].position.y,
            msg->markers[i].position.z));
      }
      else
      {
        ROS_WARN_THROTTLE(
            1, "Marker %s is not present in calib_marker_pos_file, skipping it",
            msg->markers[i].name.c_str());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1, "Marker %s is occluded, skipping it",
                        msg->markers[i].name.c_str());
    }
  }
  if(!QualisysCalib::getTransform(calib_ref_points, calib_actual_points,
                                        calib_transform))
  {
    ROS_WARN("QualisysCalib::getTransform failed");
    calib_stand_ready = false;
  }
  else
  {
    calib_stand_ready = true;
  }
}

void QualisysCalib::subjectCallback(const qualisys::Subject::ConstPtr &msg)
{
  static bool calibrating = false;
  static double t_x, t_y, t_z;
  static double q_x, q_y, q_z, q_w;
  static unsigned int count;

  if(enable_calibration && !calibrating)
  {
    // Start calib
    t_x = t_y = t_z = 0;
    q_x = q_y = q_z = q_w = 0;
    count = 0;
    calibrating = true;
  }
  else if(!enable_calibration && calibrating)
  {
    // Stop calib
    calibrating = false;

    //vicon::SetPose pose;
    Eigen::Affine3d zero_pose;
    zero_pose.setIdentity();
    Eigen::Quaterniond q(q_w / count, q_x / count, q_y / count, q_z / count);
    q.normalize();
    Eigen::Vector3d t(t_x/count, t_y/count, t_z/count);
    zero_pose.translate(t);
    zero_pose.rotate(q);

    model_name = msg->name;
    string model_zero_pose_file = zero_pose_dir+"/"+model_name+".yaml";
    saveZeroPoseToFile(zero_pose, model_zero_pose_file);
    //pose.request.subject_name = msg->name;
    //pose.request.pose.position.x = t_x / count;
    //pose.request.pose.position.y = t_y / count;
    //pose.request.pose.position.z = t_z / count;
    //pose.request.pose.orientation.x = q.x();
    //pose.request.pose.orientation.y = q.y();
    //pose.request.pose.orientation.z = q.z();
    //pose.request.pose.orientation.w = q.w();
    //set_zero_pose_srv.call(pose);
  }

  if(calibrating)
  {
    Eigen::Affine3d zero_pose, current_pose;
    Eigen::Vector3d t(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                         msg->orientation.y, msg->orientation.z);
    model_name = msg->name;
    current_pose.setIdentity();
    current_pose.translate(t);
    current_pose.rotate(q);

    zero_pose = calib_transform.inverse() * current_pose;

    t = zero_pose.translation();
    q = Eigen::Quaterniond(zero_pose.rotation());

    // Accumulate (to calculate mean)
    t_x += t(0);
    t_y += t(1);
    t_z += t(2);
    q_x += q.x();
    q_y += q.y();
    q_z += q.z();
    q_w += q.w();
    count++;

    geometry_msgs::PoseStamped::Ptr zero_pose_msg(
        new geometry_msgs::PoseStamped);
    zero_pose_msg->header.stamp = msg->header.stamp;
    zero_pose_msg->header.frame_id = "/qualisys";

    zero_pose_msg->pose.position.x = t_x / count;
    zero_pose_msg->pose.position.y = t_y / count;
    zero_pose_msg->pose.position.z = t_z / count;

    q = Eigen::Quaterniond(q_w / count, q_x / count, q_y / count, q_z / count);
    q.normalize();

    zero_pose_msg->pose.orientation.x = q.x();
    zero_pose_msg->pose.orientation.y = q.y();
    zero_pose_msg->pose.orientation.z = q.z();
    zero_pose_msg->pose.orientation.w = q.w();

    zero_pose_pub.publish(zero_pose_msg);
  }
}

bool QualisysCalib::toggleCalibCallback(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
  if(!calib_stand_ready)
  {
    ROS_ERROR("Do not have calib stand pose, cannot calibrate");
    enable_calibration = false;
    return false;
  }

  enable_calibration = !enable_calibration;
  return true;
}
}
