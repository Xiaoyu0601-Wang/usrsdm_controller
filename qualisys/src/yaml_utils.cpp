#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <qualisys/yaml_utils.h>

namespace qualisys {
namespace yaml_utils {

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

// Read SE3 pose from a file
bool loadPoseFromFile(
    const std::string &filename, Eigen::Affine3d &pose) {

  // Init pose to identity.
  // If exception occurs, pose will be returned as identity.
  pose = Eigen::Affine3d::Identity();

  // Open File
  std::ifstream fin(filename.c_str());
  if(!fin.is_open())
    return false;

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
      pose.translate(v);
      pose.rotate(q);
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

  return ret;
}

// Save SE3 pose to a file
bool savePoseToFile(
    const Eigen::Affine3d &pose, const std::string &filename) {

  YAML::Emitter out;

  Eigen::Vector3d v(pose.translation());
  Eigen::Quaterniond q(pose.rotation());

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

  // Open the file
  std::ofstream fout(filename.c_str(),
      std::ios_base::out | std::ios_base::trunc);
  if(!fout.is_open())
    return false;

  bool ret = true;

  fout << out.c_str() << std::endl;
  if(!fout.good())
    ret = false;

  fout.close();

  return ret;
}


} // End namespace yaml_utils
} // End namespace qualisys
