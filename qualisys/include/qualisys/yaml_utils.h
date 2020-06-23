#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <map>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace qualisys {
/*
 * @brief yaml_utils This is a namespace containing utility
 * functions to deal with yaml files, including load or save
 * parameters from or to a yaml file.
 */
namespace yaml_utils {

/*
 * @brief loadPoseFromFile Load a SE3 pose from a file
 * @param filename Name of the file
 * @param pose Pose read from file
 * @return True if successful
 */
bool loadPoseFromFile(const std::string &filename,
    Eigen::Affine3d &pose);

/*
 * @brief savePoseToFile Save a SE3 pose to a file
 * @param filename Name of the file
 * @param pose Pose to be saved
 * @return True if successful
 */
bool savePoseToFile(const Eigen::Affine3d &pose,
    const std::string &filename);
}
}

#endif
