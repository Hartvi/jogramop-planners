
#ifndef SCENE_H
#define SCENE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "grasps.h"

class ObjFile
{
public:
    std::string obj_path;
    Eigen::Matrix4d obj_pose;
};

class GraspFile
{
public:
    std::string grasp_file;
    std::vector<Burs::Grasp> grasps;
};

class RobotFile
{
    std::string robot_file;
    Eigen::Matrix4d robot_pose;
};

class Scene
{
    std::vector<ObjFile> obj_files;
    std::vector<GraspFile> grasp_files;
    RobotFile robot_file;
};
#endif