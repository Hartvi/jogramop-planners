#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

#ifndef GRASPS_H
#define GRASPS_H

namespace Burs
{
    class Grasp
    {
    public:
        Grasp(std::string grasp_data_csv);

        KDL::Frame
        ToFrame();

    public:
        static std::vector<Grasp>
        LoadGrasps(const std::string &path_to_grasps_csv_file);

        static Eigen::Matrix4d
        ConvertCSVToMatrix4d(const std::string &csv);

    public:
        Eigen::Matrix4d data;
    };
}
#endif