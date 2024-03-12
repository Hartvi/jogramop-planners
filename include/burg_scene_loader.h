
#ifndef BURG_SCENE_LOADER
#define BURG_SCENE_LOADER

// #include <yaml-cpp/yaml.h>
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

class BurgLoader
{
public:
    BurgLoader();
    BurgLoader(std::string path);

    std::vector<std::tuple<std::string, Eigen::MatrixXd>>
    GetObjects(std::string objs_label);

    std::vector<std::tuple<std::string, Eigen::MatrixXd>>
    GetObstacles();

    std::string path;
    std::string lib_path;
    YAML::Node config;
    YAML::Node library;
};
#endif
