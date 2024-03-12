

#include "burg_scene_loader.h"
#include <iostream>
// #include <ostream>
#include <filesystem>
#include <algorithm>
namespace fs = std::filesystem;

BurgLoader::BurgLoader() : path("")
{
    std::cout << "burg loader constructor\n";
}

BurgLoader::BurgLoader(std::string path) : path(path)
{
    this->config = YAML::LoadFile(path);
    std::string lib_path = fs::path(path).parent_path() / fs::path(config["object_library_fn"].as<std::string>()).string();
    this->lib_path = lib_path;
    this->library = YAML::LoadFile(lib_path);
}

std::vector<std::tuple<std::string, Eigen::MatrixXd>>
BurgLoader::GetObjects(std::string objs_label)
{
    auto objects = this->config[objs_label];

    std::vector<std::tuple<std::string, Eigen::MatrixXd>> ret(objects.size());
    for (unsigned int i = 0; i < objects.size(); ++i)
    {
        Eigen::MatrixXd T(4, 4);
        auto pose = objects[i]["pose"];
        assert(pose.size() == 4);
        for (unsigned int k = 0; k < pose.size(); ++k)
        {
            assert(pose[k].size() == 4);
            for (unsigned int j = 0; j < pose.size(); ++j)
            {
                T(k, j) = pose[k][j].as<double>();
            }
        }
        std::string obj_type = objects[i]["object_type"].as<std::string>();
        auto library_objects = this->library["objects"];

        auto it = std::find_if(library_objects.begin(), library_objects.end(), [obj_type](YAML::Node element)
                               { return element["name"].as<std::string>() == obj_type; });

        std::string obj_path;
        if (it != library_objects.end())
        {
            auto obstacle = *it;
            obj_path = (fs::path(this->lib_path).parent_path() / fs::path(obstacle["mesh_fn"].as<std::string>())).string();
            // std::cout << "Name: " << obstacle["identifier"].as<std::string>() << " .obj: " << obj_path << "\n";
        }
        else
        {
            throw std::runtime_error("object library doesn't contain " + obj_type);
        }
        ret[i] = {obj_path, T};
        // std::cout << "path: " << obj_path << "\nT:\n"
        //           << T << "\n";
    }
    return ret;
}

std::vector<std::tuple<std::string, Eigen::MatrixXd>>
BurgLoader::GetObstacles()
{
    auto objs = this->GetObjects("objects");
    auto bg_objs = this->GetObjects("bg_objects");
    objs.insert(objs.end(), bg_objs.begin(), bg_objs.end());
    return objs;
}
