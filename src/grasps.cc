#include "grasps.h"
#include <iostream>
#include <sstream>
#include <fstream>

namespace Burs
{
    Grasp::Grasp(std::string grasp_data_csv)
    {
        // expecting row0, row1, row2, row3
        // row{i} = el1,el2,el3,el4
        this->data = Grasp::ConvertCSVToMatrix4d(grasp_data_csv);
        this->frame = this->ToFrame();
        // this->dv = std::make_shared<VecDistPair>();
    }

    std::vector<KDL::Frame>
    Grasp::GraspsToFrames(const std::vector<Grasp> &grasps)
    {
        std::vector<KDL::Frame> frames;
        frames.reserve(grasps.size());

        for (const auto &item : grasps)
        {
            frames.push_back(item.ToFrame());
        }
        return frames;
    }

    std::vector<Grasp>
    Grasp::LoadGrasps(const std::string &path_to_grasps_csv_file)
    {
        std::vector<Grasp> grasps;
        std::ifstream file(path_to_grasps_csv_file);
        std::string line;

        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + path_to_grasps_csv_file);
        }

        while (std::getline(file, line))
        {
            Grasp grasp(line);
            grasps.push_back(grasp);
        }

        file.close();
        return grasps;
    }

    KDL::Frame
    Grasp::ToFrame() const
    {
        KDL::Rotation rotation(
            data(0, 0), data(0, 1), data(0, 2),
            data(1, 0), data(1, 1), data(1, 2),
            data(2, 0), data(2, 1), data(2, 2));
        KDL::Vector position(data(0, 3), data(1, 3), data(2, 3));

        return KDL::Frame(rotation, position);
    }

    Eigen::Matrix4d
    Grasp::ConvertCSVToMatrix4d(const std::string &csv)
    {
        std::stringstream ss(csv);
        std::string item;
        std::vector<double> matrixElements;

        // Split the string by ','
        while (std::getline(ss, item, ','))
        {
            // Parse each string to double
            matrixElements.push_back(std::stod(item));
        }

        // Check if we have exactly 16 elements (for a 4x4 matrix)
        if (matrixElements.size() != 16)
        {
            throw std::runtime_error("Input string does not contain enough elements for a 4x4 matrix.");
        }

        // Populate the Eigen::Matrix4d
        Eigen::Matrix4d matrix;
        for (int i = 0; i < 16; ++i)
        {
            matrix(i / 4, i % 4) = matrixElements[i];
        }

        return matrix;
    }
}
