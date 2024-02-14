
#ifndef GRASPS_H
#define GRASPS_H

#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <memory>
// #include "vec_dist_pair.h"
#include "robot_state.h"

namespace Burs
{
    using namespace Eigen;

    class Grasp
    {
    public:
        Grasp(Vector3d pos);
        Grasp(std::string grasp_data_csv);
        Grasp() = default;

        virtual ~Grasp() = default;

    protected:
        KDL::Frame
        ToFrame() const;

    public:
        static std::vector<Grasp>
        LoadGrasps(const std::string &path_to_grasps_csv_file);

        static Eigen::Matrix4d
        ConvertCSVToMatrix4d(const std::string &csv);

        static std::vector<KDL::Frame>
        GraspsToFrames(const std::vector<Grasp> &grasps);

    public:
        Eigen::Matrix4d data;
        KDL::Frame frame;

    public:
        double best_dist;
        int best_state = -1;
        // RS *best_state;
        // std::reference_wrapper<RS> best_state;
        // std::shared_ptr<VecDistPair> dv;

        /// Frame of best configuration
        // KDL::Frame best_frame;
    };

}
#endif
