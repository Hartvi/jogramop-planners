#include "robot_collision.h"

namespace Burs
{

    RobotCollision::RobotCollision(std::string urdf_filename) : RobotBase(urdf_filename)
    {
        // // Get the directory of the URDF file
        // std::filesystem::path urdf_path(urdf_filename);
        // std::filesystem::path urdf_dir = urdf_path.parent_path();

        // this->urdf_filename = urdf_dir / urdf_path;

        // int numModels = 0;
        // // Initialization specific to RobotCollision
        // // std::cout << "number of segments " << this->kdl_chain.getNrOfSegments() << std::endl;
        // std::cout << "URDF dir: " << urdf_dir << "\n";
        // for (int i = 0; i < this->kdl_chain.getNrOfSegments(); ++i)
        // {
        //     const KDL::Segment &segment = kdl_chain.getSegment(i);
        //     std::cout << "Segment name:     " << this->kdl_chain.getSegment(i).getName() << "\n";
        //     if (this->segmentIdToFile.find(i) != this->segmentIdToFile.end())
        //     {
        //         // Add relative path to urdf file
        //         std::filesystem::path model_path = urdf_dir / this->segmentIdToFile[i];

        //         // for later visualization purposess
        //         this->mObjs.push_back(model_path);

        //         std::shared_ptr<RtModels::RtModel> trpqpmodel = std::make_shared<RtModels::RtModel>(model_path);
        //         std::cout << "Robot segment:    " << i << " \n  File:           " << this->segmentIdToFile[i] << " \n  Number of tris: " << trpqpmodel->pqpModel->num_tris << "\n";
        //         // std::cout << "radii: " << trpqpmodel->encompassingRadii.transpose() << "\n";
        //         this->segmentIdToModel.push_back(trpqpmodel);
        //         numModels++;
        //     }
        //     else
        //     {
        //         this->segmentIdToModel.push_back({});
        //     }
        //     std::cout << "\n";
        // }
        // // throw std::runtime_error("ROBOT COLLISION DEBUG THROW");
        // this->numberOfModels = numModels;

        // // std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n";
        // // std::cout << "num models: " << this->numberOfModels << "\n";
        // auto movablejoints = this->MovableJoints();
        // this->segmentToJntCausality = movablejoints;
        // // std::cout << "MOVABLE JOINT MAPPING\n";
        // // for (unsigned int i = 0; i < movablejoints.size(); ++i)
        // // {
        // //     std::cout << "segment " << i << ": " << movablejoints[i] << "\n";
        // // }
        // // exit(0);
        // // std::cout << "Initialized RobotCollision. Number of models: " << this->numberOfModels << std::endl;
    }

    std::vector<bool>
    RobotCollision::GetValidTransforms()
    {
        unsigned int l = this->segmentIdToModel.size();
        std::vector<bool> b(l);
        int k = 0;
        for (unsigned int i = 0; i < l; ++i)
        {
            b[i] = this->segmentIdToModel[i] ? true : false;
            if (b[i])
            {
                k++;
            }
        }
        std::cout << "num valid models: " << k << "\n";
        return b;
    }

    std::vector<std::shared_ptr<RtModels::RtModel>> RobotCollision::GetModels()
    {
        std::vector<std::shared_ptr<RtModels::RtModel>> models(this->numberOfModels);
        int k = 0;
        for (int i = 0; i < this->segmentIdToModel.size(); ++i)
        {
            if (this->segmentIdToModel[i])
            {
                models[k] = this->segmentIdToModel[i].value();
                k++;
            }
        }
        return models;
    }

    double
    RobotCollision::EEDistance(const RS &state1, const RS &state2) const
    {
        return (this->GetEEFrame(state1).p - this->GetEEFrame(state2).p).Norm();
    }

    KDL::Frame
    RobotCollision::GetEEFrame(const RS &state) const
    {
        return state.frames.back();
    }

    std::pair<int, std::vector<double>>
    RobotCollision::MaxDistances(const RS &state1, const RS &state2) const
    {
        auto f1 = state1.frames;
        auto f2 = state2.frames;

        double max_dist = 0;
        int max_idx = -1;
        std::vector<double> dists(f1.size());
        for (unsigned int i = 0; i < f1.size(); ++i)
        {
            // dist is in meters
            double dist = (f1[i].p - f2[i].p).Norm();
            dists[i] = dist;
            if (dist > max_dist)
            {
                max_dist = dist;
                max_idx = i;
            }
        }
        return {max_idx, dists};
    }

    double
    RobotCollision::MaxDistance(const RS &state1, const RS &state2) const
    {
        auto f1 = state1.frames;
        auto f2 = state2.frames;

        double max_dist = 0;
        for (unsigned int i = 0; i < f1.size(); ++i)
        {
            // dist is in meters
            double dist = (f1[i].p - f2[i].p).Norm();
            if (dist > max_dist)
            {
                max_dist = dist;
            }
        }
        // std::cout << "Wire max dist: " << max_dist << "\n";
        // double mesh_dist = this->MaxDistanceMeshes(state1, state2);
        // std::cout << "Mesh max dist: " << mesh_dist << "\n\n";
        return max_dist;
        // return std::max(mesh_dist, max_dist);
    }

    std::vector<double>
    RobotCollision::MaxDistanceMeshPositions(const RS &state1, const RS &state2) const
    {
        // TODO: for small changes this should return small values
        auto f1 = state1.frames;
        auto f2 = state2.frames;
        std::vector<double> max_dists(this->numberOfModels);
        int frame_id = 0;
        for (auto &it : this->segmentIdToModel)
        {
            if (it)
            {
                auto rtmodel = it.value();
                auto num_tris = rtmodel->pqpModel->num_tris;
                // std::cout << "Num tris: " << num_tris << "\n";
                auto tmpf2 = state2.frames[frame_id];
                auto tmpf1 = state1.frames[frame_id];
                double dist = (tmpf1.p - tmpf2.p).Norm();
                max_dists[frame_id] = dist;
            }
            ++frame_id;
        }
        // std::cout << "\n\n\n\n";
        return max_dists;
    }

    double
    RobotCollision::MaxDistanceMeshes(const RS &state1, const RS &state2) const
    {
        // TODO: for small changes this should return small values
        auto f1 = state1.frames;
        auto f2 = state2.frames;
        double max_dist = 0;
        int frame_id = 0;
        for (auto &it : this->segmentIdToModel)
        {
            if (it)
            {
                auto rtmodel = it.value();
                auto num_tris = rtmodel->pqpModel->num_tris;
                // std::cout << "Num tris: " << num_tris << "\n";
                auto tmpf2 = state2.frames[frame_id];
                auto tmpf1 = state1.frames[frame_id];
                auto deltaR2 = tmpf2.M;

                for (size_t i = 0; i < num_tris; ++i)
                {
                    auto p1 = rtmodel->pqpModel->tris[i].p1;
                    auto p1vec = KDL::Vector(p1[0], p1[1], p1[2]);
                    auto tmp_dist = (tmpf1 * p1vec - tmpf2 * p1vec).Norm();
                    if (tmp_dist > max_dist)
                    {
                        max_dist = tmp_dist;
                    }
                }
            }
            ++frame_id;
        }
        // std::cout << "\n\n\n\n";
        return max_dist;
    }

    std::pair<Matrix3d, Vector3d>
    RobotCollision::KDLFrameToEigen(const KDL::Frame &f)
    {
        Vector3d t;
        // Directly assign values
        t.x() = f.p.x();
        t.y() = f.p.y();
        t.z() = f.p.z();
        // KDL USES ROW MAJOR
        // EIGEN USES COL MAJOR => COPY INDEX BY INDEX
        Matrix3d R;
        for (unsigned int l = 0; l < 3; ++l)
        {
            for (unsigned int m = 0; m < 3; ++m)
            {
                R(l, m) = f.M(l, m);
            }
        }
        return {R, t};
    }

    // std::optional<VectorXd>
    // RobotCollision::GetInverseKinematics(KDL::ChainIkSolverPos_LMA &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt)
    // {
    //     KDL::JntArray res(q_init.rows());
    //     // KDL::ChainIkSolverPos_LMA::E_NOERROR
    //     if (solver.CartToJnt(q_init, tgt, res) >= 0)
    //     {
    //         return res.data;
    //     }
    //     return {};
    // }

    std::optional<VectorXd>
    RobotCollision::GetInverseKinematics(KDL::ChainIkSolverPos &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt)
    {
        KDL::JntArray res(q_init.rows());
        // KDL::ChainIkSolverPos_LMA::E_NOERROR
        if (solver.CartToJnt(q_init, tgt, res) >= 0)
        {
            return res.data;
        }
        return {};
    }

}
