#include "robot_related/robot_collision.h"

namespace Burs
{

    RobotCollision::RobotCollision(std::string urdf_filename) : RobotBase(urdf_filename)
    {
        // Get the directory of the URDF file
        std::filesystem::path urdf_path(urdf_filename);
        std::filesystem::path urdf_dir = urdf_path.parent_path();

        int numModels = 0;
        // Initialization specific to RobotCollision
        // std::cout << "number of segments " << this->kdl_chain.getNrOfSegments() << std::endl;
        for (int i = 0; i < this->kdl_chain.getNrOfSegments(); ++i)
        {
            if (this->segmentIdToFile.find(i) != this->segmentIdToFile.end())
            {
                // add relative path to urdf file
                // std::cout << "\nADDING MODEL " << this->segmentIdToFile[i] << std::endl;
                std::filesystem::path model_path = urdf_dir / this->segmentIdToFile[i];
                std::shared_ptr<RtModels::RtModel> trpqpmodel = std::make_shared<RtModels::RtModel>(model_path);
                this->segmentIdToModel.push_back(trpqpmodel);
                numModels++;
            }
            else
            {
                this->segmentIdToModel.push_back({});
            }
        }
        this->numberOfModels = numModels;
        std::cout << "Initialized RobotCollision. Number of models: " << this->numberOfModels << std::endl;
    }

    // ForwardQ returns N rotations and translation, but we have M <= N objects, select only transforms relevant to existing meshes
    std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>
    RobotCollision::SelectedForwardQ(const VectorXd &q_in)
    {

        std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>> rawRts = this->ForwardQ(q_in);

        std::vector<Matrix3d> rawRotations = std::get<0>(rawRts);
        std::vector<Vector3d> rawTranslations = std::get<1>(rawRts);

        std::vector<Matrix3d> Rs(this->numberOfModels);
        std::vector<Vector3d> ts(this->numberOfModels);

        int k = 0;
        for (int i = 0; i < this->segmentIdToModel.size(); ++i)
        {
            if (this->segmentIdToModel[i])
            {
                ts[k] = rawTranslations[i];
                Rs[k] = rawRotations[i];
                k++;
            }
        }

        if (k != this->numberOfModels)
        {
            throw std::runtime_error("Number of models in forward doesn't match initialized number of models. ");
        }
        return make_tuple(Rs, ts);
    }

    ForwardRt
    RobotCollision::GetSelectedForwardRtFunc()
    {
        ForwardRt srt = [this](const VectorXd &configuration) -> std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>
        {
            return this->SelectedForwardQ(configuration);
        };
        return srt;
    }

    std::vector<std::shared_ptr<RtModels::RtModel>>
    RobotCollision::GetModels()
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
}
