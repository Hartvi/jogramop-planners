#include "robot_collision.h"

namespace Burs
{

    RobotCollision::RobotCollision(std::string urdf_filename) : RobotBase(urdf_filename)
    {
        // Get the directory of the URDF file
        std::filesystem::path urdf_path(urdf_filename);
        std::filesystem::path urdf_dir = urdf_path.parent_path();

        this->urdf_filename = urdf_dir / urdf_path;

        int numModels = 0;
        // Initialization specific to RobotCollision
        std::cout << "URDF dir: " << urdf_dir << "\n";
        for (int i = 0; i < this->kdl_chain.getNrOfSegments(); ++i)
        {
            const KDL::Segment &segment = kdl_chain.getSegment(i);
            std::cout << "Segment name:     " << this->kdl_chain.getSegment(i).getName() << "\n";
            if (this->segmentIdToFile.find(i) != this->segmentIdToFile.end())
            {
                // Add relative path to urdf file
                std::filesystem::path model_path = urdf_dir / this->segmentIdToFile[i];

                // for later visualization purposess
                this->mObjs.push_back(model_path);

                std::shared_ptr<RtModels::RtModel> trpqpmodel = std::make_shared<RtModels::RtModel>(model_path);
                std::cout << "Robot segment:    " << i << " \n  File:           " << this->segmentIdToFile[i] << " \n  Number of tris: " << trpqpmodel->pqpModel->num_tris << "\n";
                this->segmentIdToModel.push_back(trpqpmodel);
                numModels++;
            }
            else
            {
                this->segmentIdToModel.push_back({});
            }
            std::cout << "\n";
        }
        this->numberOfModels = numModels;

        auto movablejoints = this->MovableJoints();
        this->segmentToJntCausality = movablejoints;
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

    std::vector<VectorXd>
    RobotCollision::MovableJoints() const
    {
        // TODO create a mask for each case so that when I generate random samples I can zero out joints that aren't supposed to be moved
        // each segment denotes joint ids that affect the segment
        // segment 1 => joints 0,1,2 affect it => 2
        std::vector<VectorXd> segmentToJointVector(this->numberOfModels, VectorXd::Ones(this->kdl_chain.getNrOfJoints()));
        std::vector<int> segmentToJointCausality;
        // lastInactiveSegment is the id of the segment that had the distance too close
        // It is only from the set of segments that have models:
        int joint = 0;
        /*
        lastInactiveSegment: 4/8
        i:0
         nothing
        i:1
         joint+=1
        i:2
         joint+=1
         segments+=1
        i:3
         joint+=1
         segments+=1
        i:4
         joint+=1
         segments+=1
        i:5
         joint+=1
         segments+=1
        */
        int k = 0;
        for (unsigned int i = 0; i < this->segmentIdToModel.size(); ++i)
        {
            if (this->kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
            {
                ++joint;
            }

            if (this->segmentIdToModel[i])
            {
                for (int l = k; l < segmentToJointVector.size(); ++l)
                {
                    for (int m = 0; m < joint; ++m)
                    {
                        segmentToJointVector[l](m) = 0.0;
                    }
                }
                segmentToJointCausality.push_back(joint);
                ++k;
            }
            else
            {
            }
        }
        return segmentToJointVector;
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

}
