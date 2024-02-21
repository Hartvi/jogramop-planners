#include <iostream>
#include <string>
#include "rt_model.h"
#include "tiny_obj_loader.h"
#include "pqp_load.h"

namespace RtModels
{
    // Implementation of constructor and destructor
    RtModel::RtModel(std::string filePath)
        : filePath(filePath)
    {
        if (filePath.empty())
        {
            throw std::invalid_argument("filePath cannot be empty");
        }
        tinyobj::ObjReader reader;
        std::shared_ptr<PQP_Model> obj1 = std::make_shared<PQP_Model>();

        pqploader::read_file(filePath, reader);

        if (pqploader::tiny_OBJ_to_PQP_model(reader, obj1.get()))
        {
            // std::cout << filePath << " buildstate: " << obj1->build_state << std::endl;
        }

        // Initialize class members they should auto-delete
        this->pqpModel = obj1;

        this->SetRotation(Eigen::Matrix<PQP_REAL, 3, 3>::Identity()); // Initialized as identity matrix
        this->SetTranslation(Eigen::Vector3d::Zero());                // Initialized as zero vector
        // std::cout << "RtModel: Loaded " << filePath << std::endl;

        // this->filePath = filePath;
    }

    std::string
    RtModel::GetFilePath() const
    {
        std::filesystem::path cwd = std::filesystem::current_path();
        std::filesystem::path relPath(this->filePath);
        std::filesystem::path absPath = cwd / relPath;
        return absPath.string();
    }

    std::string
    RtModel::ToString()
    {
        std::ostringstream os;

        os << std::fixed << std::setprecision(4);

        os << "file" << std::endl;
        os << this->GetFilePath() << std::endl;
        os << "R" << std::endl;

        auto R = this->getR();
        for (size_t i = 0; i < 3; ++i)
        {
            auto row = R[i];
            for (size_t j = 0; j < 3; ++j)
            {
                os << row[j] << (j < 2 ? "," : "");
            }
            os << std::endl;
        }

        os << "t" << std::endl;
        auto T = this->getT();
        for (size_t i = 0; i < 3; ++i)
        {
            os << T[i] << (i < 2 ? "," : "");
        }
        os << "\n\n";

        return os.str();
    }

    PQP_REAL(*RtModel::getR())
    [3]
    {
        return this->rotation;
    }

    PQP_REAL *
    RtModel::getT()
    {
        return this->translation;
    }

    void
    RtModel::Rotate(Eigen::Matrix<double, 3, 3> rotation)
    {
        this->R = rotation * this->R;
        Eigen::Map<Eigen::Matrix<PQP_REAL, 3, 3, Eigen::RowMajor>>(this->rotation[0], 3, 3) = this->R;
    }

    void
    RtModel::SetRotation(Eigen::Matrix<double, 3, 3> rotation)
    {
        this->R = rotation;
        Eigen::Map<Eigen::Matrix<PQP_REAL, 3, 3, Eigen::RowMajor>>(this->rotation[0], 3, 3) = this->R;
    }

    void
    RtModel::Translate(Eigen::Vector3d tr)
    {
        this->t = this->t + tr;
        Eigen::Map<Eigen::Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
    }

    void
    RtModel::SetTranslation(Eigen::Vector3d tr)
    {
        this->t = tr;
        Eigen::Map<Eigen::Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
    }

    Eigen::Vector3d
    RtModel::GetGlobalPositionFromVector(Eigen::Vector3d p) const
    {
        return this->t + this->R * p;
    }

    Eigen::Vector3d
    RtModel::GetGlobalPositionFromPointer(PQP_REAL p[3]) const
    {
        Eigen::Map<Eigen::Matrix<PQP_REAL, 3, 1>> pVector(p);
        return this->t + this->R * pVector;
    }

    void
    RtModel::CheckDistanceStatic(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, RtModel *m1, RtModel *m2)
    {
        PQP_Distance(result, m1->getR(), m1->getT(), m1->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get(), rel_err, abs_err);
    }

    void
    RtModel::CollideStatic(PQP_CollideResult *result, RtModel *m1, RtModel *m2)
    {
        PQP_Collide(result, m1->getR(), m1->getT(), m1->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get());
    }

    void
    RtModel::CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, RtModel *m2)
    {
        PQP_Distance(result, this->getR(), this->getT(), this->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get(), rel_err, abs_err);
    }

    void
    RtModel::Collide(PQP_CollideResult *result, RtModel *m2)
    {
        PQP_Collide(result, this->getR(), this->getT(), this->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get());
    }

}