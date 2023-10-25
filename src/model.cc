#include <iostream>
#include "model.h"
#include "tiny_obj_loader.h"
#include "load.h"

// Implementation of constructor and destructor
TrPQPModel::TrPQPModel(std::string filePath) : filePath(filePath)
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
        std::cout << filePath << " buildstate: " << obj1->build_state << std::endl;
    }

    // Initialize class members they should auto-delete
    this->pqpModel = obj1;

    this->SetRotation(Matrix<PQP_REAL, 3, 3>::Identity()); // Initialized as identity matrix
    this->SetTranslation(Vector3<PQP_REAL>::Zero());       // Initialized as zero vector
}

// TrPQPModel::TrPQPModel(std::shared_ptr<PQP_Model> model, std::string filePath)
//     : pqpModel(model), // Set the external PQP_Model
//       filePath(filePath)
// {
//     if (filePath.empty())
//     {
//         throw std::invalid_argument("filePath cannot be empty");
//     }
//     this->SetRotation(Matrix<PQP_REAL, 3, 3>::Identity()); // Initialized as identity matrix
//     this->SetTranslation(Matrix<PQP_REAL, 3, 1>::Zero());  // Initialized as zero vector
// }

PQP_REAL(*TrPQPModel::getR())
[3]
{
    return this->rotation;
}

PQP_REAL *
TrPQPModel::getT()
{
    return this->translation;
}

void TrPQPModel::Rotate(Matrix<double, 3, 3> rotation)
{
    this->R = rotation * this->R;
    Map<Matrix<PQP_REAL, 3, 3, RowMajor>>(this->rotation[0], 3, 3) = this->R;
}

void TrPQPModel::SetRotation(Matrix<double, 3, 3> rotation)
{
    this->R = rotation;
    Map<Matrix<PQP_REAL, 3, 3, RowMajor>>(this->rotation[0], 3, 3) = this->R;
}

void TrPQPModel::Translate(Vector3<double> translation)
{
    this->t = this->t + translation;
    Map<Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
}

void TrPQPModel::SetTranslation(Vector3<double> translation)
{
    this->t = translation;
    Map<Matrix<PQP_REAL, 3, 1>>(this->translation, 3) = this->t;
}

Vector3<PQP_REAL> TrPQPModel::GetGlobalPositionFromVector(Vector3<PQP_REAL> p) const
{
    return this->t + this->R * p;
}

Vector3<PQP_REAL> TrPQPModel::GetGlobalPositionFromPointer(PQP_REAL p[3]) const
{
    Map<Matrix<PQP_REAL, 3, 1>> pVector(p);
    return this->t + this->R * pVector;
}

void TrPQPModel::CheckDistanceStatic(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, TrPQPModel *m1, TrPQPModel *m2)
{
    PQP_Distance(result, m1->getR(), m1->getT(), m1->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get(), rel_err, abs_err);
}

void TrPQPModel::CollideStatic(PQP_CollideResult *result, TrPQPModel *m1, TrPQPModel *m2)
{
    PQP_Collide(result, m1->getR(), m1->getT(), m1->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get());
}

void TrPQPModel::CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, TrPQPModel *m2)
{
    PQP_Distance(result, this->getR(), this->getT(), this->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get(), rel_err, abs_err);
}

void TrPQPModel::Collide(PQP_CollideResult *result, TrPQPModel *m2)
{
    // std::cout << "Checkin collision at m1: " << Vector3d(this->getT()) << " m2: " << Vector3d(m2->getT()) << std::endl;
    PQP_Collide(result, this->getR(), this->getT(), this->pqpModel.get(), m2->getR(), m2->getT(), m2->pqpModel.get());
}
