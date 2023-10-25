#include <Eigen/Dense>
#include <memory>
#include "PQP.h"

#ifndef MODEL_H
#define MODEL_H

using namespace Eigen;

typedef Matrix<PQP_REAL, Dynamic, Dynamic, RowMajor> RowMatrixXi;

class TrPQPModel
{
public:
    // Copy constructor:
    // Constructors & Destructors
    // TrPQPModel(std::shared_ptr<PQP_Model> model = nullptr, std::string filePath = "");
    TrPQPModel(std::string filePath = "");
    ~TrPQPModel() = default;

    // Public member functions (e.g., setters and getters) can be added as needed

    // Public member variables
    Eigen::Matrix<PQP_REAL, 3, 3> R;     // Rotation matrix
    Eigen::Matrix<PQP_REAL, 3, 1> t;     // Translation vector
    std::shared_ptr<PQP_Model> pqpModel; // Pointer to PQP_Model which is collidable
    std::string filePath;

    PQP_REAL(*getR())
    [3];
    PQP_REAL *getT();
    void Rotate(Matrix<double, 3, 3> rotation);
    void Translate(Vector3<double> translation);

    void SetRotation(Matrix<double, 3, 3> rotation);
    void SetTranslation(Vector3<double> translation);

    Vector3<PQP_REAL> GetGlobalPositionFromVector(Vector3<PQP_REAL> p) const;
    Vector3<PQP_REAL> GetGlobalPositionFromPointer(PQP_REAL p[3]) const;

    /// @brief Intance-wise distance check
    void CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, TrPQPModel *m2);

    /// @brief Intance-wise collision check
    void Collide(PQP_CollideResult *result, TrPQPModel *m2);

    static void CheckDistanceStatic(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, TrPQPModel *m1, TrPQPModel *m2);
    static void CollideStatic(PQP_CollideResult *result, TrPQPModel *m1, TrPQPModel *m2);

private:
    PQP_REAL rotation[3][3];
    PQP_REAL translation[3];
};

#endif