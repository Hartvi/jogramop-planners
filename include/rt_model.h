
#ifndef RT_MODEL_H
#define RT_MODEL_H

#include <string>
#include <Eigen/Dense>
#include <memory>
#include "PQP.h"

namespace RtModels
{
    class RtModel
    {
    public:
        RtModel(std::string filePath = "");
        ~RtModel() = default;

        // Public member functions (e.g., setters and getters) can be added as needed

        // Public member variables
        Eigen::Matrix<PQP_REAL, 3, 3> R;     // Rotation matrix
        Eigen::Matrix<PQP_REAL, 3, 1> t;     // Translation vector
        std::shared_ptr<PQP_Model> pqpModel; // Pointer to PQP_Model which is collidable
        std::string filePath;

        PQP_REAL(*getR())
        [3];
        PQP_REAL *getT();
        void Rotate(Eigen::Matrix<double, 3, 3> rotation);
        void Translate(Eigen::Vector3d tr);

        void SetRotation(Eigen::Matrix<double, 3, 3> rotation);
        void SetTranslation(Eigen::Vector3d tr);

        /// @brief Intance-wise distance check
        void CheckDistance(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, RtModel *m2);

        /// @brief Intance-wise collision check
        void Collide(PQP_CollideResult *result, RtModel *m2);

        static void CheckDistanceStatic(PQP_DistanceResult *result, PQP_REAL rel_err, PQP_REAL abs_err, RtModel *m1, RtModel *m2);
        static void CollideStatic(PQP_CollideResult *result, RtModel *m1, RtModel *m2);

        std::string GetFilePath() const;
        std::string ToString();

        friend std::ostream &operator<<(std::ostream &os, RtModel &model);

    private:
        // std::string filePath;
        PQP_REAL rotation[3][3];
        PQP_REAL translation[3];
    };

}
#endif
