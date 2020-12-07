#include <vector>
#include <array>
#include "eigen3/Eigen/Dense"

namespace PRModel
{
    void InverseKinematics(
        Eigen::Matrix<double, 4, 3> &Q, 
        const std::array<double, 4> &X, 
        const std::vector<double> &RParam);
}