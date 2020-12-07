#include <vector>
#include <array>

#include "eigen3/Eigen/Dense"

namespace PRModel
{
    //*** Inverse Kinematics ***//
    void InverseKinematics(
                            Eigen::Matrix<double, 4, 3> &Q, 
                            const std::array<double, 4> &X, 
                            const std::vector<double> &RParam);


    //*** Forward Kinematics ***//
    namespace FK
    {
        void EqPosition(
                                Eigen::Vector4d &f, 
                                const Eigen::RowVector4d &X, 
                                const std::array<double, 4UL> &QA, 
                                const std::vector<double> &RParam);



        void Jacobian(
                                Eigen::Matrix<double, 4, 4> &DJ, 
                                const Eigen::RowVector4d &X, 
                                const std::vector<double> &RParam);
    }


    std::array<double, 4UL>  ForwardKinematics(
                                std::array<double, 4UL> &QA, 
                                std::vector<double> &X_ant, 
                                const std::vector<double> &RParam, 
                                const double tol=0.0000007, 
                                const int iter_max=30);

}