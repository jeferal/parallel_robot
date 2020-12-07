#ifndef PR_LIB__MODEL_HPP_
#define PR_LIB__MODEL_HPP_

#include <vector>
#include <array>
#include <cmath>

#include "eigen3/Eigen/Dense"

namespace PRModel
{
    //*** Inverse Kinematics ***//
    void InverseKinematics(
                            Eigen::Matrix<double, 4, 3> &Q, 
                            const std::array<double, 4> &X, 
                            const std::vector<double> &RParam);
    
    void InverseKinematicsPrism(
                            Eigen::RowVector4d &Q, 
                            Eigen::RowVector4d &X, 
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
                                const double tol, 
                                const int iter_max);


    //*** Dependent Jacobian ***//
    void DepJacobian(
                                Eigen::Matrix<double, 11, 11> &DepJ, 
                                const Eigen::Matrix<double, 4, 3> &Q, 
                                const double &theta, 
                                const double &psi, 
                                const std::vector<double> &RParam);

    //*** Independent Jacobian ***//
    void IndJacobian(
                                Eigen::Matrix<double, 11, 4> &IndJ, 
                                const Eigen::Matrix<double, 4, 3> &Q);

    //*** RastT matrix ***//
    void RastT(
                                Eigen::Matrix<double, 15, 4> &RastT , 
                                const Eigen::Matrix<double, 11,11> &DepJ, 
                                const Eigen::Matrix<double, 11, 4> &IndJ);

    //*** Gravitational terms ***//
    void QGravFunction(
                                Eigen::Matrix<double, 4, 1> &Qgrav, 
                                const Eigen::Matrix<double, 4, 15> &RastT, 
                                const double &theta, double &psi, 
                                const Eigen::Matrix<double, 4, 3> &Q, 
                                const std::vector<double> &P11, 
                                const std::vector<double> &P12, 
                                const std::vector<double> &P21, 
                                const std::vector<double> &P22, 
                                const std::vector<double> &P31, 
                                const std::vector<double> &P32, 
                                const std::vector<double> &P41, 
                                const std::vector<double> &P42, 
                                const std::vector<double> &Pm);

    //*** Pose from optitrack markers ***//
    namespace OptiTrack
    {
        void PosOriPM(
                                Eigen::Matrix<double, 3, 2> &Coordinates,
                                const Eigen::Vector3d &mf1,
                                const Eigen::Vector3d &mf2,
                                const Eigen::Vector3d &mf3,
                                const Eigen::Vector3d &mm1,
                                const Eigen::Vector3d &mm2,
                                const Eigen::Vector3d &mm3);
    }

}

#endif