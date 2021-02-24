#ifndef PR_LIB__SINGULARITY_HPP_
#define PR_LIB__SINGULARITY_HPP_

#include <vector>
#include <array>
#include <cmath>

#include "eigen3/Eigen/Dense"

#include "pr_lib/pr_model.hpp"

namespace PRSingularity
{
    //*** Calculate Ang OTS ***//

    Eigen::Matrix<double,6,1> CalculateAngOts(
        const Eigen::Vector4d &X,
        const Eigen::Vector4d &Q,
        Eigen::Matrix<double, 5, 1> &X_OTS,
        Eigen::Matrix<double,6,4> &solOTS,
        Eigen::Matrix<double, 5, 1> &Xn_OTS,
        std::vector<double> &RParam,
        int iter_OTS, double tol_OTS);

    //*** Ang OTS Jacobian Equations ***//

    Eigen::Matrix<double, 5, 5> EqOTSJacobian(
        const double &wx, const double &wy, const double &wz, 
        const double &theta, const double &psi, 
        const Eigen::Vector4d &q, 
        const int &op, 
        const double &Rm1, const double &Rm2, const double &Rm3, 
        const double &betaMD, const double &betaMI
    );

    Eigen::Matrix<double,5,1> EqOTS(
       const double &wx, const double &wy, const double &wz, 
       const double &vx, const double &vz, 
       const double &theta, const double &psi, 
       const Eigen::Vector4d &q, 
       const int &op, 
       const double &Rm1, const double &Rm2, const double &Rm3, 
       const double &betaMD, const double &betaMI 
    );

    Eigen::Vector4d CalculateQindMod(
        const Eigen::Vector4d &X_cart, 
        const Eigen::Vector4d &q_ref, 
        const Eigen::Matrix<double,6,1> &angOTS, 
        const Eigen::Matrix<double,6,4> &solOTS
    );

}


#endif  // PR_LIB__SINGULARITY_HPP_