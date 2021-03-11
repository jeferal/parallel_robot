#ifndef PR_LIB__LIMITS_HPP_
#define PR_LIB__LIMITS_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"


namespace PRLimits {

    Eigen::Matrix<double,4,2> LimActuators(void);

    Eigen::Vector4d LimAngles(void);
}


#endif