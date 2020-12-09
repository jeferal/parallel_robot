#ifndef PR_LIB__UTILS_HPP_
#define PR_LIB__UTILS_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"


namespace PRUtils
{
    int read_file(Eigen::MatrixXd &ref_matrix, const std::string &file_path);

    void vector2matrix(
        Eigen::MatrixXd &matrix, 
        const std::vector<std::vector<double> > &vec);

    void array2vector(const std::array<double, 4> &ar, std::vector<double> &vec);

}

#endif