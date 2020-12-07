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
        const std::vector<std::vector<double>> &vec);

}