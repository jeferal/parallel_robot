#ifndef PR_LIB__UTILS_HPP_
#define PR_LIB__UTILS_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"


namespace PRUtils
{
    int read_file(
        Eigen::MatrixXd &ref_matrix, 
        const std::string &file_path);

    void vector2matrix(
        Eigen::MatrixXd &matrix, 
        const std::vector<std::vector<double> > &vec);


    //Usar template functions!!
    void array2vector(const std::array<double, 4> &ar, std::vector<double> &vec);

    void Eigen2Mat(
        const Eigen::MatrixXd &matrix, 
        pr_msgs::msg::PRMatH &vec);

    void Mat2Eigen__11_4(
	    const pr_msgs::msg::PRMatH::SharedPtr vec,
	    Eigen::Matrix<double, 11, 4> &matrix);

}

#endif