#ifndef PR_LIB__UTILS_HPP_
#define PR_LIB__UTILS_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

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


    void Eigen2ArMsg(const Eigen::Vector4d &eig_vec, pr_msgs::msg::PRArrayH &ar_msg);

    void ArRMsg2Eigen(const pr_msgs::msg::PRArrayH::ConstPtr& ar_msg, Eigen::Vector4d &eig_vec);


    template <typename DerivedA>
    void Eigen2MatMsg(
        const Eigen::MatrixBase<DerivedA> &matrix,
        pr_msgs::msg::PRMatH &mat_msg)
    {
        //Return error if cols and rows don't match (msg and matrix)
        for(int i=0; i<matrix.rows(); i++)
        {
            for(int j=0; j<matrix.cols(); j++)
                mat_msg.data.push_back(matrix(i,j));
        }
        mat_msg.rows = matrix.rows();
        mat_msg.cols = matrix.cols();
    }

    template <typename DerivedA>
    void Eigen2MatMsgT(
        const Eigen::MatrixBase<DerivedA> &matrix,
        pr_msgs::msg::PRMatH &mat_msg)
    {
        //Return error if cols and rows don't match (msg and matrix)
        for(int i=0; i<matrix.cols(); i++)
        {
            for(int j=0; j<matrix.rows(); j++)
                mat_msg.data.push_back(matrix(j,i));
        }
        mat_msg.rows = matrix.cols();
        mat_msg.cols = matrix.rows();
    }


    template <typename DerivedA>
    void MatMsg2Eigen(
            const pr_msgs::msg::PRMatH::SharedPtr mat_msg,
            Eigen::MatrixBase<DerivedA> &matrix)
    {
        //Return error if cols and rows don't match (msg and matrix)
        for(int i=0; i<mat_msg->data.size(); i++)
        {
            int row = i/(matrix.rows()-1);
            int col = i%(matrix.cols());
            matrix.coeffRef(row,col) = mat_msg->data[i];
        }
    }


    template <typename DerivedA>
    void MatMsgR2Eigen(
            const pr_msgs::msg::PRMatH::ConstPtr& mat_msg,
            Eigen::MatrixBase<DerivedA> &matrix)
    {
        std::cout << "Calculando, size: " << mat_msg->data.size() << std::endl;
        std::cout << "Size msg: " << mat_msg->rows << ", " << mat_msg->cols << std::endl;
        std::cout << "Size matrix: " << matrix.rows() << ", " << matrix.cols() << std::endl;

        //Return error if cols and rows don't match (msg and matrix)
        for(int i=0; i<mat_msg->data.size(); i++)
        {
            int row = i/(matrix.rows()-1);
            int col = i%(matrix.cols());
            std::cout << "(" << row << ", " << col << ")" << std::endl;
            matrix.coeffRef(row,col) = mat_msg->data[i];
        }
    }

}

#endif