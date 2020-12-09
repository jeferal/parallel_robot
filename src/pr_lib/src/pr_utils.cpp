#include "pr_lib/pr_utils.hpp"

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"
#include "pr_msgs/msg/pr_mat_h.hpp"
#include "pr_msgs/msg/pr_array_h.hpp"

int PRUtils::read_file(Eigen::MatrixXd &ref_matrix, const std::string &file_path)
{
    std::vector<std::vector<double> > data;
	std::ifstream fileRead;
	std::string linea, tok;
	std::vector<double> line_data;

	fileRead.open(file_path.c_str());

	if (fileRead.is_open()){
		while (getline(fileRead, linea)) {
			std::stringstream iss(linea);
			line_data.clear();
			while (getline(iss, tok, ' ')) {
				line_data.push_back(strtod(tok.c_str(), NULL));
			}
			data.push_back(line_data);
		}
		fileRead.close();
		vector2matrix(ref_matrix, data);
        return 1;
	}
	else
        //Could not open file
        return -1;
}

void PRUtils::vector2matrix(
        Eigen::MatrixXd &matrix, 
        const std::vector<std::vector<double> > &vec)
{
    matrix.resize(vec.size(), vec[0].size());
	for (int i=0; i<(int)vec.size(); i++){
		for (int j=0; j<(int)vec[0].size(); j++){
			matrix(i,j) = vec[i][j];
		}
	}
}

void PRUtils::array2vector(const std::array<double, 4> &ar, std::vector<double> &vec)
{
	for(int i=0; i<(int)ar.size(); i++)
		vec[i] = ar[i];
}


//Generalizar función (Eigen2Matmsg)
void PRUtils::Eigen2Mat(
	const Eigen::MatrixXd &matrix, 
	pr_msgs::msg::PRMatH &mat_msg)
{
	for(int i=0; i<matrix.rows(); i++)
	{
		for(int j=0; j<matrix.cols(); j++)
			mat_msg.data.push_back(matrix(i,j));
	}

	mat_msg.rows = matrix.rows();
	mat_msg.cols = matrix.cols();
}


//Ver cómo generalizar más esta función (Matmsg2Eigen)
void PRUtils::Mat2Eigen__4_3(
	    const pr_msgs::msg::PRMatH::SharedPtr mat_msg,
	    Eigen::Matrix<double, 4, 3> &matrix)
{
	for(int i=0; i<(int)mat_msg->data.size(); i++)
		matrix(i%matrix.rows(), i%matrix.cols()) = mat_msg->data[i];

	mat_msg->rows = matrix.rows();
	mat_msg->cols = matrix.cols();
}