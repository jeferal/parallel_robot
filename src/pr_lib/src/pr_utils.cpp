#include "pr_lib/pr_utils.hpp"

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Dense"

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
	for (size_t i=0; i<vec.size(); i++){
		for (size_t j=0; j<vec[0].size(); j++){
			matrix(i,j) = vec[i][j];
		}
	}
}

void PRUtils::array2vector(const std::array<double, 4> &ar, std::vector<double> &vec)
{
	for(int i=0; i<ar.size(); i++)
		vec[i] = ar[i];
}
