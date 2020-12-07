#include "pr_lib/pr_model.hpp"


void PRModel::InverseKinematics(
        Eigen::Matrix<double, 4, 3> &Q, 
        const std::array<double, 4> &X, 
        const std::vector<double> &RParam)
{
    double bb, dd, ff;
    //PATA 1
    //Articulación Prismatica
    Q(0,2) = sqrt(-2*  cos(X[2]) *  cos(X[3]) * RParam[0] * RParam[6] - 2* cos(X[2]) * cos(X[3]) * RParam[6] * X[0] + 2* cos(X[3]) * RParam[6] * sin(X[2]) * X[1] + pow(RParam[0],2) + 2* RParam[0] * X[0] + pow(RParam[6], 2) + pow(X[0], 2) + pow(X[1], 2));
    //Articulación Universal-Segunda J. Revolución
    bb=2 * cos(X[2]) * cos(X[3]) * RParam[0] * RParam[6] + 2 *  cos(X[2]) *  cos(X[3]) * RParam[6] * X[0] -  pow(cos(X[3]), 2) * pow(RParam[6], 2) - 2 *  cos(X[3]) * RParam[6] *  sin(X[2]) * X[1] - pow(RParam[0], 2) - 2 * RParam[0] * X[0] - pow(X[0], 2) - pow(X[1], 2);
    Q(0,1) = atan2(sqrt(-bb / pow(Q(0,2), 2)),  sin(X[3]) * RParam[6] / Q(0,2));
    //Articulación Universal-Primera J. Revolución
    Q(0,0) = atan2((X[1] + cos(X[3]) * RParam[6] * sin(X[2])) / sqrt(-bb), -(cos(X[2]) * cos(X[3]) * RParam[6] - RParam[0] - X[0]) / sqrt(-bb));
    //PATA 2
    //Articulación Prismatica
    Q(1,2) = sqrt(2 *  sin(RParam[9]) *  cos(RParam[4]) *  cos(X[2]) *  sin(X[3]) * RParam[1] * RParam[7] - 2 *  cos(RParam[9]) *  cos(X[3]) *  cos(RParam[4]) *  cos(X[2]) * RParam[1] * RParam[7] - 2 *  sin(RParam[9]) *  cos(X[3]) *  sin(RParam[4]) * RParam[1] * RParam[7] - 2 *  sin(RParam[9]) *  cos(X[2]) *  sin(X[3]) * RParam[7] * X[0] + 2 * RParam[7] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[9]) * X[1] + 2 *  cos(RParam[9]) *  cos(X[3]) *  cos(X[2]) * RParam[7] * X[0] - 2 *  cos(RParam[9]) *  cos(X[3]) * RParam[7] *  sin(X[2]) * X[1] - 2 *  cos(RParam[9]) *  sin(RParam[4]) *  sin(X[3]) * RParam[1] * RParam[7] - 2 * RParam[1] * X[0] *  cos(RParam[4]) + pow(RParam[1], 2) + pow(RParam[7], 2) + pow(X[0], 2) + pow(X[1], 2));
    //Articulación Universal-Segunda J. Revolución
    dd=2 *  cos(RParam[9]) *  cos(X[3]) *  cos(RParam[4]) *  cos(X[2]) * RParam[1] * RParam[7] - 2 *  sin(RParam[9]) *  cos(RParam[4]) *  cos(X[2]) *  sin(X[3]) * RParam[1] * RParam[7] + 2 *  cos(RParam[9]) *  cos(X[3]) * pow(RParam[7], 2) *  sin(X[3]) *  sin(RParam[9]) + 2 *  pow(cos(RParam[9]), 2) * pow(RParam[7], 2) *  pow(sin(X[3]), 2) - 2 *  cos(RParam[9]) *  cos(X[3]) *  cos(X[2]) * RParam[7] * X[0] + 2 *  cos(RParam[9]) *  cos(X[3]) * RParam[7] *  sin(X[2]) * X[1] + 2 *  sin(RParam[9]) *  cos(X[2]) *  sin(X[3]) * RParam[7] * X[0] - 2 * RParam[7] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[9]) * X[1] -  pow(cos(RParam[4]), 2) * pow(RParam[1], 2) -  pow(cos(RParam[9]), 2) * pow(RParam[7], 2) - pow(RParam[7], 2) *  pow(sin(X[3]), 2) + 2 * RParam[1] * X[0] *  cos(RParam[4]) - pow(X[0], 2) - pow(X[1], 2);
    Q(1,1) = atan2(sqrt(-dd / pow(Q(1,2), 2)), -( cos(X[3]) * RParam[7] *  sin(RParam[9]) +  sin(X[3]) * RParam[7] *  cos(RParam[9]) - RParam[1] *  sin(RParam[4])) / Q(1,2));
    //Articulación Universal-Primera J. Revolución
    Q(1,0) =atan2((X[1] -  cos(RParam[9]) *  cos(X[3]) * RParam[7] *  sin(X[2]) + RParam[7] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[9])) / sqrt(-dd), -(- cos(RParam[9]) *  cos(X[3]) *  cos(X[2]) * RParam[7] +  sin(RParam[9]) *  cos(X[2]) *  sin(X[3]) * RParam[7] + RParam[1] *  cos(RParam[4]) - X[0]) / sqrt(-dd));
            
    //PATA 3
    //Articulación Prismatica
    Q(2,2) = sqrt(-2 *  cos(RParam[10]) *  cos(RParam[5]) *  cos(X[3]) *  cos(X[2]) * RParam[2] * RParam[8] - 2 *  sin(RParam[10]) *  cos(RParam[5]) *  cos(X[2]) *  sin(X[3]) * RParam[2] * RParam[8] + 2 *  sin(RParam[5]) *  cos(RParam[10]) *  sin(X[3]) * RParam[2] * RParam[8] - 2 *  sin(RParam[5]) *  sin(RParam[10]) *  cos(X[3]) * RParam[2] * RParam[8] + 2 *  cos(RParam[10]) *  cos(X[3]) *  cos(X[2]) * RParam[8] * X[0] - 2 *  cos(RParam[10]) *  cos(X[3]) * RParam[8] *  sin(X[2]) * X[1] + 2 *  sin(RParam[10]) *  cos(X[2]) *  sin(X[3]) * RParam[8] * X[0] - 2 * RParam[8] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[10]) * X[1] - 2 * RParam[2] * X[0] *  cos(RParam[5]) + pow(RParam[2], 2) + pow(RParam[8], 2) + pow(X[0], 2) + pow(X[1], 2));
    //Articulación Universal-Segunda J. Revolución
    ff=2 *  pow(cos(RParam[10]), 2) * pow(RParam[8], 2) *  pow(sin(X[3]), 2) - 2 *  cos(RParam[10]) *  cos(X[3]) * pow(RParam[8], 2) *  sin(X[3]) *  sin(RParam[10]) + 2 *  sin(RParam[10]) *  cos(RParam[5]) *  cos(X[2]) *  sin(X[3]) * RParam[2] * RParam[8] + 2 *  cos(RParam[10]) *  cos(RParam[5]) *  cos(X[3]) *  cos(X[2]) * RParam[2] * RParam[8] - 2 *  sin(RParam[10]) *  cos(X[2]) *  sin(X[3]) * RParam[8] * X[0] + 2 * RParam[8] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[10]) * X[1] - 2 *  cos(RParam[10]) *  cos(X[3]) *  cos(X[2]) * RParam[8] * X[0] + 2 *  cos(RParam[10]) *  cos(X[3]) * RParam[8] *  sin(X[2]) * X[1] - pow(RParam[8], 2) *  pow(sin(X[3]), 2) -  pow(cos(RParam[10]), 2) * pow(RParam[8], 2) -  pow(cos(RParam[5]), 2) * pow(RParam[2], 2) + 2 * RParam[2] * X[0] *  cos(RParam[5]) - pow(X[0], 2) - pow(X[1], 2);
    Q(2,1) = atan2(sqrt(-ff / pow(Q(2,2), 2)), -(- cos(X[3]) * RParam[8] *  sin(RParam[10]) +  sin(X[3]) * RParam[8] *  cos(RParam[10]) + RParam[2] *  sin(RParam[5])) / Q(2,2));
    //Articulación Universal-Primera J. Revolución
    Q(2,0) = atan2(-( cos(RParam[10]) *  cos(X[3]) * RParam[8] *  sin(X[2]) + RParam[8] *  sin(X[2]) *  sin(X[3]) *  sin(RParam[10]) - X[1]) / sqrt(-ff), ( cos(RParam[10]) *  cos(X[3]) *  cos(X[2]) * RParam[8] +  sin(RParam[10]) *  cos(X[2]) *  sin(X[3]) * RParam[8] - RParam[2] *  cos(RParam[5]) + X[0]) / sqrt(-ff));
            
    //PATA 4
    //Articulación Prismatica
    Q(3,1) = sqrt(pow(RParam[3], 2) - 2 * RParam[3] * X[0] + pow(X[0], 2) + pow(X[1], 2));
    //Articulación de Revolución
    Q(3,0) = atan2((RParam[3] - X[0]) / Q(3,1), X[1] / Q(3,1));
}


void PRModel::FK::EqPosition(
        Eigen::Vector4d &f, 
        const Eigen::RowVector4d &X, 
        const std::array<double, 4UL> &QA, 
        const std::vector<double> &RParam)
{
    f << pow(QA[0], 2) + 2 * cos(X[2]) * cos(X[3]) * RParam[0] * RParam[6] + 2 * cos(X[2]) * cos(X[3]) * RParam[6] * X[0] - 2 * cos(X[3]) * RParam[6] * sin(X[2]) * X[1] - pow(RParam[0], 2) - 2 * RParam[0] * X[0] - pow(RParam[6], 2) - pow(X[0], 2) - pow(X[1], 2),
         pow(QA[1], 2) - 2 * sin(X[3]) * cos(X[2]) * cos(RParam[4]) * sin(RParam[9]) * RParam[1] * RParam[7] + 2 * cos(X[2]) * cos(RParam[4]) * cos(X[3]) * cos(RParam[9]) * RParam[1] * RParam[7] + 2 * sin(X[3]) * cos(X[2]) * sin(RParam[9]) * RParam[7] * X[0] + 2 * sin(X[3]) * sin(RParam[4]) * cos(RParam[9]) * RParam[1] * RParam[7] - 2 * RParam[7] * sin(X[2]) * sin(X[3]) * sin(RParam[9]) * X[1] - 2 * cos(X[2]) * cos(X[3]) * cos(RParam[9]) * RParam[7] * X[0] + 2 * sin(RParam[4]) * cos(X[3]) * sin(RParam[9]) * RParam[1] * RParam[7] + 2 * cos(RParam[9]) * cos(X[3]) * RParam[7] * sin(X[2]) * X[1] + 2 * RParam[1] * X[0] * cos(RParam[4]) - pow(RParam[1], 2) - pow(RParam[7], 2) - pow(X[0], 2) - pow(X[1], 2),
         pow(QA[2], 2) + 2 * cos(X[2]) * cos(RParam[5]) * sin(X[3]) * sin(RParam[10]) * RParam[2] * RParam[8] + 2 * cos(X[2]) * cos(RParam[5]) * cos(X[3]) * cos(RParam[10]) * RParam[2] * RParam[8] - 2 * sin(X[3]) * cos(X[2]) * sin(RParam[10]) * RParam[8] * X[0] - 2 * cos(X[2]) * cos(X[3]) * cos(RParam[10]) * RParam[8] * X[0] - 2 * sin(RParam[5]) * sin(X[3]) * cos(RParam[10]) * RParam[2] * RParam[8] + 2 * sin(RParam[5]) * cos(X[3]) * sin(RParam[10]) * RParam[2] * RParam[8] + 2 * RParam[8] * sin(X[2]) * sin(X[3]) * sin(RParam[10]) * X[1] + 2 * cos(RParam[10]) * cos(X[3]) * RParam[8] * sin(X[2]) * X[1] + 2 * RParam[2] * X[0] * cos(RParam[5]) - pow(RParam[2], 2) - pow(RParam[8], 2) - pow(X[0], 2) - pow(X[1], 2),
        -pow(RParam[3], 2) + 2 * RParam[3] * X[0] + pow(QA[3], 2) - pow(X[0], 2) - pow(X[1], 2);
}

void PRModel::FK::Jacobian(
        Eigen::Matrix<double, 4, 4> &DJ, 
        const Eigen::RowVector4d &X, 
        const std::vector<double> &RParam)
{
    DJ << 2 *  cos(X[2]) *  cos(X[3]) * RParam[6] - (2 * RParam[0]) - (2 * X[0]),     -2 *  sin(X[2]) *  cos(X[3]) * RParam[6] - (2 * X[1]),     -2 *  cos(X[3]) * RParam[6] * ( cos(X[2]) * X[1] +  sin(X[2]) * RParam[0] +  sin(X[2]) * X[0]),      -2 *  sin(X[3]) * RParam[6] * ( cos(X[2]) * RParam[0] +  cos(X[2]) * X[0] -  sin(X[2]) * X[1]),
          2 *  cos(X[2]) *  sin(X[3]) * RParam[7] *  sin(RParam[9]) - 2 *  cos(X[2]) *  cos(X[3]) * RParam[7] *  cos(RParam[9]) + 2 * RParam[1] *  cos(RParam[4]) - (2 * X[0]),        -2 *  sin(X[2]) *  sin(X[3]) * RParam[7] *  sin(RParam[9]) + 2 *  sin(X[2]) *  cos(X[3]) * RParam[7] *  cos(RParam[9]) - (2 * X[1]),      -2 * RParam[7] * (- sin(X[2]) *  sin(X[3]) *  cos(RParam[4]) *  sin(RParam[9]) * RParam[1] +  sin(X[2]) *  cos(X[3]) *  cos(RParam[4]) *  cos(RParam[9]) * RParam[1] +  cos(X[2]) *  sin(X[3]) *  sin(RParam[9]) * X[1] -  cos(X[2]) *  cos(X[3]) *  cos(RParam[9]) * X[1] +  sin(X[2]) *  sin(X[3]) *  sin(RParam[9]) * X[0] -  sin(X[2]) *  cos(X[3]) *  cos(RParam[9]) * X[0]),       -2 * RParam[7] * ( cos(X[2]) *  sin(X[3]) *  cos(RParam[4]) *  cos(RParam[9]) * RParam[1] +  cos(X[2]) *  cos(X[3]) *  cos(RParam[4]) *  sin(RParam[9]) * RParam[1] -  cos(X[2]) *  sin(X[3]) *  cos(RParam[9]) * X[0] -  cos(X[2]) *  cos(X[3]) *  sin(RParam[9]) * X[0] +  sin(X[2]) *  sin(X[3]) *  cos(RParam[9]) * X[1] +  sin(X[2]) *  cos(X[3]) *  sin(RParam[9]) * X[1] +  sin(X[3]) *  sin(RParam[9]) *  sin(RParam[4]) * RParam[1] -  cos(X[3]) *  cos(RParam[9]) *  sin(RParam[4]) * RParam[1]),
         -2 *  cos(X[2]) *  sin(X[3]) * RParam[8] *  sin(RParam[10]) - 2 *  cos(X[2]) *  cos(X[3]) * RParam[8] *  cos(RParam[10]) + 2 * RParam[2] *  cos(RParam[5]) - (2 * X[0]),       2 *  sin(X[2]) *  sin(X[3]) * RParam[8] *  sin(RParam[10]) + 2 *  sin(X[2]) *  cos(X[3]) * RParam[8] *  cos(RParam[10]) - (2 * X[1]),       2 * RParam[8] * (- sin(X[2]) *  sin(X[3]) *  sin(RParam[10]) *  cos(RParam[5]) * RParam[2] -  sin(X[2]) *  cos(X[3]) *  cos(RParam[5]) *  cos(RParam[10]) * RParam[2] +  cos(X[2]) *  sin(X[3]) *  sin(RParam[10]) * X[1] +  cos(X[2]) *  cos(X[3]) *  cos(RParam[10]) * X[1] +  sin(X[2]) *  sin(X[3]) *  sin(RParam[10]) * X[0] +  sin(X[2]) *  cos(X[3]) *  cos(RParam[10]) * X[0]),        -2 * RParam[8] * ( cos(X[2]) *  sin(X[3]) *  cos(RParam[5]) *  cos(RParam[10]) * RParam[2] -  cos(X[2]) *  cos(X[3]) *  sin(RParam[10]) *  cos(RParam[5]) * RParam[2] -  cos(X[2]) *  sin(X[3]) *  cos(RParam[10]) * X[0] +  cos(X[2]) *  cos(X[3]) *  sin(RParam[10]) * X[0] +  sin(X[2]) *  sin(X[3]) *  cos(RParam[10]) * X[1] -  sin(X[2]) *  cos(X[3]) *  sin(RParam[10]) * X[1] +  sin(X[3]) *  sin(RParam[10]) *  sin(RParam[5]) * RParam[2] +  cos(X[3]) *  cos(RParam[10]) *  sin(RParam[5]) * RParam[2]),
          2 * RParam[3] - 2 * X[0], -2 * X[1], 0, 0;
}


std::array<double, 4UL>  PRModel::ForwardKinematics(
        std::array<double, 4UL> &QA, 
        std::vector<double> &X_ant, 
        const std::vector<double> &RParam, 
        const double tol, 
        const int iter_max)
{
    Eigen::Vector4d f;
    Eigen::RowVector4d Xn, X;
    Eigen::Matrix<double, 4, 4> DJ;

    for(int i=0; i<4; i++){
        X(i) = X_ant[i];
    }

    FK::EqPosition(f, X, QA, RParam);
    double error = f.norm();
    int i=0;

    while(error > tol){
        FK::EqPosition(f, X, QA, RParam);
        error = f.norm();
        FK::Jacobian(DJ, X, RParam);

        Xn = X.transpose() - DJ.inverse()*f;

        X = Xn;
        i++;
        if (i>iter_max)
            break;
    }
            
    return {X[0], X[1], X[2], X[3]};
}