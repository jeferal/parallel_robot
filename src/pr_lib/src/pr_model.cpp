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

void PRModel::InverseKinematicsPrism(
                            Eigen::RowVector4d &Q, 
                            Eigen::RowVector4d &X, 
                            const std::vector<double> &RParam)
{
    //JOINT 1
    Q(0) = sqrt(-2*  cos(X(2)) *  cos(X(3)) * RParam[0] * RParam[6] - 2* cos(X(2)) * cos(X(3)) * RParam[6] * X(0) + 2* cos(X(3)) * RParam[6] * sin(X(2)) * X(1) + pow(RParam[0],2) + 2* RParam[0] * X(0) + pow(RParam[6], 2) + pow(X(0), 2) + pow(X(1), 2));
    //JOINT 2
    Q(1) = sqrt(2 *  sin(RParam[9]) *  cos(RParam[4]) *  cos(X(2)) *  sin(X(3)) * RParam[1] * RParam[7] - 2 *  cos(RParam[9]) *  cos(X(3)) *  cos(RParam[4]) *  cos(X(2)) * RParam[1] * RParam[7] - 2 *  sin(RParam[9]) *  cos(X(3)) *  sin(RParam[4]) * RParam[1] * RParam[7] - 2 *  sin(RParam[9]) *  cos(X(2)) *  sin(X(3)) * RParam[7] * X(0) + 2 * RParam[7] *  sin(X(2)) *  sin(X(3)) *  sin(RParam[9]) * X(1) + 2 *  cos(RParam[9]) *  cos(X(3)) *  cos(X(2)) * RParam[7] * X(0) - 2 *  cos(RParam[9]) *  cos(X(3)) * RParam[7] *  sin(X(2)) * X(1) - 2 *  cos(RParam[9]) *  sin(RParam[4]) *  sin(X(3)) * RParam[1] * RParam[7] - 2 * RParam[1] * X(0) *  cos(RParam[4]) + pow(RParam[1], 2) + pow(RParam[7], 2) + pow(X(0), 2) + pow(X(1), 2));
    //JOINT 3
    Q(2) = sqrt(-2 *  cos(RParam[10]) *  cos(RParam[5]) *  cos(X(3)) *  cos(X(2)) * RParam[2] * RParam[8] - 2 *  sin(RParam[10]) *  cos(RParam[5]) *  cos(X(2)) *  sin(X(3)) * RParam[2] * RParam[8] + 2 *  sin(RParam[5]) *  cos(RParam[10]) *  sin(X(3)) * RParam[2] * RParam[8] - 2 *  sin(RParam[5]) *  sin(RParam[10]) *  cos(X(3)) * RParam[2] * RParam[8] + 2 *  cos(RParam[10]) *  cos(X(3)) *  cos(X(2)) * RParam[8] * X(0) - 2 *  cos(RParam[10]) *  cos(X(3)) * RParam[8] *  sin(X(2)) * X(1) + 2 *  sin(RParam[10]) *  cos(X(2)) *  sin(X(3)) * RParam[8] * X(0) - 2 * RParam[8] *  sin(X(2)) *  sin(X(3)) *  sin(RParam[10]) * X(1) - 2 * RParam[2] * X(0) *  cos(RParam[5]) + pow(RParam[2], 2) + pow(RParam[8], 2) + pow(X(0), 2) + pow(X(1), 2));
    //JOINT 4
    Q(3) = sqrt(pow(RParam[3], 2) - 2 * RParam[3] * X(0) + pow(X(0), 2) + pow(X(1), 2));
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

void PRModel::DepJacobian(
        Eigen::Matrix<double, 11, 11> &DepJ, 
        const Eigen::Matrix<double, 4, 3> &Q, 
        const double &theta, 
        const double &psi, 
        const std::vector<double> &RParam)
{
    DepJ(0,0)=-sin(Q(0,0))*sin(Q(0,1))*Q(0,2);
    DepJ(0,1)=cos(Q(0,1))*cos(Q(0,0))*Q(0,2);
    DepJ(0,7)=-1;
    DepJ(0,9)=-sin(theta)*cos(psi)*RParam[6];
    DepJ(0,10)=-cos(theta)*sin(psi)*RParam[6];
            
    DepJ(1,1)=sin(Q(0,1))*Q(0,2);
    DepJ(1,10)=cos(psi)*RParam[6];
            
    DepJ(2,0)=cos(Q(0,0))*sin(Q(0,1))*Q(0,2);
    DepJ(2,1)=sin(Q(0,0))*cos(Q(0,1))*Q(0,2);
    DepJ(2,8)=-1;
    DepJ(2,9)=-cos(theta)*cos(psi)*RParam[6];
    DepJ(2,10)=sin(theta)*sin(psi)*RParam[6];
            
    DepJ(3,2)=-sin(Q(1,0))*sin(Q(1,1))*Q(1,2);
    DepJ(3,3)=cos(Q(1,0))*cos(Q(1,1))*Q(1,2);
    DepJ(3,7)=-1;
    DepJ(3,9)=sin(theta)*cos(psi)*RParam[7]*cos(RParam[9])-sin(theta)*sin(psi)*RParam[7]*sin(RParam[9]);
    DepJ(3,10)=cos(theta)*sin(psi)*RParam[7]*cos(RParam[9])+cos(theta)*cos(psi)*RParam[7]*sin(RParam[9]);
            
    DepJ(4,3)=sin(Q(1,1))*Q(1,2);
    DepJ(4,10)=-cos(psi)*RParam[7]*cos(RParam[9])+sin(psi)*RParam[7]*sin(RParam[9]);
            
    DepJ(5,2)=cos(Q(1,0))*sin(Q(1,1))*Q(1,2);
    DepJ(5,3)=sin(Q(1,0))*cos(Q(1,1))*Q(1,2);
    DepJ(5,8)=-1;
    DepJ(5,9)=cos(theta)*cos(psi)*RParam[7]*cos(RParam[9])-cos(theta)*sin(psi)*RParam[7]*sin(RParam[9]);
    DepJ(5,10)=-sin(theta)*sin(psi)*RParam[7]*cos(RParam[9])-sin(theta)*cos(psi)*RParam[7]*sin(RParam[9]);
            
    DepJ(6,4)=-sin(Q(2,0))*sin(Q(2,1))*Q(2,2);
    DepJ(6,5)=cos(Q(2,0))*cos(Q(2,1))*Q(2,2);
    DepJ(6,7)=-1;
    DepJ(6,9)=sin(theta)*cos(psi)*RParam[7]*cos(RParam[10])+sin(theta)*sin(psi)*RParam[7]*sin(RParam[10]);
    DepJ(6,10)=cos(theta)*sin(psi)*RParam[7]*cos(RParam[10])-cos(theta)*cos(psi)*RParam[7]*sin(RParam[10]);
            
    DepJ(7,5)=sin(Q(2,1))*Q(2,2);
    DepJ(7,10)=-cos(psi)*RParam[7]*cos(RParam[10])-sin(psi)*RParam[7]*sin(RParam[10]);
            
    DepJ(8,4)=cos(Q(2,0))*sin(Q(2,1))*Q(2,2);
    DepJ(8,5)=sin(Q(2,0))*cos(Q(2,1))*Q(2,2);
    DepJ(8,8)=-1;
    DepJ(8,9)=cos(theta)*cos(psi)*RParam[7]*cos(RParam[10])+cos(theta)*sin(psi)*RParam[7]*sin(RParam[10]);
    DepJ(8,10)=-sin(theta)*sin(psi)*RParam[7]*cos(RParam[10])+sin(theta)*cos(psi)*RParam[7]*sin(RParam[10]);
            
    DepJ(9,6)=-cos(Q(3,0))*Q(3,1);
    DepJ(9,7)=-1;
            
    DepJ(10,6)=-sin(Q(3,0))*Q(3,1);
    DepJ(10,8)=-1;
}

void PRModel::IndJacobian(
        Eigen::Matrix<double, 11, 4> &IndJ, 
        const Eigen::Matrix<double, 4, 3> &Q)
{
    IndJ = Eigen::Matrix<double, 11, 4>::Zero();
    //Ecuaciones para la matriz del jacobiano independiente
    IndJ(0,0)=cos(Q(0,0))*sin(Q(0,1));
    IndJ(1,0)=-cos(Q(0,1));
    IndJ(2,0)=sin(Q(0,0))*sin(Q(0,1));
    IndJ(3,1)=cos(Q(1,0))*sin(Q(1,1));
    IndJ(4,1)=-cos(Q(1,1));
    IndJ(5,1)=sin(Q(1,0))*sin(Q(1,1));
    IndJ(6,2)=cos(Q(2,0))*sin(Q(2,1));
    IndJ(7,2)=-cos(Q(2,1));
    IndJ(8,2)=sin(Q(2,0))*sin(Q(2,1));
    IndJ(9,3)=-sin(Q(3,0));
    IndJ(10,3)=cos(Q(3,0));    
}

void PRModel::Rast(
        Eigen::Matrix<double, 15, 4> &Rast , 
        const Eigen::Matrix<double, 11,11> &DepJ, 
        const Eigen::Matrix<double, 11, 4> &IndJ)
{
    Rast.setZero();
    Rast.block(0, 0, 11, 4) =  -DepJ.inverse() * IndJ;
    Rast.block(11, 0, 4, 4) = Eigen::Matrix<double, 4, 4>::Identity();
}


void PRModel::QGravFunction(
                                Eigen::Matrix<double, 4, 1> &Qgrav, 
                                const Eigen::Matrix<double, 4, 15> &RastT, 
                                const double &theta, const double &psi, 
                                const Eigen::Matrix<double, 4, 3> &Q, 
                                const std::vector<double> &P11, 
                                const std::vector<double> &P12, 
                                const std::vector<double> &P21, 
                                const std::vector<double> &P22, 
                                const std::vector<double> &P31, 
                                const std::vector<double> &P32, 
                                const std::vector<double> &P41, 
                                const std::vector<double> &P42, 
                                const std::vector<double> &Pm)
{
    Eigen::Matrix<double, 15, 1> VQgrav;
    double g=9.81;

    VQgrav(0)= g * (cos(Q(0,1)) * cos(Q(0,0)) * P11[0] * P11[1] + cos(Q(0,1)) * cos(Q(0,0)) * P12[0] * P12[1] + cos(Q(0,0)) * sin(Q(0,1)) * P11[0] * P11[3] + cos(Q(0,0)) * sin(Q(0,1)) * P12[0] * Q(0,2) + cos(Q(0,0)) * sin(Q(0,1)) * P12[0] * P12[3] - sin(Q(0,0)) * P11[0] * P11[2] - sin(Q(0,0)) * P12[0] * P12[2]);
    VQgrav(1)= g * sin(Q(0,0)) * (cos(Q(0,1)) * P11[0] * P11[3] + cos(Q(0,1)) * P12[0] * Q(0,2) + cos(Q(0,1)) * P12[0] * P12[3] - sin(Q(0,1)) * P11[0] * P11[1] - sin(Q(0,1)) * P12[0] * P12[1]);
    VQgrav(2)= g * (cos(Q(1,1)) * cos(Q(1,0)) * P21[0] * P21[1] + cos(Q(1,1)) * cos(Q(1,0)) * P22[0] * P22[1] + cos(Q(1,0)) * sin(Q(1,1)) * P21[0] * P21[3] + cos(Q(1,0)) * sin(Q(1,1)) * P22[0] * Q(1,2) + cos(Q(1,0)) * sin(Q(1,1)) * P22[0] * P22[3] - sin(Q(1,0)) * P21[0] * P21[2] - sin(Q(1,0)) * P22[0] * P22[2]);
    VQgrav(3)= g * sin(Q(1,0)) * (cos(Q(1,1)) * P21[0] * P21[3] + cos(Q(1,1)) * P22[0] * Q(1,2) + cos(Q(1,1)) * P22[0] * P22[3] - sin(Q(1,1)) * P21[0] * P21[1] - sin(Q(1,1)) * P22[0] * P22[1]);
    VQgrav(4)= g * (cos(Q(2,1)) * cos(Q(2,0)) * P31[0] * P31[1] + cos(Q(2,1)) * cos(Q(2,0)) * P32[0] * P32[1] + cos(Q(2,0)) * sin(Q(2,1)) * P31[0] * P31[3] + cos(Q(2,0)) * sin(Q(2,1)) * P32[0] * Q(2,2) + cos(Q(2,0)) * sin(Q(2,1)) * P32[0] * P32[3] - sin(Q(2,0)) * P31[0] * P31[2] - sin(Q(2,0)) * P32[0] * P32[2]);
    VQgrav(5)= g * sin(Q(2,0)) * (cos(Q(2,1)) * P31[0] * P31[3] + cos(Q(2,1)) * P32[0] * Q(2,2) + cos(Q(2,1)) * P32[0] * P32[3] - sin(Q(2,1)) * P31[0] * P31[1] - sin(Q(2,1)) * P32[0] * P32[1]);
    VQgrav(6)= -g * (sin(Q(3,0)) * P41[0] * P41[3] + sin(Q(3,0)) * P42[0] * Q(3,1) - sin(Q(3,0)) * P42[0] * P42[2] + Pm[0] * sin(Q(3,0)) * Q(3,1) - cos(Q(3,0)) * P41[0] * P41[1] + cos(Q(3,0)) * P42[0] * P42[1]);
    VQgrav(7)=0;
    VQgrav(8)=0;
    VQgrav(9)= Pm[0] * g * (cos(theta) * sin(psi) * Pm[2] - cos(theta) * cos(psi) * Pm[1] - sin(theta) * Pm[3]);
    VQgrav(10)= Pm[0] * g * sin(theta) * (sin(psi) * Pm[1] + cos(psi) * Pm[2]);
    VQgrav(11)= P12[0] * g * sin(Q(0,0)) * sin(Q(0,1));
    VQgrav(12)= P22[0] * g * sin(Q(1,0)) * sin(Q(1,1));
    VQgrav(13)= P32[0] * g * sin(Q(2,0)) * sin(Q(2,1));
    VQgrav(14)= cos(Q(3,0)) * g * (P42[0] + Pm[0]);

    Qgrav = (RastT) * VQgrav; 
}


void PRModel::OptiTrack::PosOriPM(
        Eigen::Matrix<double, 3, 2> &Coordinates,
        const Eigen::Vector3d &mf1,
        const Eigen::Vector3d &mf2,
        const Eigen::Vector3d &mf3,
        const Eigen::Vector3d &mm1,
        const Eigen::Vector3d &mm2,
        const Eigen::Vector3d &mm3,
        const bool &robot_5p)
{
        //Sistema de referencia fijo
        
        //Matriz de rotación del sistema fijo al sistema local
        Eigen::Matrix<double, 3, 3> Rlf;
        //eje xf
        Eigen::Vector3d rmf1mf2 = mf2-mf1;
        Rlf.col(0) = rmf1mf2/sqrt(rmf1mf2.transpose()*rmf1mf2);
        //eje yf
        Eigen::Vector3d rmf1mf3 = mf3-mf1;
        Rlf.col(1) = rmf1mf3/sqrt(rmf1mf3.transpose()*rmf1mf3);
        //eje zf
        Eigen::Vector3d rzf_l = rmf1mf2.cross(rmf1mf3);
        Rlf.col(2) = rzf_l/sqrt(rzf_l.transpose()*rzf_l);
        
        //Matriz de rotación del sistema local al sistema fijo
        Eigen::Matrix<double, 3, 3> Rfl = Rlf.transpose();

        //Sistema de referencia móvil
        
        //Matriz de rotación del sistema movil al sistema local
        Eigen::Matrix<double, 3, 3> Rlm;
        //eje xm
        Eigen::Vector3d rmm1mm2_l = mm2 - mm1;
        if(robot_5p)
            rmm1mm2_l = -rmm1mm2_l;
        
        Rlm.col(0) = rmm1mm2_l/sqrt(rmm1mm2_l.transpose()*rmm1mm2_l);
        //eje ym
        Eigen::Vector3d rmm1mm3_l = mm3 - mm1;
        Rlm.col(1) = rmm1mm3_l/sqrt(rmm1mm3_l.transpose()*rmm1mm3_l);
        //eje zm
        Eigen::Vector3d rzm_l = rmm1mm2_l.cross(rmm1mm3_l);
        Rlm.col(2) = rzm_l/sqrt(rzm_l.transpose()*rzm_l);
        
        //Matriz de rotación entre el sistema móvil y el sistema fijo
        Eigen::Matrix<double, 3, 3> Rfm = Rfl*Rlm;

        //Orientación de la plataforma móvil
        //Orientación de la plataforma móvil
        Coordinates(1,1) = atan2(Rfm(0,2), Rfm(2,2));
        Coordinates(2,1) = atan2(Rfm(1,0), Rfm(1,1));

        //Posición de la plataforma móvil
        Eigen::Vector3d rdf_f;
        Eigen::Vector3d rdm_m;
        if(robot_5p){
            rdf_f << -0.350, 0, 0;
            rdm_m << -0.320, 0.150, -0.098;
        } else {
            rdf_f << -0.276, 0.064, 0.018;
            rdm_m << 0,0,-0.1605;
        }
        
        Coordinates.col(0) = Rfl*(mm1 + Rlm*rdm_m - (mf1 + Rlf*rdf_f));
}

void PRModel::ForwardJacobian(Eigen::Matrix<double,4,4> &ForJ, const std::array<double,4> &X, const std::vector<double> &Rparam){
	
	double x_m = X[0], z_m = X[1], theta = X[2], psi = X[3];

	ForJ(0,0) = -(cos(theta) * cos(psi) * Rparam[6] - Rparam[0] - x_m) * pow((-2 * cos(theta) * cos(psi) * Rparam[0] * Rparam[6] - 2 * cos(theta) * cos(psi) * Rparam[6] * x_m + 2 * cos(psi) * Rparam[6] * sin(theta) * z_m + pow(Rparam[0],2) + 2 * Rparam[0] * x_m + pow(Rparam[6],2) + pow(x_m,2) + pow(z_m,2)), (-1.0 / 2.0));

	ForJ(0,1) = (z_m + sin(theta) * cos(psi) * Rparam[6]) * pow((-2 * cos(theta) * cos(psi) * Rparam[0] * Rparam[6] - 2 * cos(theta) * cos(psi) * Rparam[6] * x_m + 2 * cos(psi) * Rparam[6] * sin(theta) * z_m + pow(Rparam[0], 2) + 2 * Rparam[0] * x_m + pow(Rparam[6], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(0,2) = cos(psi) * Rparam[6] * (cos(theta) * z_m + sin(theta) * Rparam[0] + sin(theta) * x_m) * pow((-2 * cos(theta) * cos(psi) * Rparam[0] * Rparam[6] - 2 * cos(theta) * cos(psi) * Rparam[6] * x_m + 2 * cos(psi) * Rparam[6] * sin(theta) * z_m + pow(Rparam[0], 2) + 2 * Rparam[0] * x_m + pow(Rparam[6], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(0,3) = sin(psi) * Rparam[6] * (cos(theta) * Rparam[0] + cos(theta) * x_m - sin(theta) * z_m) * pow((-2 * cos(theta) * cos(psi) * Rparam[0] * Rparam[6] - 2 * cos(theta) * cos(psi) * Rparam[6] * x_m + 2 * cos(psi) * Rparam[6] * sin(theta) * z_m + pow(Rparam[0], 2) + 2 * Rparam[0] * x_m + pow(Rparam[6], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0)); 

	ForJ(1,0) = -(-cos(theta) * cos(psi) * Rparam[7] * cos(Rparam[9]) + cos(theta) * sin(psi) * Rparam[7] * sin(Rparam[9]) + Rparam[1] * cos(Rparam[4]) - x_m) * pow((2 * cos(theta) * sin(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * cos(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * sin(psi) * sin(Rparam[9]) * Rparam[7] * x_m + 2 * cos(theta) * cos(psi) * cos(Rparam[9]) * Rparam[7] * x_m - 2 * sin(psi) * sin(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] + 2 * Rparam[7] * sin(theta) * sin(psi) * sin(Rparam[9]) * z_m - 2 * cos(psi) * sin(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(psi) * cos(Rparam[9]) * Rparam[7] * sin(theta) * z_m - 2 * Rparam[1] * x_m * cos(Rparam[4]) + pow(Rparam[1], 2) + pow(Rparam[7], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(1,1) = (z_m - sin(theta) * cos(psi) * Rparam[7] * cos(Rparam[9]) + sin(theta) * sin(psi) * Rparam[7] * sin(Rparam[9])) * pow((2 * cos(theta) * sin(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * cos(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * sin(psi) * sin(Rparam[9]) * Rparam[7] * x_m + 2 * cos(theta) * cos(psi) * cos(Rparam[9]) * Rparam[7] * x_m - 2 * sin(psi) * sin(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] + 2 * Rparam[7] * sin(theta) * sin(psi) * sin(Rparam[9]) * z_m - 2 * cos(psi) * sin(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(psi) * cos(Rparam[9]) * Rparam[7] * sin(theta) * z_m - 2 * Rparam[1] * x_m * cos(Rparam[4]) + pow(Rparam[1], 2) + pow(Rparam[7], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(1,2) = Rparam[7] * (-sin(theta) * sin(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] + sin(theta) * cos(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] + cos(theta) * sin(psi) * sin(Rparam[9]) * z_m - cos(theta) * cos(psi) * cos(Rparam[9]) * z_m + sin(theta) * sin(psi) * sin(Rparam[9]) * x_m - sin(theta) * cos(psi) * cos(Rparam[9]) * x_m) * pow((2 * cos(theta) * sin(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * cos(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * sin(psi) * sin(Rparam[9]) * Rparam[7] * x_m + 2 * cos(theta) * cos(psi) * cos(Rparam[9]) * Rparam[7] * x_m - 2 * sin(psi) * sin(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] + 2 * Rparam[7] * sin(theta) * sin(psi) * sin(Rparam[9]) * z_m - 2 * cos(psi) * sin(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(psi) * cos(Rparam[9]) * Rparam[7] * sin(theta) * z_m - 2 * Rparam[1] * x_m * cos(Rparam[4]) + pow(Rparam[1], 2) + pow(Rparam[7], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(1,3) = Rparam[7] * (cos(theta) * sin(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] + cos(theta) * cos(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] - cos(theta) * sin(psi) * cos(Rparam[9]) * x_m - cos(theta) * cos(psi) * sin(Rparam[9]) * x_m + sin(theta) * sin(psi) * cos(Rparam[9]) * z_m + sin(theta) * cos(psi) * sin(Rparam[9]) * z_m + sin(psi) * sin(Rparam[9]) * sin(Rparam[4]) * Rparam[1] - cos(psi) * cos(Rparam[9]) * sin(Rparam[4]) * Rparam[1]) * pow((2 * cos(theta) * sin(psi) * cos(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * cos(psi) * cos(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(theta) * sin(psi) * sin(Rparam[9]) * Rparam[7] * x_m + 2 * cos(theta) * cos(psi) * cos(Rparam[9]) * Rparam[7] * x_m - 2 * sin(psi) * sin(Rparam[4]) * cos(Rparam[9]) * Rparam[1] * Rparam[7] + 2 * Rparam[7] * sin(theta) * sin(psi) * sin(Rparam[9]) * z_m - 2 * cos(psi) * sin(Rparam[4]) * sin(Rparam[9]) * Rparam[1] * Rparam[7] - 2 * cos(psi) * cos(Rparam[9]) * Rparam[7] * sin(theta) * z_m - 2 * Rparam[1] * x_m * cos(Rparam[4]) + pow(Rparam[1],2) + pow(Rparam[7], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0)); 
    
	ForJ(2,0) = (cos(theta) * sin(psi) * Rparam[8] * sin(Rparam[10]) + cos(theta) * cos(psi) * Rparam[8] * cos(Rparam[10]) - Rparam[2] * cos(Rparam[5]) + x_m) * pow((-2 * sin(Rparam[10]) * cos(Rparam[5]) * cos(theta) * sin(psi) * Rparam[2] * Rparam[8] - 2 * cos(Rparam[10]) * cos(Rparam[5]) * cos(theta) * cos(psi) * Rparam[2] * Rparam[8] - 2 * sin(Rparam[10]) * sin(Rparam[5]) * cos(psi) * Rparam[2] * Rparam[8] + 2 * sin(Rparam[10]) * cos(theta) * sin(psi) * Rparam[8] * x_m - 2 * Rparam[8] * sin(theta) * sin(psi) * sin(Rparam[10]) * z_m + 2 * cos(Rparam[10]) * sin(Rparam[5]) * sin(psi) * Rparam[2] * Rparam[8] + 2 * cos(Rparam[10]) * cos(theta) * cos(psi) * Rparam[8] * x_m - 2 * cos(psi) * cos(Rparam[10]) * Rparam[8] * sin(theta) * z_m - 2 * Rparam[2] * x_m * cos(Rparam[5]) + pow(Rparam[2], 2) + pow(Rparam[8], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(2,1) = -(sin(theta) * sin(psi) * Rparam[8] * sin(Rparam[10]) + sin(theta) * cos(psi) * Rparam[8] * cos(Rparam[10]) - z_m) * pow((-2 * sin(Rparam[10]) * cos(Rparam[5]) * cos(theta) * sin(psi) * Rparam[2] * Rparam[8] - 2 * cos(Rparam[10]) * cos(Rparam[5]) * cos(theta) * cos(psi) * Rparam[2] * Rparam[8] - 2 * sin(Rparam[10]) * sin(Rparam[5]) * cos(psi) * Rparam[2] * Rparam[8] + 2 * sin(Rparam[10]) * cos(theta) * sin(psi) * Rparam[8] * x_m - 2 * Rparam[8] * sin(theta) * sin(psi) * sin(Rparam[10]) * z_m + 2 * cos(Rparam[10]) * sin(Rparam[5]) * sin(psi) * Rparam[2] * Rparam[8] + 2 * cos(Rparam[10]) * cos(theta) * cos(psi) * Rparam[8] * x_m - 2 * cos(psi) * cos(Rparam[10]) * Rparam[8] * sin(theta) * z_m - 2 * Rparam[2] * x_m * cos(Rparam[5]) + pow(Rparam[2], 2) + pow(Rparam[8], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(2,2) = -Rparam[8] * (-sin(theta) * sin(psi) * sin(Rparam[10]) * cos(Rparam[5]) * Rparam[2] - sin(theta) * cos(psi) * cos(Rparam[5]) * cos(Rparam[10]) * Rparam[2] + cos(theta) * sin(psi) * sin(Rparam[10]) * z_m + cos(theta) * cos(psi) * cos(Rparam[10]) * z_m + sin(theta) * sin(psi) * sin(Rparam[10]) * x_m + sin(theta) * cos(psi) * cos(Rparam[10]) * x_m) * pow((-2 * sin(Rparam[10]) * cos(Rparam[5]) * cos(theta) * sin(psi) * Rparam[2] * Rparam[8] - 2 * cos(Rparam[10]) * cos(Rparam[5]) * cos(theta) * cos(psi) * Rparam[2] * Rparam[8] - 2 * sin(Rparam[10]) * sin(Rparam[5]) * cos(psi) * Rparam[2] * Rparam[8] + 2 * sin(Rparam[10]) * cos(theta) * sin(psi) * Rparam[8] * x_m - 2 * Rparam[8] * sin(theta) * sin(psi) * sin(Rparam[10]) * z_m + 2 * cos(Rparam[10]) * sin(Rparam[5]) * sin(psi) * Rparam[2] * Rparam[8] + 2 * cos(Rparam[10]) * cos(theta) * cos(psi) * Rparam[8] * x_m - 2 * cos(psi) * cos(Rparam[10]) * Rparam[8] * sin(theta) * z_m - 2 * Rparam[2] * x_m * cos(Rparam[5]) + pow(Rparam[2], 2) + pow(Rparam[8], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(2,3) = Rparam[8] * (cos(theta) * sin(psi) * cos(Rparam[5]) * cos(Rparam[10]) * Rparam[2] - cos(theta) * cos(psi) * sin(Rparam[10]) * cos(Rparam[5]) * Rparam[2] - cos(theta) * sin(psi) * cos(Rparam[10]) * x_m + cos(theta) * cos(psi) * sin(Rparam[10]) * x_m + sin(theta) * sin(psi) * cos(Rparam[10]) * z_m - sin(theta) * cos(psi) * sin(Rparam[10]) * z_m + sin(psi) * sin(Rparam[10]) * sin(Rparam[5]) * Rparam[2] + cos(psi) * cos(Rparam[10]) * sin(Rparam[5]) * Rparam[2]) * pow((-2 * sin(Rparam[10]) * cos(Rparam[5]) * cos(theta) * sin(psi) * Rparam[2] * Rparam[8] - 2 * cos(Rparam[10]) * cos(Rparam[5]) * cos(theta) * cos(psi) * Rparam[2] * Rparam[8] - 2 * sin(Rparam[10]) * sin(Rparam[5]) * cos(psi) * Rparam[2] * Rparam[8] + 2 * sin(Rparam[10]) * cos(theta) * sin(psi) * Rparam[8] * x_m - 2 * Rparam[8] * sin(theta) * sin(psi) * sin(Rparam[10]) * z_m + 2 * cos(Rparam[10]) * sin(Rparam[5]) * sin(psi) * Rparam[2] * Rparam[8] + 2 * cos(Rparam[10]) * cos(theta) * cos(psi) * Rparam[8] * x_m - 2 * cos(psi) * cos(Rparam[10]) * Rparam[8] * sin(theta) * z_m - 2 * Rparam[2] * x_m * cos(Rparam[5]) + pow(Rparam[2], 2) + pow(Rparam[8], 2) + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));

	ForJ(3,0) = -(Rparam[3] - x_m) * pow((pow(Rparam[3], 2) - 2 * Rparam[3] * x_m + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0));
	ForJ(3,1) = pow((pow(Rparam[3], 2) - 2 * Rparam[3] * x_m + pow(x_m, 2) + pow(z_m, 2)), (-1.0 / 2.0)) * z_m;
	ForJ(3,2) = 0;
	ForJ(3,3) = 0;
}