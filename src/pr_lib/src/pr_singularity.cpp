#include "pr_lib/pr_singularity.hpp"


Eigen::Matrix<double,6,1> PRSingularity::CalculateAngOts(
        const Eigen::Vector4d &X,
        const Eigen::Vector4d &q,
        Eigen::Matrix<double, 5, 1> &X_OTS,
        Eigen::Matrix<double,6,4> &solOTS,
        Eigen::Matrix<double, 5, 1> &Xn_OTS,
        std::vector<double> &RParam,
        int iter_OTS, double tol_OTS)
{
    double theta = X(2), psi = X(3);
	double error_OTS;
	int ci;
	Eigen::Matrix<double, 5, 1> f_OTS;
	Eigen::Matrix<double, 3, 1> ang_OTS_i, ang_OTS_j;
	Eigen::Matrix<double, 6, 1> sol_AngOTS;
	Eigen::Matrix<double, 5, 5> J_OTS;

    // SOLUCION DE LOS CUATRO OTS
	for (int op=1; op<=4; op++){
		
		// Punto inicial para solucionar el sistema de ecuaciones para un OTS
		X_OTS(0) = solOTS(0,op-1); X_OTS(1) = solOTS(1,op-1); X_OTS(2) = solOTS(2,op-1); X_OTS(3) = solOTS(3,op-1); X_OTS(4) = solOTS(5,op-1);
		
		// Error inicial para la resolucion del sistema basada en el punto inicial
		error_OTS = EqOTS(X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]).norm();

		// Iteracion inicial
		ci = 1;
		
		// Algoritmo de Newton Raphson
		while (error_OTS>tol_OTS){
			// Funcion con las ecuaciones que determinan los componentes de un OTS
			f_OTS = EqOTS(X_OTS(0), X_OTS(1), X_OTS(2), X_OTS(3), X_OTS(4), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
			
			// Error de la solucion actual
			error_OTS = f_OTS.norm();
			
			// Jacobiano del sistema de ecuaciones para un OTS
			J_OTS = EqOTSJacobian(X_OTS(0), X_OTS(1), X_OTS(2), theta, psi, q, op, RParam[6], RParam[7], RParam[8], RParam[9], RParam[10]);
			
			// Calculo de la nueva solucion
			//Xn_OTS = X_OTS - linSolve(J_OTS, f_OTS);
			Xn_OTS = X_OTS - J_OTS.partialPivLu().solve(f_OTS);
			//Xn_OTS = X_OTS - J_OTS.inverse()*f_OTS;

			// Actualizo la solucion de un OTS
			X_OTS = Xn_OTS;
			//cout << X_OTS.transpose() << endl;
			
			// Incremento el contador de iteraciones
			ci++;
			// Condicion para evitar bucles infinitos
			if (ci>iter_OTS) 
                break;
		}
		
		// Almaceno la solucion del OTS seleccionado por op
		solOTS(0,op-1)=X_OTS(0);
		solOTS(1,op-1)=X_OTS(1);
		solOTS(2,op-1)=X_OTS(2);
		solOTS(3,op-1)=X_OTS(3);
		solOTS(4,op-1)=0;
		solOTS(5,op-1)=X_OTS(4);
    }
	
    // ANGULO ENTRE DOS OUTPUT TWIST SCREW CALCULADOS (Componentes angular y lineal)
	// Indice donde se almacenara el angulo
	int k=0;
	// Contador primer OTS
	for (int i=0;i<3;i++){
		// Contador segundo OTS
		for (int j=i+1;j<4;j++){
			// OTS - Componente Angular
			ang_OTS_i = (solOTS.col(i)).head(3);
			ang_OTS_j = (solOTS.col(j)).head(3);
			sol_AngOTS(k) = acos(ang_OTS_i.dot(ang_OTS_j)/(ang_OTS_i.norm()*ang_OTS_j.norm()))*180/M_PI;
			// Incremento el indice de almacenamiento del angulo entre dos OTS
			k++;

		}
	}
	
	return sol_AngOTS;
}

Eigen::Matrix<double, 5, 5> PRSingularity::EqOTSJacobian(
        const double &wx, const double &wy, const double &wz, 
        const double &theta, const double &psi, 
        const Eigen::Vector4d &q, 
        const int &op, 
        const double &Rm1, const double &Rm2, const double &Rm3, 
        const double &betaMD, const double &betaMI)
{
    Eigen::Matrix<double, 5, 5> J = Eigen::Matrix<double, 5, 5>::Zero(5,5);
	double q11 = q(0), q12 = q(1), q13 = q(2);
	double q21 = q(3), q22 = q(4), q23 = q(5);
	double q31 = q(6), q32 = q(7), q33 = q(8);
	double q41 = q(9), q42 = q(10);

	switch (op){
		case 1:
		
			J(0,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(0,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(0,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(0,3) = cos(q21) * sin(q22);
			J(0,4) = sin(q21) * sin(q22);
			
			J(1,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32));
			J(1,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31));
			J(1,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32));
			J(1,3) = cos(q31) * sin(q32);
			J(1,4) = sin(q31) * sin(q32);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;
			break;

		case 2:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = cos(q11) * sin(q12);
			
			J(1,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32));
			J(1,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31));
			J(1,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32));
			J(1,3) = cos(q31) * sin(q32);
			J(1,4) = sin(q31) * sin(q32);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

		case 3:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = sin(q11) * sin(q12);
			
			J(1,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(1,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(1,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(1,3) = cos(q21) * sin(q22);
			J(1,4) = sin(q21) * sin(q22);

			J(2,0) = 0; J(2,1) = 0; J(2,2) = 0; J(2,3) = -sin(q41); J(2,4) = cos(q41);
			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

		case 4:

			J(0,0) = -Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi));
			J(0,1) = sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11));
			J(0,2) = Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi));
			J(0,3) = cos(q11) * sin(q12);
			J(0,4) = sin(q11) * sin(q12);
			
			J(1,0) = -Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22));
			J(1,1) = -sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21));
			J(1,2) = -Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22));
			J(1,3) = cos(q21) * sin(q22);
			J(1,4) = sin(q21) * sin(q22);

			J(2,0) = -Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)); 
			J(2,1) = -sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)); 
			J(2,2) = -Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)); 
			J(2,3) = cos(q31) * sin(q32); 
			J(2,4) = sin(q31) * sin(q32);

			J(3,0) = 1; J(3,1) = 0; J(3,2) = -(sin(theta)/cos(theta)); J(3,3) = 0; J(3,4) = 0;
			J(4,0) = 2*wx; J(4,1) = 2*wy; J(4,2) = 2*wz; J(4,3) = 0; J(4,4) = 0;

			break;

	}

	return J;
}


Eigen::Matrix<double,5,1> PRSingularity::EqOTS(
       const double &wx, const double &wy, const double &wz, 
       const double &vx, const double &vz, 
       const double &theta, const double &psi, 
       const Eigen::Vector4d &q, 
       const int &op, 
       const double &Rm1, const double &Rm2, const double &Rm3, 
       const double &betaMD, const double &betaMI 
)
{
    //A partir de la orientacion (theta y psi) del centro de la plataforma del robot paralelo y de la solucion de la cinematica inversa determinamos las componentes del Output Screw Salida (OTS). El OTS a determinar se selecciona mediante la variable op.
	
	Eigen::Matrix<double,5,1> f = Eigen::Matrix<double,5,1>::Zero(5);

	f(3) = wx - (sin(theta)/cos(theta))*wz;
	f(4) = pow(wx,2) + pow(wy,2) + pow(wz,2) -1;
	double q11 = q(0), q12 = q(1), q13 = q(2);
	double q21 = q(3), q22 = q(4), q23 = q(5);
	double q31 = q(6), q32 = q(7), q33 = q(8);
	double q41 = q(9), q42 = q(10); 

	switch (op){
	
		case 1:
			f(0) = cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * cos(betaMD) * cos(q22) * sin(theta) - cos(psi) * sin(betaMD) * sin(q22) * sin(q21) - sin(psi) * cos(betaMD) * sin(q22) * sin(q21) - sin(psi) * sin(betaMD) * cos(q22) * sin(theta)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(betaMD) * cos(q21) * sin(theta) + cos(psi) * cos(betaMD) * cos(theta) * sin(q21) - sin(psi) * sin(betaMD) * cos(q21) * sin(theta) - sin(psi) * sin(betaMD) * cos(theta) * sin(q21)) * wy - Rm2 * (cos(psi) * cos(betaMD) * cos(q22) * cos(theta) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22) - sin(psi) * sin(betaMD) * cos(q22) * cos(theta)) * wz;
			f(1) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 2:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 3:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) = cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21)) * wy - Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22)) * wz;
			f(2) = -sin(q41) * vx + cos(q41) * vz;
			break;

		case 4:
			f(0) = cos(q11) * sin(q12) * vx + sin(q11) * sin(q12) * vz - Rm1 * (sin(q11) * sin(q12) * sin(psi) - sin(theta) * cos(q12) * cos(psi)) * wx + sin(q12) * cos(psi) * Rm1 * (sin(q11) * cos(theta) + sin(theta) * cos(q11)) * wy + Rm1 * (cos(theta) * cos(q12) * cos(psi) + cos(q11) * sin(q12) * sin(psi)) * wz;
			f(1) =  cos(q21) * sin(q22) * vx + sin(q21) * sin(q22) * vz - Rm2 * (cos(psi) * sin(theta) * cos(betaMD) * cos(q22) - cos(psi) * sin(q21) * sin(betaMD) * sin(q22) - sin(psi) * sin(theta) * sin(betaMD) * cos(q22) - sin(psi) * sin(q21) * cos(betaMD) * sin(q22)) * wx - sin(q22) * Rm2 * (cos(psi) * cos(theta) * sin(q21) * cos(betaMD) + cos(psi) * sin(theta) * cos(betaMD) * cos(q21) - sin(psi) * cos(theta) * sin(q21) * sin(betaMD) - sin(psi) * sin(theta) * sin(betaMD) * cos(q21)) * wy - Rm2 * (cos(psi) * cos(theta) * cos(betaMD) * cos(q22) + cos(psi) * sin(betaMD) * cos(q21) * sin(q22) - sin(psi) * cos(theta) * sin(betaMD) * cos(q22) + sin(psi) * cos(betaMD) * cos(q21) * sin(q22)) * wz;
			f(2) = cos(q31) * sin(q32) * vx + sin(q31) * sin(q32) * vz - Rm3 * (cos(psi) * sin(theta) * cos(betaMI) * cos(q32) + cos(psi) * sin(q31) * sin(betaMI) * sin(q32) + sin(psi) * sin(theta) * sin(betaMI) * cos(q32) - sin(psi) * sin(q31) * cos(betaMI) * sin(q32)) * wx - sin(q32) * Rm3 * (cos(psi) * cos(theta) * sin(q31) * cos(betaMI) + cos(psi) * sin(theta) * cos(betaMI) * cos(q31) + sin(psi) * cos(theta) * sin(q31) * sin(betaMI) + sin(psi) * sin(theta) * sin(betaMI) * cos(q31)) * wy - Rm3 * (cos(psi) * cos(theta) * cos(betaMI) * cos(q32) - cos(psi) * sin(betaMI) * cos(q31) * sin(q32) + sin(psi) * cos(theta) * sin(betaMI) * cos(q32) + sin(psi) * cos(betaMI) * cos(q31) * sin(q32)) * wz;
			break;

	}
	return f;
}