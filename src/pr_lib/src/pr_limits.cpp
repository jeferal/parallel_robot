#include "pr_lib/pr_limits.hpp"

Eigen::Matrix<double,4,2> PRLimits::LimActuators(void) {

    double q13_lim_ini=0.6537, q23_lim_ini=0.6474, q33_lim_ini=0.6502, q42_lim_ini=0.549;
	double q_desp_max=0.28;

	Eigen::Matrix<double,4,2> Mlim_q_ind;

	Mlim_q_ind <<   q13_lim_ini, q13_lim_ini+q_desp_max,
			        q23_lim_ini, q23_lim_ini+q_desp_max,
			        q33_lim_ini, q33_lim_ini+q_desp_max,
			        q42_lim_ini, q42_lim_ini+q_desp_max;

	return Mlim_q_ind;
}

Eigen::Vector4d PRLimits::LimAngles(void) {
    
	int a_esf_max=38, a_uni_max=90;
	Eigen::Vector4d Vlim_Angp;

	Vlim_Angp << a_esf_max, a_esf_max, a_esf_max, a_uni_max;

	return Vlim_Angp;
}
