#ifndef USE_IKFOM_H1
#define USE_IKFOM_H1

#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib.h"
#include "sophus/so3.hpp"

struct state_ikfom
{
	Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
	Sophus::SO3d rot = Sophus::SO3d(Eigen::Matrix3d::Identity());
	Sophus::SO3d offset_R_L_I = Sophus::SO3d(Eigen::Matrix3d::Identity());
	Eigen::Vector3d offset_T_L_I = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d grav = Eigen::Vector3d(0,0,-G_m_s2);
};

struct input_ikfom
{
	Eigen::Vector3d acc = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0);
};

Eigen::Matrix<double, 12, 12> process_noise_cov()
{
	Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);
	Q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();

	return Q;
}

Eigen::Matrix<double, 24, 1> get_f(state_ikfom s, input_ikfom in)	
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	Eigen::Vector3d omega = in.gyro - s.bg;		
	Eigen::Vector3d a_inertial = s.rot.matrix() * (in.acc - s.ba);	

	for (int i = 0; i < 3; i++)
	{
		res(i) = s.vel[i];	
		res(i + 3) = omega[i];	
		res(i + 12) = a_inertial[i] + s.grav[i];	
	}

	return res;
}

Eigen::Matrix<double, 24, 24> df_dx(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();	
	Eigen::Vector3d acc_ = in.acc - s.ba;   

	cov.block<3, 3>(12, 3) = -s.rot.matrix() * Sophus::SO3d::hat(acc_);	
	cov.block<3, 3>(12, 18) = -s.rot.matrix(); 				

	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();		
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();	
	return cov;
}

Eigen::Matrix<double, 24, 12> df_dw(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.block<3, 3>(12, 3) = -s.rot.matrix();				
	cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();		
	cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();		
	cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();	
	return cov;
}

#endif