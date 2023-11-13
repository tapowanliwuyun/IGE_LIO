#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1

#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include "use-ikfom.hpp"
#include <ikd-Tree/ikd_Tree.h>

const double epsi = 0.001;    	  //ESKF迭代时，如果dx<epsi 认为收敛

namespace esekfom
{
	using namespace Eigen;

	PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));		 
	PointCloudXYZI::Ptr normvec_n_edge(new PointCloudXYZI(100000, 1));		
	PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1)); //
	PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1)); //
	PointCloudXYZI::Ptr corr_normvect_n_edge(new PointCloudXYZI(100000, 1)); //
	bool point_selected_surf[100000] = {1};	   //

	struct dyn_share_datastruct
	{
		bool valid;			//
		bool converge;		//
		Eigen::Matrix<double, Eigen::Dynamic, 1> h;				   //	
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_matrix;			
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; 
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x_T_R_inv; 
	};

	
	class esekf
	{
	public:
		typedef Matrix<double, 24, 24> cov; 			
		typedef Matrix<double, 24, 1> vectorized_state;	

		esekf(){};
		~esekf(){};

		state_ikfom get_x()
		{
			return x_;
		}

		cov get_P()
		{
			return P_;
		}

		void change_x(state_ikfom &input_state)
		{
			x_ = input_state;
		}

		void change_P(cov &input_cov)
		{
			P_ = input_cov;
		}

		state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_)
		{
			state_ikfom x_r;
			x_r.pos = x.pos + f_.block<3, 1>(0, 0);

			x_r.rot = x.rot * Sophus::SO3d::exp(f_.block<3, 1>(3, 0) );
			x_r.offset_R_L_I = x.offset_R_L_I * Sophus::SO3d::exp(f_.block<3, 1>(6, 0) );
			x_r.offset_T_L_I = x.offset_T_L_I + f_.block<3, 1>(9, 0);
			x_r.vel = x.vel + f_.block<3, 1>(12, 0);
			x_r.bg = x.bg + f_.block<3, 1>(15, 0);
			x_r.ba = x.ba + f_.block<3, 1>(18, 0);
			x_r.grav = x.grav + f_.block<3, 1>(21, 0);

			return x_r;
		}
		
		void predict(double &dt, Eigen::Matrix<double, 12, 12> &Q, const input_ikfom &i_in)
		{
			Eigen::Matrix<double, 24, 1> f_ = get_f(x_, i_in);	  
			Eigen::Matrix<double, 24, 24> f_x_ = df_dx(x_, i_in); 
			Eigen::Matrix<double, 24, 12> f_w_ = df_dw(x_, i_in); 

			x_ = boxplus(x_, f_*dt); 

			f_x_ = Matrix<double, 24, 24>::Identity() + f_x_ * dt; 

			P_ = (f_x_)*P_ * (f_x_).transpose() + (dt * f_w_) * Q * (dt * f_w_).transpose(); 
		}

		void h_share_model(dyn_share_datastruct &ekfom_data, PointCloudXYZI::Ptr &feats_down_body,
						   KD_TREE<PointType> &ikdtree, vector<PointVector> &Nearest_Points, bool extrinsic_est , int edge_plane_flag = 3, double R = 0.001, double plane_cov = 1 )
		{
			int feats_down_size = feats_down_body->points.size();
			laserCloudOri->clear(); 
			corr_normvect->clear();  
			corr_normvect_n_edge->clear();
			#ifdef MP_EN
    			omp_set_num_threads(MP_PROC_NUM);
			#pragma omp parallel for
			#endif  
			
   			 //
			for (int i = 0; i < feats_down_size; i++) //
			{
				PointType &point_body = feats_down_body->points[i];
				PointType point_world;

				V3D p_body;
				V3D p_global;
				if(point_body.normal_z == 4 )
				{
					p_global(0)  = point_body.x;
					p_global(1)  = point_body.y;
					p_global(2)  = point_body.z;

					V3D p_body_tmp( x_.offset_R_L_I.inverse() * (x_.rot.inverse() * (p_global - x_.pos) -  x_.offset_T_L_I )  );

					p_body(0)  = p_body_tmp(0);
					p_body(1)  = p_body_tmp(1);
					p_body(2)  = p_body_tmp(2);
					point_body.normal_z = 3;

					feats_down_body->points[i].normal_y = 3;
					feats_down_body->points[i].x = p_body(0);
					feats_down_body->points[i].y = p_body(1);
					feats_down_body->points[i].z = p_body(2);



				}
				else
				{
					p_body(0)  = point_body.x;
					p_body(1)  = point_body.y;
					p_body(2)  = point_body.z;
					//
					V3D p_global_tmp(x_.rot * (x_.offset_R_L_I * p_body + x_.offset_T_L_I) + x_.pos);
					p_global(0)  = p_global_tmp(0);
					p_global(1)  = p_global_tmp(1);
					p_global(2)  = p_global_tmp(2);
				}
			
				point_world.x = p_global(0);
				point_world.y = p_global(1);
				point_world.z = p_global(2);
				point_world.intensity = point_body.intensity;

				vector<float> pointSearchSqDis_tmp(NUM_MATCH_POINTS*3);
				vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
				PointVector points_near_tmp(NUM_MATCH_POINTS);
				auto &points_near = Nearest_Points[i]; // Nearest_Points[i] 

				double ta = omp_get_wtime();
				if (ekfom_data.converge)//
				{
					ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS*3, points_near_tmp, pointSearchSqDis_tmp);
					int Nearest_num = -1;
					int Nearest_invaild_num = 0;
					PointVector().swap(points_near);
					vector<float>().swap(pointSearchSqDis);
					for(vector<float>::iterator it  = pointSearchSqDis_tmp.begin() ; it != pointSearchSqDis_tmp.end(); it++)
					{
						Nearest_num++;
						if(points_near_tmp[Nearest_num].normal_z != point_body.normal_z || ( (points_near_tmp[Nearest_num].normal_z == 4) && ( (point_body.normal_z == 1) || (point_body.normal_z == 2) )  ) || ( (points_near_tmp[Nearest_num].normal_z == 3) && ( (point_body.normal_z == 1) || (point_body.normal_z == 2) )  )) 
						{
							continue;
						}
						else
						{
							pointSearchSqDis.push_back( pointSearchSqDis_tmp[Nearest_num] );
							points_near.push_back(points_near_tmp[Nearest_num]);
							Nearest_invaild_num++;
						}
						if(Nearest_invaild_num == NUM_MATCH_POINTS) break;
					}
					if(point_body.normal_z == 1)
					{
						if(edge_plane_flag == 1 || edge_plane_flag == 3)
							point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[Nearest_invaild_num - 1] > 5 ? false: true;
						else
							point_selected_surf[i] = false;
					}
					else if(point_body.normal_z == 2)
					{
						if(edge_plane_flag == 1 || edge_plane_flag == 3)
							point_selected_surf[i] = points_near.size() < 3 ? false : pointSearchSqDis[Nearest_invaild_num - 1] > 5 ? false	: true;	
						else
							point_selected_surf[i] = false;

					}
					else if(point_body.normal_z == 3)
					{
						if(edge_plane_flag == 2 || edge_plane_flag == 3)
							point_selected_surf[i] = points_near.size() < 3 ? false : pointSearchSqDis[Nearest_invaild_num - 1] > 5 ? false	: true;	
						else
							point_selected_surf[i] = false;

					}
				}

				if (!point_selected_surf[i])
					continue; //  

				Matrix<float, 4, 1> pabcd;		//
				point_selected_surf[i] = false; //

				if( point_body.normal_z == 1 )
				{
					if (esti_plane(pabcd, points_near, 0.1f))//
					{
						float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); //
						float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); //

						if (s > 0.9) //
						{
							point_selected_surf[i] = true;//
							normvec->points[i].x = pabcd(0); //   
							normvec->points[i].y = pabcd(1);
							normvec->points[i].z = pabcd(2);
							normvec->points[i].intensity = pd2;//
						}
					}
				}
				else
				{
					int near_point;
					if (esti_edge(pabcd, points_near, near_point))//
					{
						V3D pabcd_(pabcd(0), pabcd(1), pabcd(2));
						M3D pabcd_crossmat;
						pabcd_crossmat << SKEW_SYM_MATRX(pabcd_);

						V3D pabcd_1( point_world.x - points_near[0].x ,point_world.y - points_near[0].y , point_world.z - points_near[0].z );
						V3D pabcd_2( point_world.x - points_near[1].x ,point_world.y - points_near[1].y , point_world.z - points_near[1].z );
						
						V3D u_edge = pabcd_crossmat * ((pabcd_1 +pabcd_2)/2);
						
						float pd2 = u_edge[0] * u_edge[0] + u_edge[1] * u_edge[1] + u_edge[2] * u_edge[2]; //
						pd2  = sqrt(pd2);
						float s = 1 - 0.9 * fabs(pd2) ; //    

						if (s > 0.9 && s != 1) //
						{
							point_selected_surf[i] = true;//
							normvec->points[i].x = pabcd(0); 
							normvec->points[i].y = pabcd(1);
							normvec->points[i].z = pabcd(2);
							normvec->points[i].intensity = pd2;//

							normvec_n_edge->points[i].x = u_edge[0];
							 normvec_n_edge->points[i].y = u_edge[1];
							normvec_n_edge->points[i].z = u_edge[2];
						}
					}
				}
			}
			int effct_feat_num = 0; //
			int effct_feat_num_plane = 0; //
			int effct_feat_num_edge = 0; //
			int effct_feat_num_inten_edge = 0; //
			int effct_feat_num_window_edge = 0; //
			for (int i = 0; i < feats_down_size; i++)
			{
				if (point_selected_surf[i]) //
				{
					laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; //
					corr_normvect->points[effct_feat_num] = normvec->points[i];			//
					corr_normvect_n_edge->points[effct_feat_num] = normvec_n_edge->points[i];
					effct_feat_num ++;  //

					if(feats_down_body->points[i].normal_z == 1)
					{
						effct_feat_num_plane ++;
					}
					else if(feats_down_body->points[i].normal_z == 2)
					{
						effct_feat_num_edge ++;
					}
					else if(feats_down_body->points[i].normal_z == 3)
					{
						effct_feat_num_inten_edge ++;
					}
					else
					{
						effct_feat_num_window_edge++;
					}
				}
			}

			if (effct_feat_num < 1)
			{
				ekfom_data.valid = false;
				ROS_WARN("No Effective Points! \n");
				return;
			}

			ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //
			ekfom_data.h_x_T_R_inv = MatrixXd::Zero(effct_feat_num, 12);
			ekfom_data.h.resize(effct_feat_num); //
			ekfom_data.R_matrix = MatrixXd::Zero(effct_feat_num,effct_feat_num);
			for (int i = 0; i < effct_feat_num; i++)
			{
				double r_;
				if( laserCloudOri->points[i].normal_z == 1 )
				{
					V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
					M3D point_crossmat;//
					point_crossmat << SKEW_SYM_MATRX(point_);        // 
					V3D point_I_ = x_.offset_R_L_I * point_ + x_.offset_T_L_I;
					M3D point_I_crossmat;
					point_I_crossmat << SKEW_SYM_MATRX(point_I_);//

					const PointType &norm_p = corr_normvect->points[i];
					V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

					V3D C(x_.rot.matrix().transpose() * norm_vec);    //

					V3D A(point_I_crossmat * C);    

					double range_plane_cov = 1/(1+exp(-laserCloudOri->points[i].intensity));
					r_ = (R * plane_cov * range_plane_cov );
					if (extrinsic_est)
					{
						V3D B(point_crossmat * x_.offset_R_L_I.matrix().transpose() * C); //
						ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
						
						A = A/r_;
						B = B/r_;
						C = C/r_;
						ekfom_data.h_x_T_R_inv.block<1, 12>(i, 0) << norm_p.x/r_, norm_p.y/r_, norm_p.z/r_, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
					}
					else
					{
						ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
						A = A/r_;
						ekfom_data.h_x_T_R_inv.block<1, 12>(i, 0) << norm_p.x/r_, norm_p.y/r_, norm_p.z/r_, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
					}

					ekfom_data.h(i) = -norm_p.intensity;//

				}
				else
				{
					V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
					M3D point_crossmat;//
					point_crossmat << SKEW_SYM_MATRX(point_);        // 
					// 
					V3D point_I_ = x_.offset_R_L_I * point_ + x_.offset_T_L_I;
					M3D point_I_crossmat;
					point_I_crossmat << SKEW_SYM_MATRX(point_I_);//

					const PointType &norm_p = corr_normvect->points[i];
					V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
					M3D norm_vec_crossmat;
					norm_vec_crossmat << SKEW_SYM_MATRX(norm_vec);

					V3D n_edge_( corr_normvect_n_edge->points[i].x/ norm_p.intensity , corr_normvect_n_edge->points[i].y/ norm_p.intensity , corr_normvect_n_edge->points[i].z/ norm_p.intensity );
					V3D D( norm_vec_crossmat *  n_edge_ );
					V3D A(  -point_I_crossmat * x_.rot.matrix().transpose() * D );      

					if(laserCloudOri->points[i].normal_z == 3 || laserCloudOri->points[i].normal_z == 4)
					{
						r_ = (0.6 * R * laserCloudOri->points[i].intensity);
					}
					else
					{
						double range_edge_cov = 1/(1+exp(-laserCloudOri->points[i].intensity));
						r_ = (0.6 * R * range_edge_cov);
					}

					if (extrinsic_est)
					{
						V3D B(  -point_crossmat *  x_.offset_R_L_I.matrix().transpose()  * x_.rot.matrix().transpose()   * D    ); 
						V3D C(  -x_.rot.matrix().transpose() * D  );  
						ekfom_data.h_x.block<1, 12>(i, 0) << VEC_FROM_ARRAY(-D),  VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
						A = A/r_;
						B = B/r_;
						C = C/r_;
						D = D/r_;
						ekfom_data.h_x_T_R_inv.block<1, 12>(i, 0) << VEC_FROM_ARRAY(-D),  VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
					}
					else
					{
						ekfom_data.h_x.block<1, 12>(i, 0) << VEC_FROM_ARRAY(-D), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
						A = A/r_;
						D = D/r_;
						ekfom_data.h_x_T_R_inv.block<1, 12>(i, 0) << VEC_FROM_ARRAY(-D),  VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
					}

					ekfom_data.h(i) = -norm_p.intensity;//
				}

			}
		}

		vectorized_state boxminus(state_ikfom x1, state_ikfom x2 )
		{
			vectorized_state x_r = vectorized_state::Zero();

			x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;

			x_r.block<3, 1>(3, 0) = Sophus::SO3d( x2.rot.matrix().transpose() * x1.rot.matrix() ).log() ;
			x_r.block<3, 1>(6, 0) = Sophus::SO3d( x2.offset_R_L_I.matrix().transpose() * x1.offset_R_L_I.matrix() ).log() ;

			x_r.block<3, 1>(9, 0) = x1.offset_T_L_I - x2.offset_T_L_I;
			x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
			x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
			x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
			x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

			return x_r;
		}

		void update_iterated_dyn_share_modified(double R, PointCloudXYZI::Ptr &feats_down_body,
												KD_TREE<PointType> &ikdtree, vector<PointVector> &Nearest_Points, int maximum_iter, bool extrinsic_est , int edge_plane_flag = 3, double plane_cov = 1, bool is_our_weight = false)
		{
			normvec->resize(int(feats_down_body->points.size()));
			normvec_n_edge->resize(int(feats_down_body->points.size()));
			dyn_share_datastruct dyn_share;
			dyn_share.valid = true;
			dyn_share.converge = true;
			int t = 0;
			state_ikfom x_propagated = x_; //
			cov P_propagated = P_;

			vectorized_state dx_new = vectorized_state::Zero(); // 
			std::cout <<  std::endl;
			for (int i = -1; i < maximum_iter; i++)	
			{				
				dyn_share.valid = true;
				
				h_share_model(dyn_share, feats_down_body, ikdtree, Nearest_Points, extrinsic_est , edge_plane_flag , R , plane_cov ); 

				if(! dyn_share.valid)
				{
					continue;
				}

				vectorized_state dx;
				dx_new = boxminus(x_, x_propagated); 

				auto H = dyn_share.h_x;  
				auto H_t_R_inv  = dyn_share.h_x_T_R_inv;

				Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
				if(!is_our_weight)
				{
					Eigen::Matrix<double, 24, 24> HTH = Matrix<double, 24, 24>::Zero(); 
					HTH.block<12, 12>(0, 0) = H.transpose() * H;
					auto K_front = (HTH / R + P_.inverse()).inverse();//
					K = K_front.block<24, 12>(0, 0) * H.transpose() / R;  //
				}
				else
				{
					Eigen::Matrix<double, 24, 24> HTR_H = Matrix<double, 24, 24>::Zero();   
					HTR_H.block<12, 12>(0, 0) = H_t_R_inv.transpose() * H;
					auto K_front = (HTR_H  + P_.inverse()).inverse();//
					K = K_front.block<24, 12>(0, 0) * H_t_R_inv.transpose()  ;  //
				}
				Eigen::Matrix<double, 24, 24> KH = Matrix<double, 24, 24>::Zero();  
				KH.block<24, 12>(0, 0) = K * H;
				Matrix<double, 24, 1> dx_ = K * dyn_share.h + (KH - Matrix<double, 24, 24>::Identity()) * dx_new;   
				x_ = boxplus(x_, dx_);	//(18)
				dyn_share.converge = true;
				for(int j = 0; j < 24 ; j++)
				{
					if(std::fabs(dx_[j]) > epsi)   
					{
						dyn_share.converge = false;
						break;
					}
				}

				if(dyn_share.converge) t++;

				if(!t && i == maximum_iter - 2) 
				{
					dyn_share.converge = true;
				}

				if(t > 1 || i == maximum_iter - 1)
				{
					P_ = (Matrix<double, 24, 24>::Identity() - KH) * P_ ;   
					return;
				}
 
			}
		}

	private:
		state_ikfom x_;
		cov P_ = cov::Identity();
	};

} // namespace esekfom

#endif 
