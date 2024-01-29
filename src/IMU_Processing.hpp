#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#include "use-ikfom.hpp"
#include "esekfom.hpp"


#include <boost/filesystem.hpp>
#include "glog/logging.h"

#define MAX_INI_COUNT (10)  

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias);
  Eigen::Matrix<double, 12, 12> Q;    
  void Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &pcl_un_);

  V3D cov_acc;            
  V3D cov_gyr;            
  V3D cov_acc_scale;      
  V3D cov_gyr_scale;    
  V3D cov_bias_gyr;     
  V3D cov_bias_acc;    
  double first_lidar_time; 

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;      
  sensor_msgs::ImuConstPtr last_imu_;   
  vector<Pose6D> IMUpose;         
  M3D Lidar_R_wrt_IMU;             
  V3D Lidar_T_wrt_IMU;            
  V3D mean_acc;                   
  V3D mean_gyr;                
  V3D angvel_last;         
  V3D acc_s_last;                    
  double start_timestamp_;          
  double last_lidar_end_time_;       
  int init_iter_num = 1;                
  bool b_first_frame_ = true;           
  bool imu_need_init_ = true;            
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                          
  Q = process_noise_cov();               
  cov_acc = V3D(0.1, 0.1, 0.1);    
  cov_gyr = V3D(0.1, 0.1, 0.1);          
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001); 
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;                     
  Lidar_T_wrt_IMU = Zero3d;               
  Lidar_R_wrt_IMU = Eye3d;             
  last_imu_.reset(new sensor_msgs::Imu());   
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;              
  start_timestamp_ = -1;                 
  init_iter_num = 1;                  
  IMUpose.clear();                      
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI()); 
}


void ImuProcess::set_param(const V3D &transl, const M3D &rot, const V3D &gyr, const V3D &acc, const V3D &gyr_bias, const V3D &acc_bias)  
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
  cov_gyr_scale = gyr;
  cov_acc_scale = acc;
  cov_bias_gyr = gyr_bias;
  cov_bias_acc = acc_bias;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N)
{

  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_) 
  {
    Reset();   
    N = 1;    
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;  
    const auto &gyr_acc = meas.imu.front()->angular_velocity;    
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;        
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;      
    first_lidar_time = meas.lidar_beg_time;             
  }

  for (const auto &imu : meas.imu)  
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    V3D mean_acc_last = mean_acc;
    V3D mean_gyr_last = mean_gyr;

    mean_acc  += (cur_acc - mean_acc) / N;  
    mean_gyr  += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc_last)  / N;
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr_last)  / N;

    N ++;
  }
  
  state_ikfom init_state = kf_state.get_x();      
  init_state.grav = - mean_acc / mean_acc.norm() * G_m_s2;   
  
  init_state.bg  = mean_gyr;     
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;    
  init_state.offset_R_L_I = Sophus::SO3d(Lidar_R_wrt_IMU);
  kf_state.change_x(init_state);    

  Matrix<double, 24, 24> init_P = MatrixXd::Identity(24,24);    
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001; 
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;  
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;  
  init_P(21,21) = init_P(22,22) = init_P(23,23) = 0.00001;   
  kf_state.change_P(init_P);   
  last_imu_ = meas.imu.back();    
  
}


void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_out)
{

  auto v_imu = meas.imu;        
  v_imu.push_front(last_imu_); 
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();  
  const double &pcl_beg_time = meas.lidar_beg_time;    
  const double &pcl_end_time = meas.lidar_end_time;
  
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); 

  state_ikfom imu_state = kf_state.get_x();  
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.matrix()));
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu; 
  M3D R_imu;   

  double dt = 0;

  input_ikfom in;

  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);      
    auto &&tail = *(it_imu + 1);  
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),    
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr  = acc_avr * G_m_s2 / mean_acc.norm(); 

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_; 
    } 
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();    
    }
    
    in.acc = acc_avr;   
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;      
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

    kf_state.predict(dt, Q, in);   

    imu_state = kf_state.get_x();   

    angvel_last = V3D(tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z) - imu_state.bg;

    acc_s_last  = V3D(tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z) * G_m_s2 / mean_acc.norm();   

    acc_s_last = imu_state.rot * (acc_s_last - imu_state.ba) + imu_state.grav;

    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;    
    IMUpose.push_back( set_pose6d( offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.matrix() ) );
  }

  dt = abs(pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  imu_state = kf_state.get_x();  
  last_imu_ = meas.imu.back();       
  last_lidar_end_time_ = pcl_end_time;     
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;

  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);  
    vel_imu<<VEC_FROM_ARRAY(head->vel);    
    pos_imu<<VEC_FROM_ARRAY(head->pos);  
    acc_imu<<VEC_FROM_ARRAY(tail->acc);  
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);  

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;    
      M3D R_i(R_imu * Sophus::SO3d::exp(angvel_avr * dt).matrix() );   
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);   
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos); 
      V3D P_compensate = imu_state.offset_R_L_I.matrix().transpose() * (imu_state.rot.matrix().transpose() * (R_i * (imu_state.offset_R_L_I.matrix() * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}


double T1,T2;
void ImuProcess::Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr &cur_pcl_un_)
{
  // T1 = omp_get_wtime();

  if(meas.imu.empty())  {return;}; 
  ROS_ASSERT(meas.lidar != nullptr); 

  if (imu_need_init_)   
  {
    // The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);  
    
    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();

    if (init_iter_num > MAX_INI_COUNT)// MAX_INI_COUNT = 10
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2); 
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
    }
    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_); 
}
