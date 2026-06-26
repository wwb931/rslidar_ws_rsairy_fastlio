#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
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
#include "preprocess.h"

/// *************Preconfiguration

#define MAX_INI_COUNT (10) //定义最大初始化次数

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);}; //定义排序比骄函数

/// *************IMU Process and undistortion
class ImuProcess
{
 public:  //对外服务接口
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //EIGEN数据对齐

  ImuProcess();  //构造函数 
  ~ImuProcess();  //析构函数
  
  void Reset(); //重载重置方法
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);  //重载外参设置接口
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;  //过程噪声协方差矩阵
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);
  // 一个误差状态迭代卡尔曼滤波器
  // 状态类型：state_ikfom（IMU状态:姿态、位置、速度、零偏和重力）

  // 输入类型：input_ikfom（IMU测量:角速度 gyro、加速度 acc）

  // 噪声维度：12（陀螺仪噪声3 + 加速度计噪声3 + 陀螺仪偏置噪声3 + 加速度计偏置噪声3）

  ofstream fout_imu;  // output file stream：输出文件流,专门用于记录 IMU 数据到日志文件，方便调试和性能分析。
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;

 private:  //内部工具函数
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);  //声明和定义分离
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;  // IMU数据处理的起始时间戳，通常用于IMU数据的时间同步和初始化。
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true; //是否为第一帧点云
  bool   imu_need_init_ = true; //是否需要进行IMU初始化（重置），通常在系统启动或重置时为true，完成初始化后设为false，除非系统再次需要重置。
};

ImuProcess::ImuProcess() //初始化列表，成员变量在构造函数体执行前被初始化
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)  // 构造函数初始化成员变量
{
  init_iter_num = 1;
  Q = process_noise_cov();  //调用函数获取过程噪声协方差矩阵，并将其赋值给成员变量 Q，供后续的卡尔曼滤波器使用。
  cov_acc       = V3D(0.1, 0.1, 0.1);  //加速度计噪声协方差
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;  //上一次的角速度测量值，初始化为零向量，通常用于计算当前角速度与上一次测量之间的差异，以估计系统的旋转变化。
  Lidar_T_wrt_IMU = Zero3d;  // LiDAR相对于IMU的平移向量，初始化为零向量，表示LiDAR和IMU在同一位置。这个变量通常用于将LiDAR测量转换到IMU坐标系下，以实现传感器融合。
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());  // last_imu_是一个智能指针，指向一个新的sensor_msgs::Imu对象，表示当前系统中最后一次接收到的IMU测量数据。这个变量通常用于存储和访问最新的IMU数据，以便在后续的处理和滤波过程中使用。
}

ImuProcess::~ImuProcess() {}  //析构函数，当前没有需要特殊处理的资源释放，因此函数体为空。

void ImuProcess::Reset()  //重置方法，重新初始化IMU处理器的状态和参数，通常在系统启动或需要重新校准时调用。
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);  //加速度计测量的平均值，初始假设为静止状态下的重力加速度，指向下方。
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;  //上一次的角速度测量值，初始化为零向量，通常用于计算当前角速度与上一次测量之间的差异，以估计系统的旋转变化。
  imu_need_init_    = true;  //是否需要进行IMU初始化（重置），通常在系统启动或重置时为true，完成初始化后设为false，除非系统再次需要重置。
  start_timestamp_  = -1;  // IMU数据处理的起始时间戳，通常用于IMU数据的时间同步和初始化。
  init_iter_num     = 1;    //初始化迭代次数，通常用于IMU数据的初始估计过程，随着每次重置而递增，以便在连续的重置过程中逐步增加对IMU状态的估计精度。
  v_imu_.clear(); // IMU测量数据的双端队列，清空其中的所有元素，通常在系统重置时调用，以确保后续处理使用的是新的IMU数据。
  IMUpose.clear();  // 存储IMU姿态信息的向量，清空其中的所有元素，通常在系统重置时调用，以确保后续处理使用的是新的IMU姿态数据。
  last_imu_.reset(new sensor_msgs::Imu());  // last_imu_是一个智能指针，指向一个新的sensor_msgs::Imu对象，表示当前系统中最后一次接收到的IMU测量数据。这个变量通常用于存储和访问最新的IMU数据，以便在后续的处理和滤波过程中使用。
  cur_pcl_un_.reset(new PointCloudXYZI());  // cur_pcl_un_是一个智能指针，指向一个新的PointCloudXYZI对象，表示当前处理的点云数据。这个变量通常用于存储和访问当前帧的点云数据，以便在后续的处理和滤波过程中使用。
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)  //重载外参设置接口，接受一个4x4的变换矩阵 T，通常包含旋转和平移信息，用于定义LiDAR相对于IMU的坐标变换关系。
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl) //重载外参设置接口，接受一个3D向量 transl，表示LiDAR相对于IMU的平移关系，旋转部分默认设置为单位矩阵，表示LiDAR和IMU之间没有旋转关系。
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)  //重载外参设置接口，接受一个3D向量 transl 和一个3x3旋转矩阵 rot，分别表示LiDAR相对于IMU的平移和旋转关系，用于定义LiDAR和IMU之间的完整坐标变换。
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)  
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)  // IMU初始化方法，接受当前测量数据 meas、卡尔曼滤波器对象 kf_state 和一个整数 N，通常用于根据初始的IMU测量数据估计系统的初始状态，包括重力加速度、陀螺仪偏置和加速度计偏置等参数，以便为后续的滤波过程提供一个合理的初始估计。
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr; // 当前的加速度和角速度测量值，通常用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration; // 当前IMU测量数据中的加速度分量，通常用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    const auto &gyr_acc = meas.imu.front()->angular_velocity; // 当前IMU测量数据中的角速度分量，通常用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z; // 将当前IMU测量数据中的加速度分量赋值给 mean_acc，用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z; // 将当前IMU测量数据中的角速度分量赋值给 mean_gyr，用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu) 
  {
    const auto &imu_acc = imu->linear_acceleration; // 当前IMU测量数据中的加速度分量，通常用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    const auto &gyr_acc = imu->angular_velocity; // 当前IMU测量数据中的角速度分量，通常用于计算IMU状态的初始估计，包括重力加速度的方向和陀螺仪偏置等参数。
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N; // 计算当前加速度测量值与平均加速度的差异，并根据当前样本数量 N 更新平均加速度 mean_acc，以便在IMU初始化过程中逐步估计重力加速度的方向和大小。
    mean_gyr      += (cur_gyr - mean_gyr) / N; // 计算当前角速度测量值与平均角速度的差异，并根据当前样本数量 N 更新平均角速度 mean_gyr，以便在IMU初始化过程中逐步估计陀螺仪偏置和系统的初始旋转状态。

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); // 更新加速度测量的协方差 cov_acc，使用当前加速度测量值 cur_acc 与平均加速度 mean_acc 的差异来计算新的协方差，以便在IMU初始化过程中逐步估计加速度计的噪声特性和不确定性。
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N); // 更新角速度测量的协方差 cov_gyr，使用当前角速度测量值 cur_gyr 与平均角速度 mean_gyr 的差异来计算新的协方差，以便在IMU初始化过程中逐步估计陀螺仪的噪声特性和不确定性。

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl; // 输出当前加速度测量值的范数和平均加速度的范数，用于调试和验证IMU初始化过程中重力加速度的估计是否合理。

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();  // 获取当前卡尔曼滤波器的状态估计 init_state，通常用于在IMU初始化过程中设置初始状态，包括重力加速度、陀螺仪偏置和加速度计偏置等参数。
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2); // 计算重力加速度向量的方向和大小，并将其赋值给 init_state.grav，用于在IMU初始化过程中设置初始状态，以便为后续的滤波过程提供一个合理的初始估计。
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity))); // 计算旋转矩阵，将平均加速度向量 mean_acc 与单位向量 (0, 0, -1) 的叉积作为旋转轴，使用指数映射函数 Exp 将其转换为旋转矩阵，并将其赋值给 init_state.rot，用于在IMU初始化过程中设置初始状态，以便为后续的滤波过程提供一个合理的初始估计。
  init_state.bg  = mean_gyr; // 计算陀螺仪偏置向量，将平均角速度向量 mean_gyr 赋值给 init_state.bg，用于在IMU初始化过程中设置初始状态，以便为后续的滤波过程提供一个合理的初始估计。
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state); // 将计算得到的初始状态 init_state 赋值给卡尔曼滤波器的状态估计 kf_state，以便在IMU初始化过程中设置初始状态，包括重力加速度、陀螺仪偏置和加速度计偏置等参数，为后续的滤波过程提供一个合理的初始估计。

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P(); // 获取当前卡尔曼滤波器的状态协方差矩阵 init_P，通常用于在IMU初始化过程中设置初始状态的协方差，以便为后续的滤波过程提供一个合理的初始估计，并反映对初始状态的不确定性。
  init_P.setIdentity();  //将矩阵初始化为单位矩阵
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out) // 点云去畸变方法，接受当前测量数据 meas、卡尔曼滤波器对象 kf_state 和一个点云对象 pcl_out，通常用于根据IMU的状态估计对点云数据进行去畸变处理，以消除由于传感器运动引起的点云畸变，从而提高点云数据的精度和可靠性。
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;  
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();

  double pcl_beg_time = meas.lidar_beg_time;
  double pcl_end_time = meas.lidar_end_time;

    // 由于MARSIM仿真环境中LiDAR数据的时间戳可能存在不连续或不准确的情况，因此在处理MARSIM仿真数据时，点云的起始时间 pcl_beg_time 被设置为上一次LiDAR数据的结束时间 last_lidar_end_time_，而点云的结束时间 pcl_end_time 则被设置为当前测量数据 meas 的起始时间 meas.lidar_beg_time。这种处理方式可以确保在MARSIM仿真环境中，点云数据的时间戳与IMU数据的时间戳保持一致，从而实现更准确的传感器融合和去畸变处理。
    if (lidar_type == MARSIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    }

    /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar); 
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
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

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
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

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;

  if(lidar_type != MARSIM){
      auto it_pcl = pcl_out.points.end() - 1;
      for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
      {
          auto head = it_kp - 1;
          auto tail = it_kp;
          R_imu<<MAT_FROM_ARRAY(head->rot);
          // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
          vel_imu<<VEC_FROM_ARRAY(head->vel);
          pos_imu<<VEC_FROM_ARRAY(head->pos);
          acc_imu<<VEC_FROM_ARRAY(tail->acc);
          angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

          for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
          {
              dt = it_pcl->curvature / double(1000) - head->offset_time;

              /* Transform to the 'end' frame, using only the rotation
               * Note: Compensation direction is INVERSE of Frame's moving direction
               * So if we want to compensate a point at timestamp-i to the frame-e
               * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
              M3D R_i(R_imu * Exp(angvel_avr, dt));

              V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
              V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
              V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

              // save Undistorted points and their rotation
              it_pcl->x = P_compensate(0);
              it_pcl->y = P_compensate(1);
              it_pcl->z = P_compensate(2);

              if (it_pcl == pcl_out.points.begin()) break;
          }
      }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
