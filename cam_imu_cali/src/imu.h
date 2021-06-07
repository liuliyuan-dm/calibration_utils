#ifndef IMU_
#define IMU_
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <vector>
#include <eigen3/Eigen/Geometry>

// 以下数据均认为经过矫正，已经补偿了恒偏差bias, scale,misalignment for both gyro and acc.
// Limitations: 假设了body整体线加速度为0，故仅受到重力加速度影响（0,0，-g），对acc的bias未做估计
// 由于运动状态缺少航向角度的观测，因此仅在静止状态下会对bw_z做估计，运动时刻不做更新（无法更新）
// nr，nw,na等噪声可以尝试通过allen方差的评估手段获得

using namespace std;
using namespace Eigen;
/* wb_m = K*wb + bw0
   K    = [sx mxy mxz
           myx sy myz  
           mzx mzy sz]
   bw0   = [bx0,by0,bz0]
*/
class GyroCompensation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Matrix3d gyro_scale_misalign_;
    Vector3d gyro_bias_stable_;
};

class AccCompensation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Matrix3d acc_scale_misalign_;
    Vector3d acc_bias_stable_;
};

class Timestamp
{
public:
    //double timestamp_;
    vector<int64_t> timestamps_;
};

class Gyro
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    GyroCompensation gyro_comp_;               // gyroscope intrincisc compensation (scale, bias, misalignment), got by offline calibration
    vector<Vector3d> omega_body_measurements_; // angular velocity measurement in body frame
    //vector<Vector3d> omega_body_estimations_;  // angular velocity estimation in body frame
    //Vector3d bw_e_;                            // bias angular velocity estimation
    //Vector3d nr_;                              // angular velocity noise, gaussian distribution, wb_m-b ~N(0,nr)
    //Vector3d nw_;                              // angular velocity bias noise, bw~N(0,nw)
};

class Acc
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AccCompensation acc_comp_;               // accelerator intrincisc compensation (scale, bias, misalignment), got by offline calibration
    vector<Vector3d> acc_body_measurements_; // acc body measurement,这个归一化算，不考虑g的具体值
    //Vector3d na_;                            // acc noise, ab_m~N(0,na)
};

class Imu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    bool ReadDataFromTxt(string path, Imu &imu);
    // bool ReadDataFromRec(string path,IMU &imu);
    // bool ReadDataFromRosbag(string path,IMU &imu);

    Gyro gyro_;
    Acc acc_;
    Timestamp timestamp_;
};

#endif