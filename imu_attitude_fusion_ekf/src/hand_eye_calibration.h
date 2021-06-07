#pragma once
#include <bits/stdint-intn.h>
#include <opencv2/core/matx.hpp>
#ifndef HAND_EYE_CALIBRATION_
#define HAND_EYE_CALIBRATION_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
//#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;
class HandEyeCalibrator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  struct Pose {
    int64_t timestamp;
    Vector3d translation;
    Quaterniond quaternion;
  };

  struct Imu {
    int64_t timestamp;
    Vector3d gyro;
    Vector3d acc;
  };

  bool ReadCameraPoseFromTxt(const string &filename);
  bool ReadHandPoseFromTxt(const string &filename);
  bool ReadImuDataFromTxt(const string &filename);
  bool SaveTxtFile(const cv::String &filename, const Pose &quaterniond);
  bool SaveTxtFile(const cv::String &filename, const cv::Mat &rodrigues_vec);
  void MatchDataTimestamp();
  void AngularVelocityPureIntegral(const int64_t &timestamp1,
                                   const int64_t &timestamp2);
  bool HandEyeCalibrate();
  void QuaternionToRodrigues(const Quaterniond &quaternion_input,
                             cv::Mat &rodrigues_output);

private:
  vector<Imu> imu_data_;
  Pose camera_pose_; // in camera frame
  vector<Pose> camera_poses_;
  Pose hand_pose_; // in ground frame
  Quaterniond camera_rot_ij_;
  Quaterniond hand_rot_ij_;
  vector<Pose> hand_poses_;
  vector<Pose> hand_poses_matched_;
  vector<cv::Mat> hand_rot_rodrigues_vecs_;
  vector<cv::Mat> camera_rot_rodrigues_vecs_;
};

#endif