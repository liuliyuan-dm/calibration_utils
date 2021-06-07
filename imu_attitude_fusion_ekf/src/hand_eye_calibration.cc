#include "hand_eye_calibration.h"
#include <bits/stdint-intn.h>
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <string>

bool HandEyeCalibrator::ReadImuDataFromTxt(const string &filename) {
  ifstream fin(filename);
  if (!fin) {
    cerr << "Imu data read failed!" << endl;
    return false;
  } else {
    while (!fin.eof()) {
      // p = [gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,timestamp(s)]
      VectorXd dat(6, 1);
      for (int i = 0; i < dat.size(); i++) {
        fin >> dat[i];
      }
      Imu imu_data;
      fin >> imu_data.timestamp;
      imu_data.gyro << dat[0], dat[1], dat[2];
      imu_data.acc << dat[3], dat[4], dat[5];
      imu_data_.emplace_back(move(imu_data));
    }
    cout << "Imu data load succeed!" << endl;
    return true;
  }
}

bool HandEyeCalibrator::ReadCameraPoseFromTxt(const string &filename) {
  ifstream fin(filename);
  if (!fin) {
    cerr << "Camera pose data read failed!" << endl;
    return false;
  } else {
    while (!fin.eof()) {
      // p = [q_w,q_x,q_y,q_z,timestamp(s)]
      VectorXd dat(4, 1);
      for (int i = 0; i < dat.size(); i++) {
        fin >> dat[i];
      }
      camera_pose_.quaternion.w() = dat[0];
      camera_pose_.quaternion.x() = dat[1];
      camera_pose_.quaternion.y() = dat[2];
      camera_pose_.quaternion.z() = dat[3]; // quaternion in camera frame
      fin >> camera_pose_.timestamp;
      camera_poses_.emplace_back(move(camera_pose_));
    }
    cout << "Camera pose data load succeed!" << endl;
    return true;
  }
}

bool HandEyeCalibrator::ReadHandPoseFromTxt(const string &filename) {
  ifstream fin(filename);
  if (!fin) {
    cerr << "Hand pose data read failed!" << endl;
    return false;
  } else {
    while (!fin.eof()) {
      // p = [q_w,q_x,q_y,q_z,timestamp(s)]
      VectorXd dat(4, 1);
      for (int i = 0; i < dat.size(); i++) {
        fin >> dat[i];
      }
      hand_pose_.quaternion.w() = dat[0];
      hand_pose_.quaternion.x() = dat[1];
      hand_pose_.quaternion.y() = dat[2];
      hand_pose_.quaternion.z() = dat[3]; // quaternion in ground frame.
      fin >> hand_pose_.timestamp;
      hand_poses_.emplace_back(std::move(hand_pose_));
    }
    cout << "Hand pose data load succeed!" << endl;
    return true;
  }
}

bool HandEyeCalibrator::SaveTxtFile(const cv::String &filename,
                                    const Pose &pose) {
  std::fstream output_txt;
  output_txt.open(filename + ".txt", std::ios::out | std::ios::app);
  output_txt << pose.quaternion.w() << " " << pose.quaternion.x() << " "
             << pose.quaternion.y() << " " << pose.quaternion.z() << " "
             << pose.timestamp << endl;
  output_txt.close();
  return (!output_txt.is_open());
}

bool HandEyeCalibrator::SaveTxtFile(const cv::String &filename,
                                    const cv::Mat &rodrigues_vec) {
  std::fstream output_txt;
  output_txt.open(filename + ".txt", std::ios::out | std::ios::app);
  output_txt << rodrigues_vec.at<double>(0) << " "
             << rodrigues_vec.at<double>(1) << " "
             << rodrigues_vec.at<double>(2) << endl;
  output_txt.close();
  return (!output_txt.is_open());
}

void HandEyeCalibrator::MatchDataTimestamp() {
  Pose hand_pose_tmp;
  for (auto c = 0; c < camera_poses_.size(); c++) {
    for (auto h = 0; h < hand_poses_.size(); h++) {
      if (hand_poses_[h].timestamp <= camera_poses_[c].timestamp &&
          hand_poses_[h + 1].timestamp > camera_poses_[c].timestamp) {
        if ((camera_poses_[c].timestamp - hand_poses_[h].timestamp) <=
            (hand_poses_[h + 1].timestamp - camera_poses_[c].timestamp))
          hand_pose_tmp = hand_poses_[h];
        else
          hand_pose_tmp = hand_poses_[h + 1];
        SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                    "key_frame_rotation_quaternion_in_ground_frame",
                    hand_pose_tmp);
        hand_poses_matched_.emplace_back(hand_pose_tmp);
      }
    }
  }
}

void HandEyeCalibrator::AngularVelocityPureIntegral(const int64_t &timestamp1,
                                                    const int64_t &timestamp2) {
  Quaterniond q_base, q_delta;
  q_base.setIdentity();

  for (auto i = 0; i < imu_data_.size() - 1; i++) {
    if (imu_data_[i].timestamp >= timestamp1 &&
        imu_data_[i].timestamp < timestamp2) {
      // q(k+1) = q(k)*[1,1/2*dt*w]
      q_delta.w() = 1.0;
      q_delta.x() = 0.5 *
                    (imu_data_[i + 1].timestamp - imu_data_[i].timestamp) /
                    1000000000.0 * imu_data_[i].gyro[0];
      q_delta.y() = 0.5 *
                    (imu_data_[i + 1].timestamp - imu_data_[i].timestamp) /
                    1000000000.0 * imu_data_[i].gyro[1];
      q_delta.z() = 0.5 *
                    (imu_data_[i + 1].timestamp - imu_data_[i].timestamp) /
                    1000000000.0 * imu_data_[i].gyro[2];
      q_base = q_base * q_delta;
      q_base.normalize();
    }
  }
  hand_rot_ij_ = q_base;
}

bool HandEyeCalibrator::HandEyeCalibrate() {

  // Step1: calculate q_ci_ci+1 and q_hi_hi+1 and transfer quaternion form to
  // rodrigues vector, and solve the least squares problem.
  // Reference:
  // F. Park, B. Martin, "Robot Sensor Calibration: Solving AX = XB on the
  // Euclidean Group."
  // alpha = X*beta

  if (camera_poses_.size() < 3) {
    cout << "The pose could not be calculated less than 3 point supplied!"
         << endl;
    return false;
  }

  cv::Mat camera_rot_rodrigues_vec;
  cv::Mat hand_rot_rodrigues_vec;
  Quaterniond hand_q_base;
  cv::Mat M = cv::Mat::zeros(3, 3, CV_64FC1);

  for (auto i = 0; i < camera_poses_.size(); i++) {
    for (auto j = i + 1;
         (j < camera_poses_.size()) && (j < i + camera_poses_.size()); j++) {
      AngularVelocityPureIntegral(hand_poses_matched_[i].timestamp,
                                  hand_poses_matched_[j].timestamp);
      camera_rot_ij_ =
          camera_poses_[i].quaternion * camera_poses_[j].quaternion.inverse();
      QuaternionToRodrigues(camera_rot_ij_, camera_rot_rodrigues_vec);
      QuaternionToRodrigues(hand_rot_ij_, hand_rot_rodrigues_vec);
      // M += camera_rot_rodrigues_vec * hand_rot_rodrigues_vec.t();
      M += hand_rot_rodrigues_vec * camera_rot_rodrigues_vec.t();
      SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                  "key_frame_rotation_vecs_in_ground_frame",
                  hand_rot_rodrigues_vec);
      SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                  "key_frame_rotation_vecs_in_camera_frame",
                  camera_rot_rodrigues_vec);
      camera_rot_rodrigues_vecs_.emplace_back(move(camera_rot_rodrigues_vec));
      hand_rot_rodrigues_vecs_.emplace_back(move(hand_rot_rodrigues_vec));
    }
  }
  /*
    for (auto i = 0; i < camera_poses_.size(); i++) {
      for (auto j = i + 1; j < camera_poses_.size(); j++) {
        QuaternionToRodrigues(camera_poses_[i].quaternion *
                                  camera_poses_[j].quaternion.inverse(),
                              camera_rot_rodrigues_vec);
        QuaternionToRodrigues(hand_poses_matched_[i].quaternion.inverse() *
                                  hand_poses_matched_[j].quaternion,
                              hand_rot_rodrigues_vec);
        // M += camera_rot_rodrigues_vec * hand_rot_rodrigues_vec.t();
        M += hand_rot_rodrigues_vec * camera_rot_rodrigues_vec.t();
        SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                    "key_frame_rotation_vecs_in_ground_frame",
                    hand_rot_rodrigues_vec);
        SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                    "key_frame_rotation_vecs_in_camera_frame",
                    camera_rot_rodrigues_vec);
        camera_rot_rodrigues_vecs_.emplace_back(move(camera_rot_rodrigues_vec));
        hand_rot_rodrigues_vecs_.emplace_back(move(hand_rot_rodrigues_vec));
      }
    }*/

  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(M.t() * M, eigenvalues, eigenvectors);
  cv::Mat v = cv::Mat::zeros(3, 3, CV_64FC1);
  for (int i = 0; i < 3; i++) {
    v.at<double>(i, i) = 1.0 / sqrt(eigenvalues.at<double>(i, 0));
  }
  cv::Mat R = eigenvectors.t() * v * eigenvectors * M.t(); // R_hand_camera
  cout << "R is : " << R << endl;
  cout << "size of rodrigues vec: " << camera_rot_rodrigues_vecs_.size() << " "
       << hand_rot_rodrigues_vecs_.size() << endl;

  Eigen::Matrix3d rot_mat_camera_in_hand_eigen;
  cv::cv2eigen(R, rot_mat_camera_in_hand_eigen);
  Quaterniond quat_camera_in_hand(rot_mat_camera_in_hand_eigen);
  cout << "quat_camera_in_hand is: " << quat_camera_in_hand.w() << " "
       << quat_camera_in_hand.x() << " " << quat_camera_in_hand.y() << " "
       << quat_camera_in_hand.z() << endl;
  cv::Mat vec;
  cv::Rodrigues(R, vec);
  cout << vec * 57.3 << " " << cv::norm(vec) * 57.3 << endl;
  Quaterniond quat_lidar_in_camera, quat_lidar_in_hand;
  quat_lidar_in_camera.w() = 0.50511824618134837;
  quat_lidar_in_camera.x() = 0.49247417699932516;
  quat_lidar_in_camera.y() = 0.50661314724469475;
  quat_lidar_in_camera.z() = -0.49564892958861662;

  quat_lidar_in_hand = quat_camera_in_hand * quat_lidar_in_camera;
  cout << "quat_lidar_in_hand is: " << quat_lidar_in_hand.w() << " "
       << quat_lidar_in_hand.x() << " " << quat_lidar_in_hand.y() << " "
       << quat_lidar_in_hand.z() << endl;

  // evaluation
  cv::Mat left, right,
      error; // error = hand_rodrigues_vec*R-R*camera_rodrigues_vec
  double error_sum = 0.0, error_max = 0.0, error_mean = 0.0;
  int64_t count = 0;
  for (auto i = 0; i < camera_rot_rodrigues_vecs_.size(); i++) {
    left = hand_rot_rodrigues_vecs_[i];
    right = R * camera_rot_rodrigues_vecs_[i];
    error = left - right;
    // cout << i << ": left: " << left << "; right: " << right
    //     << "; error: " << left - right
    //     << "; error norm: " << norm(left - right) * 57.3 << endl; //??
    SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                "key_frame_left_vec",
                left);
    SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                "key_frame_right_vec",
                right);
    SaveTxtFile("/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
                "key_frame_error_vec",
                error);
    // cout << i << ": " << norm(error) * 57.3 << endl;
    if (norm(error) > error_max)
      error_max = norm(error);
    error_sum += norm(error);
    count++;
  }
  error_mean = error_sum / count;
  cout << "error max: " << error_max * 57.3 << " " << error_mean * 57.3
       << endl; // 60 keyframes: 1540 19.3672 5.83855 quat_camera_in_hand is:
                // 0.00817165 -0.0180952 0.720028 -0.693661  179.077
                // 20 keyframes: 136  13.2079 5.33929 quat_camera_in_hand is:
                // -0.0222369 -0.0207332 0.708179 -0.705378 177.465
                // 40 keyframes: 528 14.5473 5.53727 quat_camera_in_hand is:
                // -0.0075187 -0.0163929 0.714734 -0.699164 179.152
                // 10 keyframes: 55 13.8311 6.05812 quat_camera_in_hand is:
                // 0.0188352 -0.0468404 0.721351 -0.690728 177.855
                // 30 keyframes: 325 18.8442 6.21869 quat_camera_in_hand is:
                // 0.010822 -0.027769 0.720188 -0.693138 178.773
                // 100 keyframes: 4005 19.7307 5.77799 quat_camera_in_hand is:
                // 0.00318145 -0.0182104 0.721864 -0.691788 179.649
}

void HandEyeCalibrator::QuaternionToRodrigues(
    const Quaterniond &quaternion_input, cv::Mat &rodrigues_output) {
  cv::Mat rotationMat;
  cv::eigen2cv(quaternion_input.toRotationMatrix(), rotationMat);
  cv::Rodrigues(rotationMat, rodrigues_output);
}

int main(int argc, char **argv) {
  HandEyeCalibrator hand_eye_calibrator;
  hand_eye_calibrator.ReadImuDataFromTxt(
      "/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
      "imudata.txt");
  hand_eye_calibrator.ReadCameraPoseFromTxt(
      "/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
      "key_frame_rotation_quaternion_in_camera_frame.txt"); // q_base_hand
  hand_eye_calibrator.ReadHandPoseFromTxt(
      "/home/liyuanliu/code/project/cam_imu_cali/data/imu/"
      "qgb_estimate.txt"); // q_world_eye
  hand_eye_calibrator.MatchDataTimestamp();
  hand_eye_calibrator.HandEyeCalibrate();
}
