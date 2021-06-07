#pragma once
#include <eigen3/Eigen/Core>
#ifndef EXTENDED_KALMAN_FILTER_
#define EXTENDED_KALMAN_FILTER_
#include <eigen3/Eigen/Dense>

using namespace Eigen;
class ExtendedKalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    void Initialization(const Vector3d &gyro_rate_noise_in, const Vector3d &gyro_bias_noise_in,
                        const Vector3d &acc_rate_noise_in, const MatrixXd &state_covariance_matrix_in);

    void Prediction(const Vector3d &omega_body_measurement,
                    const double &sample_interval);                // prediction module
    void MeasurementUpdate(const Vector3d &acc_body_measurement);  // update module
    void QuaternionUpdate(const Vector3d &omega_body_measurement); // update module

    bool is_initialized_; // flag of initialization

    Vector3d gyro_rate_noise_;                     // gyroscope rate noise
    Vector3d gyro_bias_noise_;                     // gyroscope bias drving noise (random walking)
    Vector3d acc_rate_noise_;                      // accelerator rate noise
    VectorXd state_;                               // ekf state vector
    VectorXd state_correction_;                    // correction of state vector
    Vector3d gyro_bias_estimation_;                // estimation of gyroscope bias
    Vector3d angular_velocity_estimation_in_body_; // estimation of angular velocity in body frame
    Quaterniond quaternion_body_in_ground_;        // rotation from ground to body (quaternion)
    Matrix3d rotation_ground_in_body_;             // rotation from body to ground (rotation matrix)
    MatrixXd continuous_state_transition_matrix_;  // state transition matrix (continuous time system)
    MatrixXd state_covariance_matrix_;             // state covariance matrix
    MatrixXd process_noise_covariance_matrix_;     // process noise covariance matrix (discrete time system)
    MatrixXd measurement_matrix_;                  // measurement matrix
    MatrixXd measure_noise_covariance_matrix_;     // measurement noise covariance matrix
};

#endif