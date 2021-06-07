#include "extended_kalman_filter.h"
//https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/naming/
//reference:https://blog.shipengx.com/archives/d3b96bd5.html

void ExtendedKalmanFilter::Initialization(const Vector3d &gyro_rate_noise_in, const Vector3d &gyro_bias_noise_in,
                                          const Vector3d &acc_rate_noise_in, const MatrixXd &state_covariance_matrix_in)
{
    gyro_rate_noise_ = gyro_rate_noise_in;
    gyro_bias_noise_ = gyro_bias_noise_in;
    acc_rate_noise_ = acc_rate_noise_in;
    state_covariance_matrix_ = state_covariance_matrix_in;

    state_.setZero(6, 1);
    state_correction_.setZero(6, 1);
    gyro_bias_estimation_.setZero(3, 1);
    angular_velocity_estimation_in_body_.setZero(3, 1);

    quaternion_body_in_ground_.setIdentity();
    rotation_ground_in_body_.setIdentity(3, 3);
    continuous_state_transition_matrix_.setZero(6, 6);
    process_noise_covariance_matrix_.setZero(6, 6);
    measurement_matrix_.setZero(3, 6);
    measure_noise_covariance_matrix_.setZero(3, 3);
    measure_noise_covariance_matrix_.diagonal() << acc_rate_noise_;

    is_initialized_ = true;
}

void ExtendedKalmanFilter::Prediction(const Vector3d &omega_body_measurement,
                                      const double &sample_interval)
{
    Vector3d angular_velocity_update_in_body, angular_velocity_average_in_body;
    Quaterniond angular_velocity_correction_in_body;
    MatrixXd identity_matrix = MatrixXd::Identity(6, 6);

    angular_velocity_update_in_body = omega_body_measurement - gyro_bias_estimation_;                                  // bu(k+1|k) = be(k|k), wu(k+1|k) = wm(k+1)-bu(k+1|k)
    angular_velocity_average_in_body = (angular_velocity_update_in_body + angular_velocity_estimation_in_body_) / 2.0; // wx = [wu(k+1|k)+we(k)]/2
    angular_velocity_correction_in_body = Quaterniond(1.0, 0.5 * angular_velocity_average_in_body[0] * sample_interval,
                                                      0.5 * angular_velocity_average_in_body[1] * sample_interval,
                                                      0.5 * angular_velocity_average_in_body[2] * sample_interval);

    angular_velocity_correction_in_body.normalize();
    quaternion_body_in_ground_ = quaternion_body_in_ground_ * angular_velocity_correction_in_body;
    quaternion_body_in_ground_.normalize();
    rotation_ground_in_body_ = quaternion_body_in_ground_.toRotationMatrix().transpose();

    continuous_state_transition_matrix_ << 0.0, angular_velocity_average_in_body[2], -angular_velocity_average_in_body[1], -1.0, 0.0, 0.0,
        -angular_velocity_average_in_body[2], 0.0, angular_velocity_average_in_body[0], 0.0, -1.0, 0.0,
        angular_velocity_average_in_body[1], -angular_velocity_average_in_body[0], 0.0, 0.0, 0.0, -1.0;

    MatrixXd discrete_state_transition_matrix = identity_matrix + continuous_state_transition_matrix_ * sample_interval; // state transition matrix(in discrete time system)

    process_noise_covariance_matrix_.diagonal()
        << gyro_rate_noise_.cwiseProduct(gyro_rate_noise_),
        gyro_bias_noise_.cwiseProduct(gyro_bias_noise_);
    process_noise_covariance_matrix_ = process_noise_covariance_matrix_ * sample_interval;

    state_covariance_matrix_ = discrete_state_transition_matrix * state_covariance_matrix_ * discrete_state_transition_matrix.transpose() +
                               process_noise_covariance_matrix_;
}

void ExtendedKalmanFilter::MeasurementUpdate(const Vector3d &acc_body_measurement)
{
    MatrixXd identity_matrix = MatrixXd::Identity(6, 6);
    Vector3d acc_body_residual, acc_body_predict;

    acc_body_predict << -rotation_ground_in_body_(0, 2), -rotation_ground_in_body_(1, 2), -rotation_ground_in_body_(2, 2);
    acc_body_residual = acc_body_measurement - acc_body_predict;

    measurement_matrix_ << 0.0, rotation_ground_in_body_(2, 2), -rotation_ground_in_body_(1, 2), 0.0, 0.0, 0.0,
        -rotation_ground_in_body_(2, 2), 0.0, rotation_ground_in_body_(0, 2), 0.0, 0.0, 0.0,
        rotation_ground_in_body_(1, 2), -rotation_ground_in_body_(0, 2), 0.0, 0.0, 0.0, 0.0;

    MatrixXd residual_covariance_matrix = measurement_matrix_ * state_covariance_matrix_ * measurement_matrix_.transpose() +
                                          measure_noise_covariance_matrix_; // convariance of the residual

    MatrixXd kalman_gain = state_covariance_matrix_ * measurement_matrix_.transpose() * residual_covariance_matrix.inverse(); // kalman gain

    state_correction_ = kalman_gain * acc_body_residual;

    state_covariance_matrix_ = (identity_matrix - kalman_gain * measurement_matrix_) * state_covariance_matrix_ *
                                   (identity_matrix - kalman_gain * measurement_matrix_).transpose() +
                               kalman_gain * measure_noise_covariance_matrix_ * kalman_gain.transpose(); // update模块实际上到这里已经求解完了 之后的是为了转换成四元数
}

void ExtendedKalmanFilter::QuaternionUpdate(const Vector3d &omega_body_measurement)
{
    Vector3d bias_estimate_correction, angle_axis_estimate_correction;
    bias_estimate_correction << state_correction_[3], state_correction_[4], state_correction_[5];
    angle_axis_estimate_correction << state_correction_[0] * 0.5, state_correction_[1] * 0.5, state_correction_[2] * 0.5;

    Quaterniond quaternion_body_in_ground_correction;
    double norm_scale;
    if (angle_axis_estimate_correction.squaredNorm() < 1.0)
    {
        norm_scale = sqrt(1.0 - angle_axis_estimate_correction.squaredNorm());
        quaternion_body_in_ground_correction = Quaterniond(1.0 * norm_scale,
                                                           angle_axis_estimate_correction[0],
                                                           angle_axis_estimate_correction[1],
                                                           angle_axis_estimate_correction[2]);
    }
    else
    {
        norm_scale = 1.0 / sqrt(1.0 + angle_axis_estimate_correction.squaredNorm());
        quaternion_body_in_ground_correction = Quaterniond(1.0 * norm_scale,
                                                           angle_axis_estimate_correction[0] * norm_scale,
                                                           angle_axis_estimate_correction[1] * norm_scale,
                                                           angle_axis_estimate_correction[2] * norm_scale);
    }

    quaternion_body_in_ground_ = quaternion_body_in_ground_ * quaternion_body_in_ground_correction;

    gyro_bias_estimation_ += bias_estimate_correction;

    angular_velocity_estimation_in_body_ = omega_body_measurement - gyro_bias_estimation_;
}