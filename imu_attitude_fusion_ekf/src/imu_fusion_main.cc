#include "extended_kalman_filter.h"
#include "imu.h"

int main(int argc, char **argv)
{
        Imu imu;
        imu.ReadDataFromTxt("/home/liyuanliu/code/project/cam_imu_cali/data/imu/imudata.txt", imu);

        cout << "path" << endl;
        uint64_t N;
        N = min(imu.gyro_.omega_body_measurements_.size(),
                min(imu.acc_.acc_body_measurements_.size(),
                    imu.timestamp_.timestamps_.size()));

        ofstream fout_qe("/home/liyuanliu/code/project/cam_imu_cali/data/imu/qgb_estimate.txt");
        ofstream fout_we("/home/liyuanliu/code/project/cam_imu_cali/data/imu/wb_estimate.txt");
        ofstream fout_wm("/home/liyuanliu/code/project/cam_imu_cali/data/imu/wm_measure.txt");
        ofstream fout_be("/home/liyuanliu/code/project/cam_imu_cali/data/imu/bias_estimate.txt");

        ExtendedKalmanFilter ekf;
        const Vector3d kGyro_rate_noise(0.0001, 0.0001, 0.0003),
            kGyro_bias_noise(0.00001, 0.00001, 0.00003),
            kAcc_rate_noise(0.0001, 0.0001, 0.0001);
        const MatrixXd kCovariance_matrix = MatrixXd::Identity(6, 6) * 0.1;
        ekf.Initialization(kGyro_rate_noise, kGyro_bias_noise, kAcc_rate_noise, kCovariance_matrix);

        for (uint64_t i = 1; i < N; i++)
        {
                Vector3d omega_body_measurement = imu.gyro_.omega_body_measurements_[i];
                Vector3d acc_body_measurement = imu.acc_.acc_body_measurements_[i];
                double sample_interval = imu.timestamp_.timestamps_[i] - imu.timestamp_.timestamps_[i - 1];
                sample_interval = sample_interval / 1000000000;
                cout << sample_interval << endl;

                ekf.Prediction(omega_body_measurement, sample_interval);
                cout << "Predict succeed!" << endl;

                ekf.MeasurementUpdate(acc_body_measurement);
                cout << "Measurement update succeed!" << endl;

                ekf.QuaternionUpdate(omega_body_measurement);
                cout << "Quaternion update succeed!" << endl;

                // output
                fout_qe << ekf.quaternion_body_in_ground_.w() << " "
                        << ekf.quaternion_body_in_ground_.x() << " "
                        << ekf.quaternion_body_in_ground_.y() << " "
                        << ekf.quaternion_body_in_ground_.z() << " "
                        << imu.timestamp_.timestamps_[i] << endl;
                fout_wm << imu.gyro_.omega_body_measurements_[i] << " "
                        << imu.acc_.acc_body_measurements_[i] << " "
                        << imu.timestamp_.timestamps_[i] << endl;
                fout_we << "we: " << ekf.angular_velocity_estimation_in_body_[0] << " "
                        << ekf.angular_velocity_estimation_in_body_[1] << " "
                        << ekf.angular_velocity_estimation_in_body_[2] << endl;
                fout_be << ekf.gyro_bias_estimation_[0] << " "
                        << ekf.gyro_bias_estimation_[1] << " "
                        << ekf.gyro_bias_estimation_[2] << endl;
        }

        fout_qe.close();
        fout_we.close();
        fout_be.close();
        fout_wm.close();
}