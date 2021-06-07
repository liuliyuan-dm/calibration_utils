#include "imu.h"

bool Imu::ReadDataFromTxt(string path, Imu &imu)
{
    ifstream fin(path);
    if (!fin)
    {
        cerr << "Imu data is not found!" << endl;
        return false;
    }
    else
    {
        while (!fin.eof())
        {
            VectorXd dat(6, 1);
            int64_t timestamp_dat;
            for (int i = 0; i < dat.size(); i++)
            {
                fin >> dat[i];
            }
            fin >> timestamp_dat;
            Vector3d acc_body_measurement = Vector3d(dat[3], dat[4], dat[5]);
            acc_body_measurement.normalize();
            imu.acc_.acc_body_measurements_.push_back(acc_body_measurement);
            imu.gyro_.omega_body_measurements_.push_back(Vector3d(dat[0], dat[1], dat[2]));
            imu.timestamp_.timestamps_.push_back(timestamp_dat);
        }

        cout << "Data read succeed!" << endl;
        return true;
    }
}
