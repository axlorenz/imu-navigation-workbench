
#include "imu_nav/imu_sample.hpp"

#include <Eigen/Core>
#include <iostream>


int main()
{

    const imu_nav::ImuSample sample {
        .timestamp_s = 2.0,
        .gyroscope_rad_s = {0.0, 0.0, 0.1571},
        .accelerometer_m_s2 = {0.0, 0.0, 9.81},
        .magnetometer_t = {20e-6, 0.0, -45e-6}
    };


    const Eigen::Vector3d gyroscope_rad_sec {0.0,0.0, 0.1571};



    std::cout << "Gyroscope [rad/s]: "
              << sample.gyroscope_rad_s.transpose() << '\n';

}