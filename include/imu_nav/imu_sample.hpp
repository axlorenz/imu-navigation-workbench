#pragma once

#include <Eigen/Core>

namespace imu_nav
{

struct ImuSample
{
    double timestamp_s{};

    Eigen::Vector3d gyroscope_rad_s{};
    Eigen::Vector3d accelerometer_m_s2{};
    Eigen::Vector3d magnetometer_t{};
};


}
