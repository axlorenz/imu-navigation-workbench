#pragma once

#include "imu_nav/imu_sample.hpp"

#include <Eigen/Geometry>

namespace imu_nav
{

struct SimulatedImuSample
{

    ImuSample imu_sample{};
    
    Eigen::Quaterniond true_orientation {1.0,0.0,0.0,0.0};


};


}
