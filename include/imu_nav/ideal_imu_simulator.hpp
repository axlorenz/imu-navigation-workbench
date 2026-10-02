#pragma once

#include "imu_nav/simulated_imu_sample.hpp"


#include <numbers>
#include <vector>

namespace imu_nav
{

    struct IdealImuSimulationConfig
    {
        double sample_rate_hz{100.0};
        double initial_stationary_duration_s{2.0};
        double rotation_duration_s{10.0};
        double final_stationary_duration_s{2.0};
        double angular_velocity_rad_s{std::numbers::pi_v<double> / 20.0};
    };

    std::vector<SimulatedImuSample> simulate_ideal_z_rotation(const IdealImuSimulationConfig& cfg);


}