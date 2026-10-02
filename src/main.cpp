#include "imu_nav/ideal_imu_simulator.hpp"

#include <Eigen/Core>
#include <iostream>
#include <format>
#include <numbers>

constexpr double rad_to_deg =
    180.0 / std::numbers::pi_v<double>;

int main()
{
    imu_nav::IdealImuSimulationConfig config{};
    const auto simulation_samples = imu_nav::simulate_ideal_z_rotation(config);

    const auto& specific_sample = simulation_samples.at(700);

    const auto rot_matrix = specific_sample.true_orientation.toRotationMatrix();

    const auto euler_zyx  = rot_matrix.canonicalEulerAngles(2,1,0);
    const double yaw_rad   = euler_zyx(0);
    const double pitch_rad = euler_zyx(1);
    const double roll_rad  = euler_zyx(2);

    std::cout << std::format("Sample count: {}\n",simulation_samples.size());
    std::cout << std::format("Timestamp: {:.0f} s\n",specific_sample.imu_sample.timestamp_s);
    std::cout << std::format("Roll: {:.0f} deg\n",roll_rad*rad_to_deg);
    std::cout << std::format("Pitch: {:.0f} deg\n",pitch_rad*rad_to_deg);
    std::cout << std::format("Yaw: {:.0f} deg\n",yaw_rad*rad_to_deg);


}
