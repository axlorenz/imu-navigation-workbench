#include "imu_nav/ideal_imu_simulator.hpp"

#include <cstddef>
#include <algorithm>
#include <Eigen/Geometry>

namespace
{

constexpr double standard_gravity_m_s2 = 9.80665;

const Eigen::Vector3d standard_gravity_n_m_s2{
    0.0,
    0.0,
    standard_gravity_m_s2
};

const Eigen::Vector3d magnetic_field_n_t{
    50.0e-6,
    0.0,
    0.0
};

}

namespace imu_nav
{
    std::vector<SimulatedImuSample> simulate_ideal_z_rotation(const IdealImuSimulationConfig & cfg)
    {
        
        const auto total_duration_s = cfg.initial_stationary_duration_s + cfg.rotation_duration_s + cfg.final_stationary_duration_s;
        const auto dt_s = 1.0 / cfg.sample_rate_hz;
        
        const std::size_t sample_count = static_cast<std::size_t>(cfg.sample_rate_hz * total_duration_s);
        

        std::vector<SimulatedImuSample> samples{};

        samples.reserve(sample_count);

        for(std::size_t index = 0U; index < sample_count;++index)
        {
            SimulatedImuSample sim_imu_sample{};
            
            const auto time_s = index * dt_s;
            sim_imu_sample.imu_sample.timestamp_s = time_s; 

            const auto rotation_time_s = std::clamp((time_s - cfg.initial_stationary_duration_s), 0.0, cfg.rotation_duration_s);

            
            const auto rotation_angle_rad = cfg.angular_velocity_rad_s *  rotation_time_s;
            
            const auto true_orientation_q = Eigen::Quaterniond{Eigen::AngleAxisd{ rotation_angle_rad, Eigen::Vector3d::UnitZ()}};
            sim_imu_sample.true_orientation = true_orientation_q;
            
            sim_imu_sample.imu_sample.accelerometer_m_s2 =
                true_orientation_q.conjugate() * standard_gravity_n_m_s2;

            sim_imu_sample.imu_sample.magnetometer_t = true_orientation_q.conjugate() * magnetic_field_n_t;


            const double rotation_end_s = cfg.initial_stationary_duration_s + cfg.rotation_duration_s;

            const bool is_rotating = 
                time_s >= cfg.initial_stationary_duration_s
                && time_s < rotation_end_s;
            
            if(is_rotating)
            {
                sim_imu_sample.imu_sample.gyroscope_rad_s =
                Eigen::Vector3d{0.0, 0.0, cfg.angular_velocity_rad_s};
            }

            samples.push_back(sim_imu_sample);
        }

        return samples;
    }
}
