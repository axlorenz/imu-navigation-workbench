#include "imu_nav/ideal_imu_simulator.hpp"

#include <numbers>
#include <gtest/gtest.h>

namespace
{

constexpr double deg_to_rad =
    std::numbers::pi_v<double> / 180.0; 

const imu_nav::IdealImuSimulationConfig testcfg{
    .sample_rate_hz = 100.0,
    .initial_stationary_duration_s = 2.0,
    .rotation_duration_s = 10.0,
    .final_stationary_duration_s = 3.0,
    .angular_velocity_rad_s = 9.0 * deg_to_rad
};

constexpr auto near_error = 1e-10;

constexpr double standard_gravity_m_s2 = 9.80665;

}

TEST(IdealImuSimulatorTest, GyroValueCorrectInMovementAndInRest)
{

    const auto samples = imu_nav::simulate_ideal_z_rotation(testcfg);

    EXPECT_DOUBLE_EQ(samples.at(100).imu_sample.gyroscope_rad_s.z(), 0.0);
    EXPECT_DOUBLE_EQ(samples.at(700).imu_sample.gyroscope_rad_s.z(), testcfg.angular_velocity_rad_s);
    EXPECT_DOUBLE_EQ(samples.at(1200).imu_sample.gyroscope_rad_s.z(), 0.0);
}

TEST(IdealImuSimulatorTest, GyroChangesAtRotationBoundaries)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    // t = 1.99 s: unmittelbar vor Beginn
    EXPECT_DOUBLE_EQ(
        samples.at(199).imu_sample.gyroscope_rad_s.z(),
        0.0
    );

    // t = 2.00 s: Beginn der Rotation
    EXPECT_DOUBLE_EQ(
        samples.at(200).imu_sample.gyroscope_rad_s.z(),
        testcfg.angular_velocity_rad_s
    );

    // t = 11.99 s: letztes Sample während der Rotation
    EXPECT_DOUBLE_EQ(
        samples.at(1199).imu_sample.gyroscope_rad_s.z(),
        testcfg.angular_velocity_rad_s
    );

    // t = 12.00 s: Rotation beendet
    EXPECT_DOUBLE_EQ(
        samples.at(1200).imu_sample.gyroscope_rad_s.z(),
        0.0
    );
}

TEST(IdealImuSimulatorTest, GyroRotatesOnlyAroundZAxis)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    const auto& gyro =
        samples.at(700).imu_sample.gyroscope_rad_s;

    // X-Achse prüfen
    EXPECT_DOUBLE_EQ(gyro.x(), 0.0);

    // Y-Achse prüfen
    EXPECT_DOUBLE_EQ(gyro.y(), 0.0);

    // Z-Achse prüfen
    EXPECT_DOUBLE_EQ(gyro.z(), 9.0 * deg_to_rad);

}

TEST(IdealImuSimulatorTest, AccelerometerIsConstantDuringEntireSimulation)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    for (const auto& sample : samples)
    {
        EXPECT_NEAR(sample.imu_sample.accelerometer_m_s2.x(),0.0,near_error);
        EXPECT_NEAR(sample.imu_sample.accelerometer_m_s2.y(),0.0,near_error);
        EXPECT_NEAR(sample.imu_sample.accelerometer_m_s2.z(),standard_gravity_m_s2,near_error);
    }
}

TEST(IdealImuSimulatorTest, MagnetometerValueIsCorrectAtStart)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    const auto& magnetometer =
        samples.at(0).imu_sample.magnetometer_t;

    EXPECT_NEAR(magnetometer.x(), 50.0e-6, near_error);
    EXPECT_NEAR(magnetometer.y(), 0.0, near_error);
    EXPECT_NEAR(magnetometer.z(), 0.0, near_error);
}

TEST(IdealImuSimulatorTest, MagnetometerValueIsCorrectAfterNinetyDegreeRotation)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    const auto& magnetometer =
        samples.at(1200).imu_sample.magnetometer_t;

    EXPECT_NEAR(magnetometer.x(), 0.0, near_error);
    EXPECT_NEAR(magnetometer.y(), -50.0e-6, near_error);
    EXPECT_NEAR(magnetometer.z(), 0.0, near_error);
}

TEST(IdealImuSimulatorTest, MagnetometerMagnitudeIsConstantDuringEntireSimulation)
{
    const auto samples =
        imu_nav::simulate_ideal_z_rotation(testcfg);

    for (const auto& sample : samples)
    {
        EXPECT_NEAR(
            sample.imu_sample.magnetometer_t.norm(),
            50.0e-6,
            near_error
        );
    }
}