#include <Eigen/Core>


#include <iostream>


int main()
{
    const Eigen::Vector3d gyroscope_rad_sec {0.0,0.0, 0.1571};


    std::cout << "Gyroscope [rad/s]: "
              << gyroscope_rad_sec.transpose() << '\n';

}