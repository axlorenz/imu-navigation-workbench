cmake_minimum_required(VERSION 3.25)

project(
    imu_navigation_workbench
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(Eigen3 CONFIG REQUIRED)

add_executable(imu_nav_app src/main.cpp)

target_link_libraries(imu_nav_app PRIVATE Eigen3::Eigen)
target_include_directories(imu_nav_app 
    PRIVATE 
    ${PROJECT_SOURCE_DIR}/include
)