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


add_library(imu_nav
    src/ideal_imu_simulator.cpp
)

target_link_libraries(imu_nav PUBLIC Eigen3::Eigen)


target_include_directories(imu_nav
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(imu_nav_app src/main.cpp)


target_link_libraries(imu_nav_app PRIVATE imu_nav)


include(CTest)

if(BUILD_TESTING)

    find_package(GTest CONFIG REQUIRED)

    add_executable(imu_nav_tests
        tests/ideal_imu_simulator_test.cpp
    )

    target_link_libraries(imu_nav_tests
        PRIVATE
        imu_nav
        GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(imu_nav_tests)
endif()
