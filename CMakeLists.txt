cmake_minimum_required(VERSION 3.8)
project(t24e_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(include ${EIGEN3_INCLUDE_DIR} ${GTEST_INCLUDE_DIRS})
  ament_add_gtest(test_ekf src/testing/EKFTest.cpp src/EKF.cpp)
  target_include_directories(test_ekf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_ekf Eigen3::Eigen)
endif()

add_executable(state_estimator src/state_estimator.cpp src/EKF.cpp)
ament_target_dependencies(state_estimator rclcpp lart_msgs)

target_link_libraries(state_estimator Eigen3::Eigen)

install(TARGETS
  state_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
