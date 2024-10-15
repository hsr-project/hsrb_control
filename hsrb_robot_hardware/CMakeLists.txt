cmake_minimum_required(VERSION 3.5)
project(hsrb_robot_hardware)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(hsrb_servomotor_protocol REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(transmission_interface REQUIRED)

add_library(hsrb_joint SHARED
  src/${PROJECT_NAME}/hsrb_hw_joint.cpp
  src/${PROJECT_NAME}/joint_communication.cpp
  src/${PROJECT_NAME}/joint_parameters.cpp
)
ament_target_dependencies(hsrb_joint hardware_interface hsrb_servomotor_protocol rclcpp transmission_interface)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/hsrb_hw.cpp)
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp hardware_interface hsrb_servomotor_protocol rclcpp tmc_utils)
target_link_libraries(${PROJECT_NAME} hsrb_joint)

pluginlib_export_plugin_description_file(hardware_interface hsrb_robot_hardware.xml)

install(TARGETS
  ${PROJECT_NAME}
  hsrb_joint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gmock(test_joint_communication test/joint_communication-test.cpp
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_link_libraries(test_joint_communication ${PROJECT_NAME})

  ament_add_gmock(test_hw_joint test/hsrb_hw_joint-test.cpp)
  target_link_libraries(test_hw_joint hsrb_joint)
  ament_target_dependencies(test_hw_joint tmc_utils)

  ament_add_gmock(test_hsrb_hw test/hsrb_hw-test.cpp)
  target_link_libraries(test_hsrb_hw hsrb_joint)
  ament_target_dependencies(test_hsrb_hw tmc_utils)

  ament_add_gtest(test_joint_parameters test/joint_parameters-test.cpp)
  target_link_libraries(test_joint_parameters hsrb_joint)
endif()

ament_export_libraries(${PROJECT_NAME} hsrb_joint)
ament_export_dependencies(hsrb_servomotor_protocol)

ament_package()