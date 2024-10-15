/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file  hsrb_hw-test.cpp
/// @brief HSRBHW test
#include "mocks.hpp"

namespace {

hardware_interface::ComponentInfo CreateJointInfo(const std::string& motor_id) {
  hardware_interface::ComponentInfo info;
  info.name = "test_joint";
  info.parameters = {{"motor_id", "1"},         {"control_type", "ActiveJoint"}, {"drive_mode", "3"},
                     {"reduction", "2.0"},      {"velocity_limit", "5.0"},       {"gear_ratio", "6.0"},
                     {"torque_constant", "7.0"}};
  return info;
}

hardware_interface::HardwareInfo CreateTestInfo() {
  hardware_interface::ComponentInfo joint;
  joint.name = "test_joint";
  joint.parameters = {{"motor_id", "1"},         {"control_type", "ActiveJoint"}, {"drive_mode", "3"},
                      {"reduction", "2.0"},      {"velocity_limit", "5.0"},       {"gear_ratio", "6.0"},
                      {"torque_constant", "7.0"}};

  hardware_interface::ComponentInfo gripper;
  gripper.name = "test_gripper";
  gripper.parameters = {{"motor_id", "2"},          {"control_type", "Gripper"},   {"drive_mode", "3"},
                        {"reduction", "2.0"},       {"velocity_limit", "5.0"},     {"gear_ratio", "6.0"},
                        {"torque_constant", "7.0"}, {"left_spring_joint", "left"}, {"right_spring_joint", "right"}};

  hardware_interface::HardwareInfo info;
  info.joints.push_back(joint);
  info.joints.push_back(gripper);
  return info;
}

}  // namespace

namespace hsrb_robot_hardware {

TEST(HsrbHwTest, OnInitSuccess) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);

  EXPECT_TRUE(hardware.network());

  EXPECT_TRUE(hardware.protocol());
  EXPECT_EQ(hardware.protocol()->network, hardware.network());

  ASSERT_EQ(hardware.active_joints().size(), 2);

  auto first_joint_mock = std::dynamic_pointer_cast<ActiveJointMock>(hardware.active_joints()[0]);
  EXPECT_EQ(first_joint_mock->params.joint_name, "test_joint");

  auto first_comm_mock = std::dynamic_pointer_cast<JointCommunicationMock>(first_joint_mock->comm);
  EXPECT_EQ(first_comm_mock->act_id, 1);
  EXPECT_EQ(first_comm_mock->protocol, hardware.protocol());

  auto second_joint_mock = std::dynamic_pointer_cast<GripperActiveJointMock>(hardware.active_joints()[1]);
  EXPECT_EQ(second_joint_mock->params.joint_name, "test_gripper");
  EXPECT_EQ(second_joint_mock->gripper_params.left_spring_joint, "left");

  auto second_comm_mock = std::dynamic_pointer_cast<GripperCommunicationMock>(second_joint_mock->comm);
  EXPECT_EQ(second_comm_mock->act_id, 2);
  EXPECT_EQ(second_comm_mock->protocol, hardware.protocol());
}

TEST(HsrbHwTest, OnInitWithEmptyInfo) {
  HsrbHwTestTarget<MockNetworkError, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(hardware_interface::HardwareInfo()), CallbackReturn::ERROR);

  EXPECT_FALSE(hardware.network());
}

TEST(HsrbHwTest, OnInitWithInvalidInfo) {
  auto info = CreateTestInfo();
  info.joints.front().parameters.clear();

  HsrbHwTestTarget<MockNetworkError, MockProtocolReadHashSuccess> hardware;
  EXPECT_DEATH(hardware.on_init(info), "");
}

TEST(HsrbHwTest, OnInitWithNetworkError) {
  HsrbHwTestTarget<MockNetworkError, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::ERROR);

  EXPECT_TRUE(hardware.network());
  EXPECT_FALSE(hardware.protocol());
}

TEST(HsrbHwTest, OnInitWithControlTableError) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashError> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::ERROR);

  EXPECT_TRUE(hardware.network());
  EXPECT_TRUE(hardware.protocol());
}

TEST(HsrbHwTest, ExportStateInterfaces) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);

  auto interfaces = hardware.export_state_interfaces();
  EXPECT_EQ(interfaces.size(), 29);  // Normal joint 11, grippa 18
}

TEST(HsrbHwTest, ExportCommandInterfaces) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);

  auto interfaces = hardware.export_command_interfaces();
  EXPECT_EQ(interfaces.size(), 8);  // Normal joints 3, grippers 5
}

TEST(HsrbHwTest, OnActivateDeactivate) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  EXPECT_CALL(*std::dynamic_pointer_cast<ActiveJointMock>(hardware.active_joints()[0]), start()).Times(1);
  EXPECT_CALL(*std::dynamic_pointer_cast<GripperActiveJointMock>(hardware.active_joints()[1]), start()).Times(1);
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  EXPECT_CALL(*std::dynamic_pointer_cast<ActiveJointMock>(hardware.active_joints()[0]), stop()).Times(1);
  EXPECT_CALL(*std::dynamic_pointer_cast<GripperActiveJointMock>(hardware.active_joints()[1]), stop()).Times(1);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(HsrbHwTest, Read) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  EXPECT_CALL(*std::dynamic_pointer_cast<ActiveJointMock>(hardware.active_joints()[0]), read()).Times(1);
  EXPECT_CALL(*std::dynamic_pointer_cast<GripperActiveJointMock>(hardware.active_joints()[1]), read()).Times(1);
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

TEST(HsrbHwTest, Write) {
  HsrbHwTestTarget<MockNetworkSuccess, MockProtocolReadHashSuccess> hardware;
  EXPECT_EQ(hardware.on_init(CreateTestInfo()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  EXPECT_CALL(*std::dynamic_pointer_cast<ActiveJointMock>(hardware.active_joints()[0]), write()).Times(1);
  EXPECT_CALL(*std::dynamic_pointer_cast<GripperActiveJointMock>(hardware.active_joints()[1]), write()).Times(1);
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.write(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

}  // namespace hsrb_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}
