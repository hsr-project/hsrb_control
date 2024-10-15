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
/// @brief Test of joint parameter structure
#include <gtest/gtest.h>

#include "../src/hsrb_robot_hardware/joint_parameters.hpp"

namespace {

hardware_interface::ComponentInfo CreateTestInfo() {
  hardware_interface::ComponentInfo info;
  info.name = "test_joint";
  info.parameters = {{"motor_id", "1"},         {"control_type", "ActiveJoint"}, {"drive_mode", "3"},
                     {"reduction", "2.0"},      {"velocity_limit", "5.0"},       {"gear_ratio", "6.0"},
                     {"torque_constant", "7.0"}};
  info.command_interfaces.resize(1);
  info.command_interfaces[0].name = "position";
  info.command_interfaces[0].min = "-10.0";
  info.command_interfaces[0].max = "11.0";
  return info;
}

}  // namespace

namespace hsrb_robot_hardware {

// Normal system for initialization
TEST(JointParametersTest, InitializeNormal) {
  auto info = CreateTestInfo();

  rclcpp::NodeOptions options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
  auto node = rclcpp::Node::make_shared("test_node", options);
  auto parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  parameter_descriptor.dynamic_typing = true;
  node->declare_parameter("position_offset.test_joint", 8.0, parameter_descriptor);

  auto params = JointParameters(node, info);
  EXPECT_EQ(params.joint_name, "test_joint");
  EXPECT_EQ(params.motor_id, 1);
  EXPECT_EQ(params.control_type, "ActiveJoint");
  EXPECT_EQ(params.default_drive_mode, 3);
  EXPECT_EQ(params.position_offset, 8.0);
  EXPECT_EQ(params.reduction, 2.0);
  EXPECT_EQ(params.position_min, -10.0);
  EXPECT_EQ(params.position_max, 11.0);
  EXPECT_EQ(params.velocity_limit, 5.0);
  EXPECT_EQ(params.motor_to_joint_gear_ratio, 6.0);
  EXPECT_EQ(params.torque_constant, 7.0);

  node->undeclare_parameter("position_offset.test_joint");
  params = JointParameters(node, info);
  EXPECT_EQ(params.position_offset, 0.0);
}

// Failure in the early stages
TEST(JointParametersTest, InitializeFailure) {
  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter("position_offset.test_joint", 8.0);

  auto valid_info = CreateTestInfo();
  for (const auto& param : valid_info.parameters) {
    auto info = CreateTestInfo();
    info.parameters.erase(param.first);

    EXPECT_DEATH(JointParameters(node, info), "");
  }
}

// Position_min/max of non -position control joints contains very large values
TEST(JointParametersTest, VelocityControlJoint) {
  auto info = CreateTestInfo();
  info.command_interfaces[0].name = "velocity";

  auto node = rclcpp::Node::make_shared("test_node");

  auto params = JointParameters(node, info);
  EXPECT_LT(params.position_min, -1e10);
  EXPECT_GT(params.position_max, 1e10);
}

// ISACTIVEJOINT test
TEST(JointParametersTest, IsActiveJoint) {
  JointParameters params;
  params.motor_id = 1;
  EXPECT_TRUE(params.IsActiveJoint());

  params.motor_id = 0;
  EXPECT_FALSE(params.IsActiveJoint());
}

// ISZERORESET test
TEST(JointParametersTest, IsZeroReset) {
  JointParameters params;
  params.control_type = "Wheel";
  EXPECT_TRUE(params.IsZeroReset());

  params.control_type = "ActiveJoint";
  EXPECT_FALSE(params.IsZeroReset());

  params.control_type = "Gripper";
  EXPECT_FALSE(params.IsZeroReset());
}

// Initialization of GripperJointParameters
TEST(GripperJointParametersTest, InitializeNormal) {
  hardware_interface::ComponentInfo info;
  info.parameters = {{"left_spring_joint", "left"}, {"right_spring_joint", "right"}};

  auto params = GripperJointParameters(info);
  EXPECT_EQ(params.left_spring_joint, "left");
  EXPECT_EQ(params.right_spring_joint, "right");
}

}  // namespace hsrb_robot_hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  ::testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}
