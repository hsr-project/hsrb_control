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
/// Joint parameter
#ifndef HSRB_ROBOT_HARDWARE_JOINT_PARAMETERS_HPP_
#define HSRB_ROBOT_HARDWARE_JOINT_PARAMETERS_HPP_

#include <stdint.h>
#include <string>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hsrb_robot_hardware {

struct JointParameters {
  std::string joint_name;

  uint8_t motor_id;
  std::string control_type;
  uint8_t default_drive_mode;
  double position_offset;
  double reduction;
  double position_min;
  double position_max;
  double velocity_limit;
  double motor_to_joint_gear_ratio;
  double torque_constant;

  JointParameters() = default;
  JointParameters(const rclcpp::Node::SharedPtr& nh, const hardware_interface::ComponentInfo& info);

  bool IsActiveJoint() const;
  bool IsZeroReset() const;
};

struct GripperJointParameters {
  std::string left_spring_joint;
  std::string right_spring_joint;

  GripperJointParameters() = default;
  explicit GripperJointParameters(const hardware_interface::ComponentInfo& info);
};

}  // namespace hsrb_robot_hardware
#endif  // HSRB_ROBOT_HARDWARE_JOINT_PARAMETERS_HPP_
