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

#include "joint_parameters.hpp"

#include <limits>
#include <memory>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <hsrb_servomotor_protocol/exxx_common.hpp>

namespace {
const uint8_t kInvalidActuatorID = 0;

template<typename TYPE>
TYPE ExtractParameter(const hardware_interface::ComponentInfo& info,
                       const std::string& name,
                       std::function<TYPE(const std::string&)> func) {
  const auto it = info.parameters.find(name);
  if (it != info.parameters.cend()) {
    return func(it->second);
  } else {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("JointParameters"),
                        "Cannot find " << name << " in joint parameter");
    exit(EXIT_FAILURE);
  }
}

std::string ExtractStrParameter(const hardware_interface::ComponentInfo& info,
                                const std::string& name) {
  return ExtractParameter<std::string>(info, name, [](const std::string& x)->std::string { return x; });
}

int32_t ExtractIntParameter(const hardware_interface::ComponentInfo& info,
                            const std::string& name) {
  return ExtractParameter<int>(info, name, [](const std::string& x)->int32_t { return std::stoi(x); });
}

double ExtractDoubleParameter(const hardware_interface::ComponentInfo& info,
                              const std::string& name) {
  return ExtractParameter<double>(info, name, [](const std::string& x)->double { return std::stod(x); });
}

}  // namespace

namespace hsrb_robot_hardware {

JointParameters::JointParameters(const rclcpp::Node::SharedPtr& nh, const hardware_interface::ComponentInfo& info) {
  joint_name = info.name;

  motor_id = ExtractIntParameter(info, "motor_id");
  control_type = ExtractStrParameter(info, "control_type");
  default_drive_mode = ExtractIntParameter(info, "drive_mode");
  reduction = ExtractDoubleParameter(info, "reduction");
  velocity_limit = ExtractDoubleParameter(info, "velocity_limit");
  motor_to_joint_gear_ratio = ExtractDoubleParameter(info, "gear_ratio");
  torque_constant = ExtractDoubleParameter(info, "torque_constant");

  if (nh->has_parameter("position_offset." + joint_name)) {
    position_offset = nh->get_parameter("position_offset." + joint_name).as_double();
  } else {
    position_offset = nh->declare_parameter("position_offset." + joint_name, 0.0);
  }

  position_min = std::numeric_limits<double>::lowest();
  position_max = std::numeric_limits<double>::max();
  for (const auto& interface : info.command_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      position_min = std::stod(interface.min);
      position_max = std::stod(interface.max);
    }
  }
}


bool JointParameters::IsActiveJoint() const {
  return motor_id != kInvalidActuatorID;
}

bool JointParameters::IsZeroReset() const {
  // In the case of WHEEL, zero reset when starting the angle of the wheel shaft
  return control_type == "Wheel";
}

GripperJointParameters::GripperJointParameters(const hardware_interface::ComponentInfo& info) {
  left_spring_joint = ExtractStrParameter(info, "left_spring_joint");
  right_spring_joint = ExtractStrParameter(info, "right_spring_joint");
}
}  // namespace hsrb_robot_hardware
