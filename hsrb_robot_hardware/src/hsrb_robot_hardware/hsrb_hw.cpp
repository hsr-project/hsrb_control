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
#include "hsrb_hw.hpp"

#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hsrb_servomotor_protocol/control_table.hpp>
#include <tmc_utils/parameters.hpp>

namespace {
const char* const kDefaultRS485Device = "/dev/ttyCTI2";
const bool kDefaultUSB485 = false;
const int32_t kDefaultNetworkTimeout = 2000000;
const int32_t kDefaultNetworkTick = 10000;
}  // unnamed namespace

namespace hsrb_robot_hardware {

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
CallbackReturn HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  auto nh = rclcpp::Node::make_shared("hsrb_hw");
  info_ = hardware_info;
  if (info_.joints.empty()) {
    RCLCPP_FATAL(nh->get_logger(), "No joint settings.");
    return CallbackReturn::ERROR;
  }

  // Read parameters
  std::vector<JointParameters> joint_param_seq;
  for (const auto& joint_info : info_.joints) {
    joint_param_seq.push_back(JointParameters(nh, joint_info));
  }

  // EXXX amplifier communication
  const auto rs485_device = tmc_utils::GetParameter<std::string>(nh, "rs485_device", kDefaultRS485Device);
  const auto use_usb = tmc_utils::GetParameter<bool>(nh, "use_usb", kDefaultUSB485);
  const auto network_timeout = tmc_utils::GetParameter<int32_t>(nh, "network_timeout", kDefaultNetworkTimeout);
  const auto network_tick = tmc_utils::GetParameter<int32_t>(nh, "network_tick", kDefaultNetworkTick);

  boost::system::error_code error;
  network_ = std::make_shared<Network>(rs485_device, error, use_usb, network_timeout, network_tick);
  if (error) {
    RCLCPP_FATAL_STREAM(nh->get_logger(),
                        "System cannot communicate with servo motors. Error message: " << error.message());
    return CallbackReturn::ERROR;
  }
  protocol_ = std::make_shared<Protocol>(network_);

  // Acquisition of control table
  std::shared_ptr<hsrb_servomotor_protocol::ControlTable> table(new hsrb_servomotor_protocol::ControlTable());
  auto package_path = ament_index_cpp::get_package_share_directory("exxx_control_table");
  if (!LoadControlTable(protocol_, joint_param_seq.front().motor_id, package_path, table)) {
    RCLCPP_FATAL(nh->get_logger(), "System cannot load control table.");
    return CallbackReturn::ERROR;
  }
  // Initialization of joints
  for (auto i = 0; i < joint_param_seq.size(); ++i) {
    if (joint_param_seq[i].control_type == "Gripper") {
      auto comm = std::make_shared<GripperComm>(joint_param_seq[i].motor_id, protocol_, table);
      auto gripper_param = GripperJointParameters(info_.joints[i]);
      auto active_joint = std::make_shared<Gripper>(comm, joint_param_seq[i], gripper_param);
      active_joints_.emplace_back(active_joint);
    } else {
      auto comm = std::make_shared<JointComm>(joint_param_seq[i].motor_id, protocol_, table);
      auto active_joint = std::make_shared<Joint>(comm, joint_param_seq[i]);
      active_joints_.emplace_back(active_joint);
    }
  }
  return CallbackReturn::SUCCESS;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
CallbackReturn HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  return CallbackReturn::SUCCESS;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
std::vector<hardware_interface::StateInterface>
HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto joint : active_joints_) {
    auto state_interface = joint->export_state_interfaces();
    std::move(state_interface.begin(), state_interface.end(), std::back_inserter(state_interfaces));
  }
  return state_interfaces;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
std::vector<hardware_interface::CommandInterface>
HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto joint : active_joints_) {
    auto command_interface = joint->export_command_interfaces();
    std::move(command_interface.begin(), command_interface.end(), std::back_inserter(command_interfaces));
  }
  return command_interfaces;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
CallbackReturn HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::on_activate(
  const rclcpp_lifecycle::State& previous_state) {
  for (auto joint : active_joints_) {
    joint->start();
  }
  return CallbackReturn::SUCCESS;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
CallbackReturn HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::on_deactivate(
  const rclcpp_lifecycle::State& previous_state) {
  for (auto joint : active_joints_) {
    joint->stop();
  }
  return CallbackReturn::SUCCESS;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
hardware_interface::return_type HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  for (auto joint : active_joints_) {
    joint->read();
  }
  return hardware_interface::return_type::OK;
}

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
hardware_interface::return_type HsrbHwBase<Network, Protocol, JointComm, GripperComm, Joint, Gripper>::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  for (auto joint : active_joints_) {
    joint->write();
  }
  return hardware_interface::return_type::OK;
}
}  // namespace hsrb_robot_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hsrb_robot_hardware::HsrbHW,
  hardware_interface::SystemInterface)
