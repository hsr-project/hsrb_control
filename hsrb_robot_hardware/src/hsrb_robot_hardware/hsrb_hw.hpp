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
/// @file hsrb_hw.hpp
/// HSR-B ROBOTHW class
#ifndef HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_
#define HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>

#include "hsrb_hw_joint.hpp"

namespace hsrb_robot_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template<class Network, class Protocol, class JointComm, class GripperComm, class Joint, class Gripper>
class HsrbHwBase : public hardware_interface::SystemInterface {
 public:
  virtual ~HsrbHwBase() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 protected:
  // It's not very good, put in a place that is easy to put out for the test
  std::shared_ptr<hsrb_servomotor_protocol::INetwork> network_;
  std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> protocol_;

  std::vector<std::shared_ptr<ActiveJoint>> active_joints_;
};

class HsrbHW : public HsrbHwBase<hsrb_servomotor_protocol::ExxxNetwork, hsrb_servomotor_protocol::ExxxProtocol,
                                 JointCommunication, GripperCommunication, ActiveJoint, GripperActiveJoint> {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HsrbHW);
};

}  // namespace hsrb_robot_hardware

#endif/*HSRB_ROBOT_HARDWARE_HSRB_HW_HPP_*/
