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
/// @file hsrb_hw_joint.hpp
/// HSR-B HWJOINT class
#ifndef HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_
#define HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <transmission_interface/simple_transmission.hpp>

#include "joint_communication.hpp"
#include "joint_parameters.hpp"

namespace hsrb_robot_hardware {

struct JointValues {
  using Ptr = std::shared_ptr<JointValues>;

  std::string name;
  double pos;
  double vel;
  double eff;

  explicit JointValues(const std::string& _name) : name(_name), pos(0.0), vel(0.0), eff(0.0) {}

  template<typename TYPE>
  TYPE GeneratePositionHandle() {
    return TYPE(name, hardware_interface::HW_IF_POSITION, &pos);
  }
  template<typename TYPE>
  TYPE GenerateVelocityHandle() {
    return TYPE(name, hardware_interface::HW_IF_VELOCITY, &vel);
  }
  template<typename TYPE>
  TYPE GenerateEffortHandle() {
    return TYPE(name, hardware_interface::HW_IF_EFFORT, &eff);
  }
};

// ROS2_control can only interact with Double type, so define Util classes that handle int (bit columns).
template<typename Type>
class AlarmStatus {
 public:
  AlarmStatus() { Reset(); }
  void Reset() { internal_ = 0; }
  void Update(Type current) { internal_ |= current; }
  void Export() { export_ = static_cast<double>(internal_); }

  hardware_interface::StateInterface export_state_interface(const std::string& joint_name,
                                                            const std::string& interface_name) {
    return hardware_interface::StateInterface(joint_name, interface_name, &export_);
  }

 private:
  double export_;
  Type internal_;
};

struct DiagnosticInfo {
  using Ptr = std::shared_ptr<DiagnosticInfo>;

  // The interface of ros2 control handles only double type
  double motor_id;
  double temperature;
  double current;

  AlarmStatus<uint16_t> warning_status;
  AlarmStatus<uint16_t> error_status;
  AlarmStatus<uint32_t> safety_alarm_status;

  explicit DiagnosticInfo(uint8_t _motor_id)
      : motor_id(static_cast<double>(_motor_id)), temperature(0.0), current(0.0) {}

  std::vector<hardware_interface::StateInterface> export_state_interfaces(const std::string& name);

  void ExportStatus();
  void ResetStatus();
};

class ConnectionError {
 public:
  using Ptr = std::shared_ptr<ConnectionError>;

  ConnectionError();

  void Update(const boost::system::error_code& error);

  std::vector<hardware_interface::StateInterface> export_state_interfaces(const std::string& name);

 private:
  std::deque<bool> pool_;
  uint32_t count_;
  double error_rate_;
};

// ROS2_control can only interact with Double type, so define Util classes that handle Bool type.
class BoolValue {
 public:
  BoolValue() { Set(false); }

  template<typename TYPE>
  TYPE GenerateHandle(const std::string& name, const std::string& interface) {
    return TYPE(name, interface, &value_);
  }

  bool Get() const { return value_ > 0; }
  void Set(bool value) {
    if (value) {
      value_ = 1.0;
    } else {
      value_ = -1.0;
    }
  }
  void Set(double value) { Set(value > 0.5); }

 private:
  double value_;
};

// Driving axis control class
class ActiveJoint {
 public:
  using Ptr = std::shared_ptr<ActiveJoint>;

  ActiveJoint(const JointCommunication::Ptr& comm, const JointParameters& params);
  virtual ~ActiveJoint() = default;

  // Hardware_interface :: SystemICE function name
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces();
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  virtual void start();
  virtual void stop();
  virtual void read();
  virtual void write();

 protected:
  void UpdateErrorStatus(const boost::system::error_code& error);

  JointCommunication::Ptr comm_;
  JointParameters params_;

  // Int system is desirable because there is a branch in the Switch statement by Drive_mode
  // However, StateInterface receives Double pointer, so Double is also prepared.
  int8_t drive_mode_;
  double drive_mode_out_;
  // If Command_drive_mode is a negative value, there is no command value
  // It is good if you can use optional, but it can't be helped because it can't be passed to CommandInterface
  double command_drive_mode_;

  JointValues::Ptr jnt_curr_;
  JointValues::Ptr act_curr_;
  std::shared_ptr<transmission_interface::Transmission> curr_transmission_;

  JointValues::Ptr jnt_cmd_;
  JointValues::Ptr act_cmd_;
  std::shared_ptr<transmission_interface::Transmission> cmd_transmission_;

  DiagnosticInfo::Ptr diag_info_;
  ConnectionError::Ptr connection_error_;
};

// Gripper control class
class GripperActiveJoint : public ActiveJoint {
 public:
  using Ptr = std::shared_ptr<GripperActiveJoint>;

  GripperActiveJoint(const GripperCommunication::Ptr& comm, const JointParameters& params,
                     const GripperJointParameters& gripper_params);
  virtual ~GripperActiveJoint() = default;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  void read() override;
  void write() override;

 private:
  GripperCommunication::Ptr gripper_comm_;
  GripperJointParameters gripper_params_;

  JointValues::Ptr jnt_curr_left_;
  JointValues::Ptr jnt_curr_right_;

  double force_cmd_;
  double effort_cmd_;

  BoolValue grasping_flag_curr_;
  BoolValue grasping_flag_cmd_;
};

}  // namespace hsrb_robot_hardware

#endif/*HSRB_ROBOT_HARDWARE_HSRB_HW_JOINT_HPP_*/
