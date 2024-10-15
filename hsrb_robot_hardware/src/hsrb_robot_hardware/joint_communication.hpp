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
/// Communication with joints
#ifndef HSRB_ROBOT_HARDWARE_JOINT_COMMUNICATION_HPP_
#define HSRB_ROBOT_HARDWARE_JOINT_COMMUNICATION_HPP_
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <hsrb_servomotor_protocol/control_table.hpp>
#include <hsrb_servomotor_protocol/control_table_item_descriptor.hpp>
#include <hsrb_servomotor_protocol/dynamixelish_protocol.hpp>

namespace hsrb_robot_hardware {

boost::system::error_code TryWithRetry(
    std::function<boost::system::error_code()> func,
    uint32_t retry);

bool ReadHash(
    const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol, uint8_t id,
    std::vector<uint8_t>& control_table_hash_out, std::vector<uint8_t>& firmware_hash_out);

bool LoadControlTable(
    const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol, uint8_t id,
    const std::string& package_path, std::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table_out);

enum JointAxis {
  kOutputAxis,
  kMotorAxis
};

class JointCommunication {
 public:
  using Ptr = std::shared_ptr<JointCommunication>;

  JointCommunication() = default;
  JointCommunication(uint8_t act_id, const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol,
                     const std::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table);
  virtual ~JointCommunication() {}

  virtual bool ReadHash(std::vector<uint8_t>& control_table_hash_out, std::vector<uint8_t>& firmware_hash_out);
  virtual bool ReadHash(std::string& control_table_hash_out, std::string& firmware_hash_out);

  // Do not use in a group of retry functions for initialization, in regular communication
  virtual void ResetAlarm();
  virtual void ResetDriveMode(uint8_t drive_mode);
  virtual void ResetVelocityLimit(double limit);
  virtual void GetCurrentPosition(JointAxis joint_axis, double& position_out);
  virtual void ResetPosition();

  // Regular communication
  struct ReadValues {
    double temperature;
    double current;
    double drive_mode;

    double motor_outaxis_position;
    double motor_outaxis_velocity;
    double joint_calc_outaxis_correct_position;
  };
  virtual boost::system::error_code Read(ReadValues& read_values_out);

  virtual boost::system::error_code WriteCommandPosition(double value);
  virtual boost::system::error_code WriteCommandVelocity(double value);

  virtual boost::system::error_code ReadAlarm(double& alarm_out);
  virtual boost::system::error_code ReadSafetyAlarm(double& alarm_out);

  // For exchanges other than regular communication
  virtual boost::system::error_code SetDriveMode(uint8_t drive_mode);
  virtual boost::system::error_code WriteControlTableItem(const std::string& name, double value, bool do_sync);
  virtual boost::system::error_code ReadControlTableItem(const std::string& name, double& value);
  virtual boost::system::error_code ResetOutputEncoder();

 protected:
  uint8_t act_id_;
  std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> protocol_;
  std::shared_ptr<hsrb_servomotor_protocol::ControlTable> table_;

  void ValidateControlTableVersion();

  // Control table
  typedef hsrb_servomotor_protocol::ControlTableItemDescriptor TableItem;
  std::map<std::string, TableItem::ConstPtr> items_;
  std::vector<TableItem::ConstPtr> items_to_read_;

  virtual void InitItemDescriptors();
  void InsertItemDescriptor(const std::string& name);
  virtual void InitItemToRead();

  bool has_safety_alarm_;
};

class GripperCommunication : public JointCommunication {
 public:
  using Ptr = std::shared_ptr<GripperCommunication>;

  GripperCommunication() = default;
  GripperCommunication(uint8_t act_id,
                       const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& protocol,
                       const std::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table)
      : JointCommunication(act_id, protocol, table) {
    InitItemDescriptors();
    InitItemToRead();
  }
  virtual ~GripperCommunication() {}

  // Regular communication
  struct GripperValues {
    double temperature;
    double current;
    double drive_mode;

    double hand_motor_position;
    double hand_left_position;
    double hand_right_position;
    double hand_motor_velocity;
    double hand_left_velocity;
    double hand_right_velocity;
    double hand_left_force;
    double hand_right_force;
    double hand_grasping_flag;
    double effort;
    double supply_voltage;
  };
  virtual boost::system::error_code Read(GripperValues& read_values_out);

  virtual boost::system::error_code WriteCommandForce(double value);
  virtual boost::system::error_code WriteCommandEffort(double value);
  virtual boost::system::error_code WriteGraspingFlag(bool grasping_flag);

 protected:
  void InitItemDescriptors() override;
  void InitItemToRead() override;
};

}  // namespace hsrb_robot_hardware
#endif  // HSRB_ROBOT_HARDWARE_JOINT_COMMUNICATION_HPP_
