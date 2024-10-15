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
#include "hsrb_hw_joint.hpp"

#include <algorithm>
#include <utility>

#include <hardware_interface/handle.hpp>

#include <hsrb_servomotor_protocol/exxx_common.hpp>
#include <hsrb_servomotor_protocol/exxx_error_category.hpp>
#include <hsrb_servomotor_protocol/exxx_warning_category.hpp>

namespace {
constexpr uint32_t kErrorCheckCycle = 1000;
}  // unnamed namespace

namespace hsrb_robot_hardware {

// Generation of SimpletRansmission
std::shared_ptr<transmission_interface::SimpleTransmission> ConfigureTransmission(
    double reduction, double offset, JointValues::Ptr jnt, JointValues::Ptr act) {
  auto transmission = std::make_shared<transmission_interface::SimpleTransmission>(reduction, offset);
  transmission->configure({jnt->GeneratePositionHandle<transmission_interface::JointHandle>(),
                           jnt->GenerateVelocityHandle<transmission_interface::JointHandle>(),
                           jnt->GenerateEffortHandle<transmission_interface::JointHandle>()},
                          {act->GeneratePositionHandle<transmission_interface::ActuatorHandle>(),
                           act->GenerateVelocityHandle<transmission_interface::ActuatorHandle>(),
                           act->GenerateEffortHandle<transmission_interface::ActuatorHandle>()});
  return transmission;
}

// For initialization, writing the current location of the servo and writing to the command value
void ResetCommand(uint8_t drive_mode, JointCommunication::Ptr& comm, JointValues& act,
                  transmission_interface::Transmission& trans) {
  switch (drive_mode) {
    case hsrb_servomotor_protocol::kDriveModePosition:
    case hsrb_servomotor_protocol::kDriveModeActPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeImpedance: {
      comm->GetCurrentPosition(kMotorAxis, act.pos);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity: {
      comm->GetCurrentPosition(kOutputAxis, act.pos);
      break;
    }
    // Write 0 at speed mode
    case hsrb_servomotor_protocol::kDriveModeJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeVelocity: {
      act.vel = 0.0;
      break;
    }
    default:
      break;
  }
  trans.actuator_to_joint();
}

std::vector<hardware_interface::StateInterface> DiagnosticInfo::export_state_interfaces(const std::string& name) {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(name, "motor_id", &motor_id));
  state_interfaces.emplace_back(hardware_interface::StateInterface(name, "temperature", &temperature));
  state_interfaces.emplace_back(hardware_interface::StateInterface(name, "current", &current));
  state_interfaces.emplace_back(warning_status.export_state_interface(name, "warning_status"));
  state_interfaces.emplace_back(error_status.export_state_interface(name, "error_status"));
  state_interfaces.emplace_back(safety_alarm_status.export_state_interface(name, "safety_alarm_status"));
  return state_interfaces;
}

void DiagnosticInfo::ExportStatus() {
  warning_status.Export();
  error_status.Export();
  safety_alarm_status.Export();
}

void DiagnosticInfo::ResetStatus() {
  warning_status.Reset();
  error_status.Reset();
  safety_alarm_status.Reset();
}

ConnectionError::ConnectionError() : pool_(kErrorCheckCycle, false), count_(0), error_rate_(0.0) {}

void ConnectionError::Update(const boost::system::error_code& error) {
  if (pool_.front()) {
    --count_;
  }
  pool_.pop_front();

  if (error && error.category() == boost::system::system_category()) {
    ++count_;
    pool_.push_back(true);
  } else {
    pool_.push_back(false);
  }

  error_rate_ = static_cast<double>(count_) / pool_.size();
}

std::vector<hardware_interface::StateInterface> ConnectionError::export_state_interfaces(const std::string& name) {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(name, "connection_error_rate", &error_rate_));
  return state_interfaces;
}

ActiveJoint::ActiveJoint(const JointCommunication::Ptr& comm,
                         const JointParameters& params)
    : comm_(comm), params_(params), drive_mode_(0), drive_mode_out_(0.0), command_drive_mode_(-1.0) {
  jnt_curr_ = std::make_shared<JointValues>(params.joint_name);
  act_curr_ = std::make_shared<JointValues>(params.joint_name);
  curr_transmission_ = ConfigureTransmission(
      params.reduction, params.position_offset, jnt_curr_, act_curr_);

  jnt_cmd_ = std::make_shared<JointValues>(params.joint_name);
  act_cmd_ = std::make_shared<JointValues>(params.joint_name);
  cmd_transmission_ = ConfigureTransmission(
      params.reduction, params.position_offset, jnt_cmd_, act_cmd_);

  diag_info_ = std::make_shared<DiagnosticInfo>(params.motor_id);
  connection_error_ = std::make_shared<ConnectionError>();
}

std::vector<hardware_interface::StateInterface> ActiveJoint::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(jnt_curr_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(params_.joint_name, "current_drive_mode", &drive_mode_out_));

  auto diag_interfaces = diag_info_->export_state_interfaces(params_.joint_name);
  std::move(diag_interfaces.begin(), diag_interfaces.end(), std::back_inserter(state_interfaces));

  auto connection_error_interfaces = connection_error_->export_state_interfaces(params_.joint_name);
  std::move(connection_error_interfaces.begin(), connection_error_interfaces.end(),
            std::back_inserter(state_interfaces));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActiveJoint::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(jnt_cmd_->GeneratePositionHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(jnt_cmd_->GenerateVelocityHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_drive_mode", &command_drive_mode_));
  return command_interfaces;
}

void ActiveJoint::start() {
  comm_->ResetAlarm();

  drive_mode_ = params_.default_drive_mode;
  drive_mode_out_ = static_cast<double>(drive_mode_);
  comm_->ResetDriveMode(drive_mode_);

  comm_->ResetVelocityLimit(params_.velocity_limit);
  ResetCommand(drive_mode_, comm_, *act_cmd_, *cmd_transmission_);
  if (params_.IsZeroReset()) {
    // Zero reset when the angle of the wheel axis is launched
    comm_->ResetPosition();
  }
}

void ActiveJoint::stop() {
  ResetCommand(drive_mode_, comm_, *act_cmd_, *cmd_transmission_);
}

void ActiveJoint::read() {
  JointCommunication::ReadValues values;
  const auto error = comm_->Read(values);
  UpdateErrorStatus(error);
  // Don't update the present because the value is not read when a system error occurs
  if (error && error.category() == boost::system::system_category()) {
    return;
  }

  drive_mode_ = static_cast<uint8_t>(values.drive_mode);
  drive_mode_out_ = static_cast<double>(drive_mode_);

  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
      act_curr_->pos = values.joint_calc_outaxis_correct_position;
      act_curr_->vel = values.motor_outaxis_velocity;
      break;
    default:
      act_curr_->pos = values.motor_outaxis_position;
      act_curr_->vel = values.motor_outaxis_velocity;
      break;
  }
  act_curr_->eff = values.current / params_.motor_to_joint_gear_ratio * params_.torque_constant;
  diag_info_->temperature = values.temperature;
  diag_info_->current = values.current;

  curr_transmission_->actuator_to_joint();

  // Processing to keep the robot from moving when the controller is unloaded
  // Read-> Update-> Write will be processed in the end of READ, so reset the real -time loop at the end of READ.
  jnt_cmd_->vel = 0.0;
  diag_info_->ExportStatus();
}

void ActiveJoint::write() {
  diag_info_->ResetStatus();

  jnt_cmd_->pos = std::max(params_.position_min, std::min(params_.position_max, jnt_cmd_->pos));
  cmd_transmission_->joint_to_actuator();

  boost::system::error_code error;
  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntVelocity: {
      error = comm_->WriteCommandVelocity(act_cmd_->vel);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModePosition:
    case hsrb_servomotor_protocol::kDriveModeActPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndJntVelocity:
    case hsrb_servomotor_protocol::kDriveModeJntPositionAndActVelocity:
    case hsrb_servomotor_protocol::kDriveModeImpedance: {
      error = comm_->WriteCommandPosition(act_cmd_->pos);
      break;
    }
    default: {
      break;
    }
  }
  UpdateErrorStatus(error);

  // Rewrite of control mode
  auto command_drive_mode = static_cast<int8_t>(command_drive_mode_);
  if (command_drive_mode >= 0) {
    UpdateErrorStatus(comm_->SetDriveMode(static_cast<uint8_t>(command_drive_mode)));
    command_drive_mode_ = -1.0;
  }
}

void ActiveJoint::UpdateErrorStatus(const boost::system::error_code& error) {
  connection_error_->Update(error);

  if (error.category() == hsrb_servomotor_protocol::ExxxWarningCategory()) {
    diag_info_->warning_status.Update(static_cast<uint16_t>(error.value()) &
                                      ~hsrb_servomotor_protocol::exxx_warning_code::kExxxWarningCodeAlarmStatus);

    if (error.value() & hsrb_servomotor_protocol::exxx_warning_code::kExxxWarningCodeAlarmStatus) {
      {
        double error_status = 0.0;
        const auto alarm_error = comm_->ReadAlarm(error_status);
        if (alarm_error.category() != boost::system::system_category()) {
          diag_info_->error_status.Update(static_cast<uint16_t>(error_status));
        }
      }
      {
        double error_status = 0.0;
        const auto alarm_error = comm_->ReadSafetyAlarm(error_status);
        if (alarm_error.category() != boost::system::system_category()) {
          diag_info_->safety_alarm_status.Update(static_cast<uint32_t>(error_status));
        }
      }
    }
  }
}

GripperActiveJoint::GripperActiveJoint(const GripperCommunication::Ptr& comm,
                                       const JointParameters& params,
                                       const GripperJointParameters& gripper_params)
    : ActiveJoint(comm, params), gripper_comm_(comm), gripper_params_(gripper_params) {
  jnt_curr_left_ = std::make_shared<JointValues>(gripper_params.left_spring_joint);
  jnt_curr_right_ = std::make_shared<JointValues>(gripper_params.right_spring_joint);
}

std::vector<hardware_interface::StateInterface> GripperActiveJoint::export_state_interfaces() {
  auto state_interfaces = ActiveJoint::export_state_interfaces();
  state_interfaces.emplace_back(jnt_curr_left_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_left_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_left_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GeneratePositionHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GenerateVelocityHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(jnt_curr_right_->GenerateEffortHandle<hardware_interface::StateInterface>());
  state_interfaces.emplace_back(grasping_flag_curr_.GenerateHandle<hardware_interface::StateInterface>(
      params_.joint_name, "current_grasping_flag"));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GripperActiveJoint::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(jnt_cmd_->GeneratePositionHandle<hardware_interface::CommandInterface>());
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_drive_mode", &command_drive_mode_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, hardware_interface::HW_IF_EFFORT, &effort_cmd_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(params_.joint_name, "command_force", &force_cmd_));
  command_interfaces.emplace_back(grasping_flag_cmd_.GenerateHandle<hardware_interface::CommandInterface>(
      params_.joint_name, "command_grasping_flag"));
  return command_interfaces;
}

void GripperActiveJoint::read() {
  GripperCommunication::GripperValues values;
  const auto error = gripper_comm_->Read(values);
  connection_error_->Update(error);
  // Don't update the present because the value is not read when a system error occurs
  if (error && error.category() == boost::system::system_category()) {
    return;
  }

  drive_mode_ = static_cast<uint8_t>(values.drive_mode);
  drive_mode_out_ = static_cast<double>(drive_mode_);
  grasping_flag_curr_.Set(values.hand_grasping_flag);

  act_curr_->pos = values.hand_motor_position;
  act_curr_->vel = values.hand_motor_velocity;
  act_curr_->eff = values.current / params_.motor_to_joint_gear_ratio * params_.torque_constant;

  jnt_curr_left_->pos = values.hand_left_position - act_curr_->pos;
  jnt_curr_left_->vel = values.hand_left_velocity;
  jnt_curr_left_->eff = values.hand_left_force;

  jnt_curr_right_->pos = values.hand_right_position - act_curr_->pos;
  jnt_curr_right_->vel = values.hand_right_velocity;
  jnt_curr_right_->eff = values.hand_right_force;

  curr_transmission_->actuator_to_joint();
}

void GripperActiveJoint::write() {
  cmd_transmission_->joint_to_actuator();

  boost::system::error_code error;
  switch (drive_mode_) {
    case hsrb_servomotor_protocol::kDriveModeHandGrasp: {
      // With the grip -in flags are not set,
      // Process only when there is a flag set request
      if (grasping_flag_cmd_.Get() && !grasping_flag_curr_.Get()) {
        error = gripper_comm_->WriteGraspingFlag(true);
      }
      if (error) {
        break;
      }
      error = gripper_comm_->WriteCommandEffort(effort_cmd_);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeHandPosition: {
      error = comm_->WriteCommandPosition(act_cmd_->pos);
      break;
    }
    case hsrb_servomotor_protocol::kDriveModeHandSE: {
      error = gripper_comm_->WriteCommandForce(force_cmd_);
      break;
    }
    default: {
      break;
    }
  }
  connection_error_->Update(error);

  // Rewrite of control mode
  auto command_drive_mode = static_cast<int8_t>(command_drive_mode_);
  if (command_drive_mode >= 0) {
    connection_error_->Update(comm_->SetDriveMode(static_cast<uint8_t>(command_drive_mode)));
    command_drive_mode_ = -1.0;
  }
}
}  // namespace hsrb_robot_hardware
