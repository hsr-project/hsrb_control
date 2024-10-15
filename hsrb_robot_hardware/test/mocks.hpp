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
/// @brief Test mock
#ifndef HSRB_ROBOT_HARDWARE_TEST_MOCKS_HPP_
#define HSRB_ROBOT_HARDWARE_TEST_MOCKS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>

#include "../src/hsrb_robot_hardware/hsrb_hw.cpp"  // NOLINT

namespace hsrb_robot_hardware {

// Network mock
class MockNetwork : public hsrb_servomotor_protocol::INetwork {
 public:
  MockNetwork() {}

  MOCK_METHOD4(Send, boost::system::error_code(uint8_t, uint8_t, const uint8_t*, uint16_t));
  MOCK_METHOD1(Receive, boost::system::error_code(uint8_t));
  MOCK_METHOD2(Receive, boost::system::error_code(uint8_t, std::vector<uint8_t>&));
  MOCK_CONST_METHOD0(last_packet, const std::vector<uint8_t>&());
};

class MockNetworkSuccess : public MockNetwork {
 public:
  MockNetworkSuccess(std::string _device_name, boost::system::error_code& error_out, bool _is_usb_rs485,
                     int32_t _timeout, int32_t _sleep_tick) : MockNetwork() {
    error_out = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }
};

class MockNetworkError : public MockNetwork {
 public:
  MockNetworkError(std::string _device_name, boost::system::error_code& error_out, bool _is_usb_rs485,
                   int32_t _timeout, int32_t _sleep_tick) : MockNetwork() {
    error_out = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  }
};

// Protocol mock
class MockProtocol : public hsrb_servomotor_protocol::IDynamixelishProtocol {
 public:
  MockProtocol() : network(nullptr) {}
  explicit MockProtocol(const hsrb_servomotor_protocol::INetwork::Ptr& _network) : network(_network) {}

  MOCK_METHOD3(ReadInt8, boost::system::error_code(uint8_t, uint16_t, int8_t&));
  MOCK_METHOD3(ReadInt16, boost::system::error_code(uint8_t, uint16_t, int16_t&));
  MOCK_METHOD3(ReadInt32, boost::system::error_code(uint8_t, uint16_t, int32_t&));
  MOCK_METHOD3(ReadInt64, boost::system::error_code(uint8_t, uint16_t, int64_t&));
  MOCK_METHOD3(ReadUInt8, boost::system::error_code(uint8_t, uint16_t, uint8_t&));
  MOCK_METHOD3(ReadUInt16, boost::system::error_code(uint8_t, uint16_t, uint16_t&));
  MOCK_METHOD3(ReadUInt32, boost::system::error_code(uint8_t, uint16_t, uint32_t&));
  MOCK_METHOD3(ReadUInt64, boost::system::error_code(uint8_t, uint16_t, uint64_t&));
  MOCK_METHOD3(ReadFloat32, boost::system::error_code(uint8_t, uint16_t, float&));
  MOCK_METHOD3(ReadFloat64, boost::system::error_code(uint8_t, uint16_t, double&));
  MOCK_METHOD4(ReadBlock, boost::system::error_code(uint8_t, uint16_t, uint16_t, uint8_t*));

  MOCK_METHOD3(WriteInt8, boost::system::error_code(uint8_t, uint16_t, int8_t));
  MOCK_METHOD3(WriteInt16, boost::system::error_code(uint8_t, uint16_t, int16_t));
  MOCK_METHOD3(WriteInt32, boost::system::error_code(uint8_t, uint16_t, int32_t));
  MOCK_METHOD3(WriteInt64, boost::system::error_code(uint8_t, uint16_t, int64_t));
  MOCK_METHOD3(WriteUInt8, boost::system::error_code(uint8_t, uint16_t, uint8_t));
  MOCK_METHOD3(WriteUInt16, boost::system::error_code(uint8_t, uint16_t, uint16_t));
  MOCK_METHOD3(WriteUInt32, boost::system::error_code(uint8_t, uint16_t, uint32_t));
  MOCK_METHOD3(WriteUInt64, boost::system::error_code(uint8_t, uint16_t, uint64_t));
  MOCK_METHOD3(WriteFloat32, boost::system::error_code(uint8_t, uint16_t, float));
  MOCK_METHOD3(WriteFloat64, boost::system::error_code(uint8_t, uint16_t, double));
  MOCK_METHOD4(WriteBlock, boost::system::error_code(uint8_t, uint16_t, uint16_t, const uint8_t*));

  MOCK_METHOD1(AvagoAvePos, boost::system::error_code(uint8_t));
  MOCK_METHOD3(ReadHash, boost::system::error_code(uint8_t, std::vector<uint8_t>&, std::vector<uint8_t>&));
  MOCK_METHOD1(SyncParam, boost::system::error_code(uint8_t));

  // It's for testing and do it easily with Public
  hsrb_servomotor_protocol::INetwork::Ptr network;
};

class MockProtocolReadHashError : public MockProtocol {
 public:
  explicit MockProtocolReadHashError(const hsrb_servomotor_protocol::INetwork::Ptr& _network)
      : MockProtocol(_network) {}

  boost::system::error_code ReadHash(uint8_t id, std::vector<uint8_t>& control_table_hash_out,
                                     std::vector<uint8_t>& firmware_hash_out) override {
    return boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  }
};

class MockProtocolReadHashSuccess : public MockProtocol {
 public:
  explicit MockProtocolReadHashSuccess(const hsrb_servomotor_protocol::INetwork::Ptr& _network)
      : MockProtocol(_network) {}

  boost::system::error_code ReadHash(uint8_t id, std::vector<uint8_t>& control_table_hash_out,
                                     std::vector<uint8_t>& firmware_hash_out) override {
    // V1.0.0 hash value
    control_table_hash_out = {142, 88, 226, 143, 51, 242, 57, 11, 85, 191, 50, 44, 192, 117, 109, 194};
    return boost::system::error_code(boost::system::errc::success, boost::system::system_category());
  }
};

// Communication mock
class JointCommunicationMock : public JointCommunication {
 public:
  JointCommunicationMock() = default;
  JointCommunicationMock(uint8_t _act_id,
                         const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& _protocol,
                         const std::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table)
      : JointCommunication(), act_id(_act_id), protocol(_protocol) {}

  uint8_t act_id;
  std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> protocol;

  MOCK_METHOD0(ResetAlarm, void());
  MOCK_METHOD1(ResetDriveMode, void(uint8_t));
  MOCK_METHOD1(ResetVelocityLimit, void(double));
  MOCK_METHOD2(GetCurrentPosition, void(JointAxis, double&));
  MOCK_METHOD0(ResetPosition, void());

  MOCK_METHOD1(Read, boost::system::error_code(ReadValues&));
  MOCK_METHOD1(WriteCommandPosition, boost::system::error_code(double));
  MOCK_METHOD1(WriteCommandVelocity, boost::system::error_code(double));
  MOCK_METHOD1(SetDriveMode, boost::system::error_code(uint8_t));

  MOCK_METHOD1(ReadAlarm, boost::system::error_code(double&));
  MOCK_METHOD1(ReadSafetyAlarm, boost::system::error_code(double&));
};

class GripperCommunicationMock : public GripperCommunication {
 public:
  GripperCommunicationMock() = default;
  GripperCommunicationMock(uint8_t _act_id,
                           const std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol>& _protocol,
                           const std::shared_ptr<hsrb_servomotor_protocol::ControlTable>& table)
      : GripperCommunication(), act_id(_act_id), protocol(_protocol) {}

  uint8_t act_id;
  std::shared_ptr<hsrb_servomotor_protocol::IDynamixelishProtocol> protocol;

  MOCK_METHOD1(Read, boost::system::error_code(GripperValues&));
  MOCK_METHOD1(WriteCommandPosition, boost::system::error_code(double));
  MOCK_METHOD1(WriteCommandForce, boost::system::error_code(double));
  MOCK_METHOD1(WriteCommandEffort, boost::system::error_code(double));
  MOCK_METHOD1(WriteGraspingFlag, boost::system::error_code(bool));
  MOCK_METHOD1(SetDriveMode, boost::system::error_code(uint8_t));
};

// Joint mock
class ActiveJointMock : public ActiveJoint {
 public:
  ActiveJointMock(const JointCommunication::Ptr& _comm, const JointParameters& _params)
      : ActiveJoint(_comm, _params), comm(_comm), params(_params) {}

  JointCommunication::Ptr comm;
  JointParameters params;

  MOCK_METHOD0(start, void());
  MOCK_METHOD0(stop, void());
  MOCK_METHOD0(read, void());
  MOCK_METHOD0(write, void());
};

class GripperActiveJointMock : public GripperActiveJoint {
 public:
  GripperActiveJointMock(const GripperCommunication::Ptr& _comm, const JointParameters& _params,
                         const GripperJointParameters& _gripper_params)
      : GripperActiveJoint(_comm, _params, _gripper_params),
        comm(_comm), params(_params), gripper_params(_gripper_params) {}

  GripperCommunication::Ptr comm;
  JointParameters params;
  GripperJointParameters gripper_params;

  MOCK_METHOD0(start, void());
  MOCK_METHOD0(stop, void());
  MOCK_METHOD0(read, void());
  MOCK_METHOD0(write, void());
};

// HSRBHW test template
template<class Network, class Protocol>
class HsrbHwTestTarget : public HsrbHwBase<Network, Protocol,
                                           JointCommunicationMock, GripperCommunicationMock,
                                           ActiveJointMock, GripperActiveJointMock> {
 public:
  std::shared_ptr<Network> network() const { return std::dynamic_pointer_cast<Network>(this->network_); }
  std::shared_ptr<Protocol> protocol() const { return std::dynamic_pointer_cast<Protocol>(this->protocol_); }
  std::vector<std::shared_ptr<ActiveJoint>> active_joints() const { return this->active_joints_; }
};


}  // namespace hsrb_robot_hardware

#endif  // HSRB_ROBOT_HARDWARE_TEST_MOCKS_HPP_
