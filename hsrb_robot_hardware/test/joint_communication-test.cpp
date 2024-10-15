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
/// @brief Test of communication class with joints
#include <string>
#include <vector>

#include "../src/hsrb_robot_hardware/joint_communication.hpp"

#include "mocks.hpp"

namespace {

constexpr uint32_t kActId = 42;

// I want to make a ConsTEXPR, but with Ubuntu18/Ros Melodic, the build does not pass
const boost::system::error_code kSuccess =
    boost::system::error_code(boost::system::errc::success, boost::system::system_category());
const boost::system::error_code kTimeOut =
    boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
const boost::system::error_code kInvalidArg =
    boost::system::error_code(boost::system::errc::invalid_argument, boost::system::system_category());

template<typename TYPE>
struct ArgumentCache {
  std::vector<uint16_t> address_seq;
  std::vector<TYPE> value_seq;

  void Store(uint8_t act_id, uint16_t address, TYPE value) {
    address_seq.push_back(address);
    value_seq.push_back(value);
  }
  void Clear() {
    address_seq.clear();
    value_seq.clear();
  }
};

template <class T>
double DoubleCast(const uint8_t* bytes) {
  return static_cast<double>(*(reinterpret_cast<const T*>(&bytes[0])));
}

template<typename TYPE>
struct ByteSeqCache {
  void Store(uint8_t act_id, uint16_t address, uint16_t size, const uint8_t* byte_seq) {
    value = DoubleCast<TYPE>(byte_seq);
  }
  TYPE value;
};

}  // namespace

namespace hsrb_robot_hardware {

class JointCommunicationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  virtual void Reset(const std::string& control_table_csv);

  std::shared_ptr<MockProtocol> mock_protocol_;
  std::shared_ptr<JointCommunication> joint_communication_;
};

void JointCommunicationTest::SetUp() {
  mock_protocol_ = std::make_shared<MockProtocol>();
  Reset("control_table.csv");
}

void JointCommunicationTest::Reset(const std::string& control_table_csv) {
  auto table = std::make_shared<hsrb_servomotor_protocol::ControlTable>();
  table->Load(control_table_csv);

  // Necessary because there is a Hash value check in the constructor of JointCommunication
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .Times(1)
      .WillRepeatedly(
          ::testing::DoAll(::testing::SetArgReferee<1>(table->GetMd5Sum()),
                           ::testing::Return(kSuccess)));

  joint_communication_ = std::make_shared<JointCommunication>(kActId, mock_protocol_, table);
}

// Readhash normal system
TEST_F(JointCommunicationTest, ReadHashNormal) {
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .WillRepeatedly(
          ::testing::DoAll(::testing::SetArgReferee<1>(std::vector<uint8_t>({1, 2})),
                           ::testing::SetArgReferee<2>(std::vector<uint8_t>({3, 4})),
                           ::testing::Return(kSuccess)));

  std::vector<uint8_t> control_table_hash_vec;
  std::vector<uint8_t> firmware_hash_vec;
  EXPECT_TRUE(joint_communication_->ReadHash(control_table_hash_vec, firmware_hash_vec));

  ASSERT_EQ(control_table_hash_vec.size(), 2);
  EXPECT_EQ(control_table_hash_vec[0], 1);
  EXPECT_EQ(control_table_hash_vec[1], 2);
  ASSERT_EQ(firmware_hash_vec.size(), 2);
  EXPECT_EQ(firmware_hash_vec[0], 3);
  EXPECT_EQ(firmware_hash_vec[1], 4);

  std::string control_table_hash_str;
  std::string firmware_hash_str;
  EXPECT_TRUE(joint_communication_->ReadHash(control_table_hash_str, firmware_hash_str));

  EXPECT_EQ(control_table_hash_str, "0102");
  EXPECT_EQ(firmware_hash_str, "0304");
}

// ReadHash retry
TEST_F(JointCommunicationTest, ReadHashRetry) {
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .Times(4)
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillRepeatedly(::testing::Return(kSuccess));

  std::vector<uint8_t> control_table_hash_vec;
  std::vector<uint8_t> firmware_hash_vec;
  EXPECT_TRUE(joint_communication_->ReadHash(control_table_hash_vec, firmware_hash_vec));

  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .Times(4)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_FALSE(joint_communication_->ReadHash(control_table_hash_vec, firmware_hash_vec));
}

// LoadControltable normal system
TEST_F(JointCommunicationTest, LoadControlTableNormal) {
  // V1.0.0 hash value
  std::vector<uint8_t> table_hash = {142, 88, 226, 143, 51, 242, 57, 11, 85, 191, 50, 44, 192, 117, 109, 194};
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .WillRepeatedly(
          ::testing::DoAll(::testing::SetArgReferee<1>(table_hash),
                           ::testing::Return(kSuccess)));

  auto table = std::make_shared<hsrb_servomotor_protocol::ControlTable>();
  EXPECT_TRUE(LoadControlTable(mock_protocol_, kActId, ".", table));
}

// LoadControltable fails with ReadHash
TEST_F(JointCommunicationTest, LoadControlTableReadHashError) {
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .WillRepeatedly(::testing::Return(kTimeOut));

  auto table = std::make_shared<hsrb_servomotor_protocol::ControlTable>();
  EXPECT_FALSE(LoadControlTable(mock_protocol_, kActId, ".", table));
}

// LoadControltable failed due to Hash mismatch
TEST_F(JointCommunicationTest, LoadControlTableHashMismatch) {
  std::vector<uint8_t> table_hash = {0, 88, 226, 143, 51, 242, 57, 11, 85, 191, 50, 44, 192, 117, 109, 194};
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .WillRepeatedly(
          ::testing::DoAll(::testing::SetArgReferee<1>(table_hash),
                           ::testing::Return(kSuccess)));

  auto table = std::make_shared<hsrb_servomotor_protocol::ControlTable>();
  EXPECT_FALSE(LoadControlTable(mock_protocol_, kActId, ".", table));
}

// RESETALARM's normal system, no Safety_alarm
TEST_F(JointCommunicationTest, ResetAlarmWithoutSafety) {
  ArgumentCache<uint16_t> arg_cache;
  EXPECT_CALL(*mock_protocol_, WriteUInt16(kActId, testing::_, testing::_))
      .Times(2)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&arg_cache, &ArgumentCache<uint16_t>::Store),
                                       ::testing::Return(kSuccess)));

  joint_communication_->ResetAlarm();

  EXPECT_EQ(arg_cache.address_seq[0], 7);
  EXPECT_EQ(arg_cache.value_seq[0], 0);

  EXPECT_EQ(arg_cache.address_seq[1], 31);
  EXPECT_EQ(arg_cache.value_seq[1], 0);
}

// Normal system of Resetalarm, Safety_alarm
TEST_F(JointCommunicationTest, ResetAlarmWithSafety) {
  Reset("control_table_hsrc.csv");

  EXPECT_CALL(*mock_protocol_, WriteUInt16(kActId, testing::_, testing::_))
      .Times(2)
      .WillRepeatedly(::testing::Return(kSuccess));
  EXPECT_CALL(*mock_protocol_, WriteUInt32(kActId, 9, 0))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  joint_communication_->ResetAlarm();
}

// RESETARARM retry, no safety_alarm
TEST_F(JointCommunicationTest, ResetAlarmRetryWithoutSafety) {
  // The reset of the alarm continues only with error display even if it fails
  // Try Error, Warn 4 times respectively
  ArgumentCache<uint16_t> arg_cache;
  EXPECT_CALL(*mock_protocol_, WriteUInt16(kActId, testing::_, testing::_))
      .Times(8)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&arg_cache, &ArgumentCache<uint16_t>::Store),
                                       ::testing::Return(kTimeOut)));

  joint_communication_->ResetAlarm();

  EXPECT_EQ(arg_cache.address_seq[0], 7);
  EXPECT_EQ(arg_cache.address_seq[1], 7);
  EXPECT_EQ(arg_cache.address_seq[2], 7);
  EXPECT_EQ(arg_cache.address_seq[3], 7);
  EXPECT_EQ(arg_cache.address_seq[4], 31);
  EXPECT_EQ(arg_cache.address_seq[5], 31);
  EXPECT_EQ(arg_cache.address_seq[6], 31);
  EXPECT_EQ(arg_cache.address_seq[7], 31);
}

// Resetalarm retry, Safety_alarm
TEST_F(JointCommunicationTest, ResetAlarmRetryWithSafety) {
  Reset("control_table_hsrc.csv");

  // Try ERROR, WARN, Safety 4 times respectively
  EXPECT_CALL(*mock_protocol_, WriteUInt16(kActId, testing::_, testing::_))
      .Times(8)
      .WillRepeatedly(::testing::Return(kTimeOut));
  EXPECT_CALL(*mock_protocol_, WriteUInt32(kActId, 9, 0))
      .Times(4)
      .WillRepeatedly(::testing::Return(kTimeOut));

  joint_communication_->ResetAlarm();
}

// RESETDRIVEMODE normal system
TEST_F(JointCommunicationTest, ResetDriveModeNormal) {
  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 4, 21))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  joint_communication_->ResetDriveMode(21);
}

// RESETDRIVEMODE retry
TEST_F(JointCommunicationTest, ResetDriveModeRetry) {
  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 4, 21))
      .Times(4)
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillRepeatedly(::testing::Return(kSuccess));

  joint_communication_->ResetDriveMode(21);

  ::testing::Mock::AllowLeak(&mock_protocol_);
  auto op = [this] {EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 4, 21))
                    .Times(4)
                    .WillRepeatedly(::testing::Return(kTimeOut));};

  EXPECT_DEATH({op(); joint_communication_->ResetDriveMode(21);}, "");
}

// Normal system of ResetVelocityLimit
TEST_F(JointCommunicationTest, ResetVelocityLimitNormal) {
  // With savergpointee, you can only get the first part of the byte column, so it will be compatible with Invoke.
  ByteSeqCache<int64_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 337, 8, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int64_t>::Store),
                                       ::testing::Return(kSuccess)));

  joint_communication_->ResetVelocityLimit(1.0);
  EXPECT_EQ(cache.value, 1000000);
}

// RESETVELOCITYLIMIT retry
TEST_F(JointCommunicationTest, ResetVelocityLimitRetry) {
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 337, 8, ::testing::_))
      .Times(4)
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillRepeatedly(::testing::Return(kSuccess));

  joint_communication_->ResetVelocityLimit(1.0);

  ::testing::Mock::AllowLeak(&mock_protocol_);
  auto op = [this] {EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 337, 8, ::testing::_))
                    .Times(4)
                    .WillRepeatedly(::testing::Return(kTimeOut));};

  EXPECT_DEATH({op(); joint_communication_->ResetVelocityLimit(1.0);}, "");
}

// GetCurrentPusion normal system
TEST_F(JointCommunicationTest, GetCurrentPositionNormal) {
  // Ikmd -rate expression of 10000000000
  std::vector<uint8_t> response(8);
  response[0] = 64;
  response[1] = 66;
  response[2] = 15;

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 763, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  double position = 0.0;
  joint_communication_->GetCurrentPosition(kOutputAxis, position);
  EXPECT_DOUBLE_EQ(position, 1.0);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 751, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));
  position = 0.0;
  joint_communication_->GetCurrentPosition(kMotorAxis, position);
  EXPECT_DOUBLE_EQ(position, 1.0);
}

// GetCurrentPusion retry
TEST_F(JointCommunicationTest, GetCurrentPositionRetry) {
  std::vector<uint8_t> response(8);
  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 763, 4, ::testing::_))
      .Times(4)
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  double position;
  joint_communication_->GetCurrentPosition(kOutputAxis, position);

  ::testing::Mock::AllowLeak(&mock_protocol_);
  auto op = [this, response] {
      EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 763, 4, ::testing::_))
          .Times(4)
          .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                          ::testing::Return(kTimeOut)));};

  EXPECT_DEATH({op(); joint_communication_->GetCurrentPosition(kOutputAxis, position);}, "");
}

// RESETPOSITION's normal system
TEST_F(JointCommunicationTest, ResetPositionNormal) {
  ByteSeqCache<int64_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 67, 8, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int64_t>::Store),
                                       ::testing::Return(kSuccess)));

  joint_communication_->ResetPosition();
  EXPECT_EQ(cache.value, 0);
}

// RESETPOSITION retry
TEST_F(JointCommunicationTest, ResetPositionRetry) {
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 67, 8, ::testing::_))
      .Times(4)
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillOnce(::testing::Return(kTimeOut))
      .WillRepeatedly(::testing::Return(kSuccess));

  joint_communication_->ResetPosition();

  ::testing::Mock::AllowLeak(&mock_protocol_);
  auto op = [this] {EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 67, 8, ::testing::_))
                    .Times(4)
                    .WillRepeatedly(::testing::Return(kTimeOut));};

  EXPECT_DEATH({op(); joint_communication_->ResetPosition();}, "");
}

// READ test
TEST_F(JointCommunicationTest, Read) {
  // It should be read in this order
  // int8_t,   _present_temperature
  // int16_t,  _present_current
  // uint8_t,  _present_drive_mode
  // uint16_t, _present_alarm_status
  // int32_t,  _present_motor_outaxis_position
  // int64_t,  _present_motor_outaxis_velocity
  // int32_t,  _present_joint_calc_outaxis_correct_position
  std::vector<uint8_t> response(22);
  response[0] = 1;
  response[1] = 2;
  response[3] = 3;
  response[6] = 5;
  response[10] = 6;
  response[18] = 7;

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 745, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  JointCommunication::ReadValues values;
  EXPECT_EQ(joint_communication_->Read(values), kSuccess);

  EXPECT_DOUBLE_EQ(values.temperature, 1.0);
  EXPECT_DOUBLE_EQ(values.current, 0.002);
  EXPECT_DOUBLE_EQ(values.drive_mode, 3.0);
  EXPECT_DOUBLE_EQ(values.motor_outaxis_position, 5.0e-6);
  EXPECT_DOUBLE_EQ(values.motor_outaxis_velocity, 6.0e-6);
  EXPECT_DOUBLE_EQ(values.joint_calc_outaxis_correct_position, 7.0e-6);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 745, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->Read(values), kTimeOut);
}

// WritecommandPusion test
TEST_F(JointCommunicationTest, WriteCommandPosition) {
  ByteSeqCache<int32_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 297, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int32_t>::Store),
                                       ::testing::Return(kSuccess)));

  EXPECT_EQ(joint_communication_->WriteCommandPosition(1.0), kSuccess);
  EXPECT_EQ(cache.value, 1000000);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 297, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->WriteCommandPosition(1.0), kTimeOut);
}

// Writecommandvelocity test
TEST_F(JointCommunicationTest, WriteCommandVelocity) {
  ByteSeqCache<int64_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 269, 8, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int64_t>::Store),
                                       ::testing::Return(kSuccess)));

  EXPECT_EQ(joint_communication_->WriteCommandVelocity(1.0), kSuccess);
  EXPECT_EQ(cache.value, 1000000);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 269, 8, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->WriteCommandVelocity(1.0), kTimeOut);
}

// Readalarm test
TEST_F(JointCommunicationTest, ReadAlarm) {
  std::vector<uint8_t> response(2);
  response[0] = 1;

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 7, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  double alarm = 0.0;
  EXPECT_EQ(joint_communication_->ReadAlarm(alarm), kSuccess);
  EXPECT_DOUBLE_EQ(alarm, 1.0);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 7, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->ReadAlarm(alarm), kTimeOut);
}

// READSAFETYALARM test
TEST_F(JointCommunicationTest, ReadSafetyAlarm) {
  // Safety_alarm does not exist, succeed without doing anything
  EXPECT_CALL(*mock_protocol_, ReadBlock(::testing::_, ::testing::_, ::testing::_, ::testing::_)).Times(0);

  double alarm = 1.0;
  EXPECT_EQ(joint_communication_->ReadSafetyAlarm(alarm), kSuccess);
  EXPECT_DOUBLE_EQ(alarm, 0.0);

  Reset("control_table_hsrc.csv");

  std::vector<uint8_t> response(2);
  response[0] = 1;

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 9, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  EXPECT_EQ(joint_communication_->ReadSafetyAlarm(alarm), kSuccess);
  EXPECT_DOUBLE_EQ(alarm, 1.0);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 9, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->ReadSafetyAlarm(alarm), kTimeOut);
}

// SetDriveMode test
TEST_F(JointCommunicationTest, SetDriveMode) {
  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 4, 21))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  EXPECT_EQ(joint_communication_->SetDriveMode(21), kSuccess);

  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 4, 21))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->SetDriveMode(21), kTimeOut);
}

// WritecontroltableItem normal system
TEST_F(JointCommunicationTest, WriteControlTableItemNormal) {
  ByteSeqCache<uint8_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));
  EXPECT_CALL(*mock_protocol_, SyncParam(kActId)).Times(0);

  EXPECT_EQ(joint_communication_->WriteControlTableItem("command_drive_mode", 21.0, false), kSuccess);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));
  EXPECT_CALL(*mock_protocol_, SyncParam(kActId))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  EXPECT_EQ(joint_communication_->WriteControlTableItem("command_drive_mode", 21.0, true), kSuccess);
}

// Service communication failed with WritecontroltableItem
TEST_F(JointCommunicationTest, WriteControlTableItemFailure) {
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));
  EXPECT_CALL(*mock_protocol_, SyncParam(kActId)).Times(0);

  EXPECT_EQ(joint_communication_->WriteControlTableItem("command_drive_mode", 21.0, true), kTimeOut);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));
  EXPECT_CALL(*mock_protocol_, SyncParam(kActId))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->WriteControlTableItem("command_drive_mode", 21.0, true), kTimeOut);
}

// Specify an unauthorized name in WritecontroltableItem
TEST_F(JointCommunicationTest, WriteControlTableItemWithInvalidItem) {
  EXPECT_CALL(*mock_protocol_, WriteBlock(::testing::_, ::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_EQ(joint_communication_->WriteControlTableItem("hoge", 0.0, false), kInvalidArg);
}

// READCONTROLTABLEITEM test
TEST_F(JointCommunicationTest, ReadControlTableItem) {
  std::vector<uint8_t> response(1);
  response[0] = 21;

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  double value;
  EXPECT_EQ(joint_communication_->ReadControlTableItem("command_drive_mode", value), kSuccess);
  EXPECT_DOUBLE_EQ(value, 21.0);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 4, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->ReadControlTableItem("command_drive_mode", value), kTimeOut);
}

// Specify an unauthorized name in ReadControltableItem
TEST_F(JointCommunicationTest, ReadControlTableItemWithInvalidItem) {
  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 4, ::testing::_, ::testing::_)).Times(0);
  double value;
  EXPECT_EQ(joint_communication_->ReadControlTableItem("hoge", value), kInvalidArg);
}

// ResetoutputEncoder test
TEST_F(JointCommunicationTest, ResetOutputEncoder) {
  EXPECT_CALL(*mock_protocol_, AvagoAvePos(kActId))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  EXPECT_EQ(joint_communication_->ResetOutputEncoder(), kSuccess);

  EXPECT_CALL(*mock_protocol_, AvagoAvePos(kActId))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(joint_communication_->ResetOutputEncoder(), kTimeOut);
}


class GripperCommunicationTest : public JointCommunicationTest {
 protected:
  void Reset(const std::string& control_table_csv) override;

  std::shared_ptr<GripperCommunication> gripper_communication_;
};

void GripperCommunicationTest::Reset(const std::string& control_table_csv) {
  auto table = std::make_shared<hsrb_servomotor_protocol::ControlTable>();
  table->Load(control_table_csv);

  // Necessary because there is a Hash value check in the constructor of JointCommunication
  EXPECT_CALL(*mock_protocol_, ReadHash(kActId, testing::_, testing::_))
      .Times(1)
      .WillRepeatedly(
          ::testing::DoAll(::testing::SetArgReferee<1>(table->GetMd5Sum()),
                           ::testing::Return(kSuccess)));

  gripper_communication_ = std::make_shared<GripperCommunication>(kActId, mock_protocol_, table);
}

// READ test
TEST_F(GripperCommunicationTest, Read) {
  std::vector<uint8_t> response(38);
  response[0] = 1;  // int32_t,_present_hand_motor_position
  response[4] = 2;  // int32_t,_present_hand_left_position
  response[8] = 3;  // int32_t,_present_hand_right_position
  response[12] = 4;  // int32_t,_present_hand_motor_velocity
  response[16] = 5;  // int32_t,_present_hand_left_velocity
  response[20] = 6;  // int32_t,_present_hand_right_velocity
  response[24] = 7;  // int16_t,_present_hand_left_force
  response[26] = 8;  // int16_t,_present_hand_right_force
  response[28] = 1;  // uint8_t,_present_hand_grasping_flag
  response[29] = 10;  // int32_t,_present_effort
  response[33] = 11;  // uint8_t,_present_supply_voltage
  response[34] = 12;  // int8_t,_present_temperature
  response[35] = 13;  // int16_t,_present_current
  response[37] = 14;  // uint8_t,_present_drive_mode

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 711, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::SetArrayArgument<3>(response.begin(), response.end()),
                                       ::testing::Return(kSuccess)));

  GripperCommunication::GripperValues values;
  EXPECT_EQ(gripper_communication_->Read(values), kSuccess);

  EXPECT_DOUBLE_EQ(values.hand_motor_position, 1.0e-6);
  EXPECT_DOUBLE_EQ(values.hand_left_position, 2.0e-6);
  EXPECT_DOUBLE_EQ(values.hand_right_position, 3.0e-6);

  EXPECT_DOUBLE_EQ(values.hand_motor_velocity, 4.0e-6);
  EXPECT_DOUBLE_EQ(values.hand_left_velocity, 5.0e-6);
  EXPECT_DOUBLE_EQ(values.hand_right_velocity, 6.0e-6);

  EXPECT_DOUBLE_EQ(values.hand_left_force, 7.0e-3);
  EXPECT_DOUBLE_EQ(values.hand_right_force, 8.0e-3);

  EXPECT_DOUBLE_EQ(values.hand_grasping_flag, 1.0);
  EXPECT_DOUBLE_EQ(values.effort, 10.0e-6);
  EXPECT_DOUBLE_EQ(values.supply_voltage, 11.0);

  EXPECT_DOUBLE_EQ(values.temperature, 12.0);
  EXPECT_DOUBLE_EQ(values.current, 13.0e-3);
  EXPECT_DOUBLE_EQ(values.drive_mode, 14.0);

  EXPECT_CALL(*mock_protocol_, ReadBlock(kActId, 711, ::testing::_, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(gripper_communication_->Read(values), kTimeOut);
}

// Writecommandforce test
TEST_F(GripperCommunicationTest, WriteCommandForce) {
  ByteSeqCache<int16_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 627, 2, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int16_t>::Store),
                                       ::testing::Return(kSuccess)));

  EXPECT_EQ(gripper_communication_->WriteCommandForce(0.1), kSuccess);
  EXPECT_EQ(cache.value, 100);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 627, 2, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(gripper_communication_->WriteCommandForce(0.1), kTimeOut);
}

// Writecommandeffort test
TEST_F(GripperCommunicationTest, WriteCommandEffort) {
  ByteSeqCache<int32_t> cache;
  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 219, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::DoAll(::testing::Invoke(&cache, &ByteSeqCache<int32_t>::Store),
                                       ::testing::Return(kSuccess)));

  EXPECT_EQ(gripper_communication_->WriteCommandEffort(0.1), kSuccess);
  EXPECT_EQ(cache.value, 100000);

  EXPECT_CALL(*mock_protocol_, WriteBlock(kActId, 219, 4, ::testing::_))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(gripper_communication_->WriteCommandEffort(0.1), kTimeOut);
}

// WritegraspingFlag test
TEST_F(GripperCommunicationTest, WriteGraspingFlag) {
  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 651, 1))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  EXPECT_EQ(gripper_communication_->WriteGraspingFlag(true), kSuccess);

  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 651, 0))
      .Times(1)
      .WillRepeatedly(::testing::Return(kSuccess));

  EXPECT_EQ(gripper_communication_->WriteGraspingFlag(false), kSuccess);

  EXPECT_CALL(*mock_protocol_, WriteUInt8(kActId, 651, 1))
      .Times(1)
      .WillRepeatedly(::testing::Return(kTimeOut));

  EXPECT_EQ(gripper_communication_->WriteGraspingFlag(true), kTimeOut);
}

}  // namespace hsrb_robot_hardware

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}
