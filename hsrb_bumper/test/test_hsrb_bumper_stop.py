'''
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
'''
#! /usr/bin/env python3
u"""hsrb_bumper node test"""
import threading
import time
import unittest

from controller_manager_msgs.srv import SwitchController
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import Empty


_ROS_CONN_TIMEOUT = 30
_TOPIC_TIMEOUT = 2

_TARGET_CONTROLLER_LIST = [
    'omni_base_controller',
    'head_trajectory_controller',
    'arm_trajectory_controller',
    'gripper_controller'
]


@pytest.mark.launch_test
@launch_testing.parametrize(
    'target_controller_list',
    [_TARGET_CONTROLLER_LIST, _TARGET_CONTROLLER_LIST[:-1], _TARGET_CONTROLLER_LIST[2:]])
def generate_test_description(target_controller_list):
    hsrb_bumper_stop_node = launch_ros.actions.Node(
        package='hsrb_bumper',
        executable='hsrb_bumper',
        parameters=[{'target_controller_list': target_controller_list}],
        output='screen'
    )

    return launch.LaunchDescription(
        [hsrb_bumper_stop_node, launch_testing.actions.ReadyToTest()])


class TestHsrbBumperStopNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Sleep for adjusting the timing of node launch, it may not work well if you pull it out.
        time.sleep(0.5)
        self.node = rclpy.create_node("hsrb_bumper_stop_test")

        self.executor = MultiThreadedExecutor()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node, self.executor), daemon=True)
        self.thread.start()

        self.front_bumper_pub = self.node.create_publisher(Bool, 'base_f_bumper_sensor', 10)
        self.rear_bumper_pub = self.node.create_publisher(Bool, 'base_b_bumper_sensor', 10)
        self.diag_sub = self.node.create_subscription(
            DiagnosticArray, 'diagnostics', self.diag_callback, 1)

        self.sw_service_req = None
        self.switch_service_response = True
        self.diag_status = DiagnosticStatus()

        self.switch_service = self.node.create_service(
            SwitchController, 'controller_manager/switch_controller',
            self.switch_service_mock)

        self.reset_service = self.node.create_client(Empty, 'bumper/reset')
        self.reset_service.wait_for_service(timeout_sec=_ROS_CONN_TIMEOUT)

    def tearDown(self):
        self.node.destroy_node()

    def diag_callback(self, diag):
        self.diag_status = diag.status[0]
        self.assertEqual(self.diag_status.name, "hsrb_bumper")

    def switch_service_mock(self, req, response):
        self.sw_service_req = req
        response.ok = self.switch_service_response
        return response

    def front_publish(self, data):
        message = Bool()
        message.data = data
        self.front_bumper_pub.publish(message)

    def rear_publish(self, data):
        message = Bool()
        message.data = data
        self.rear_bumper_pub.publish(message)

    def wait_for_target_topic(self, level, message=None, timeout=_TOPIC_TIMEOUT):
        start = self.node.get_clock().now()
        while (self.node.get_clock().now() - start) < rclpy.time.Duration(seconds=timeout):
            if self.diag_status.level == level:
                if message is None or message in self.diag_status.message:
                    return True
                return False
        return False

    def test_normal_without_bumper_on(self):
        u"""Testing in a standby state where the bumper is not pressed"""
        rate = self.node.create_rate(10)
        for var in range(0, 10):
            self.front_publish(False)
            self.rear_publish(False)
            rate.sleep()
        # DIAG Level is OK or test
        self.assertTrue(self.wait_for_target_topic(DiagnosticStatus.OK))

    def _test_normal_impl(self, controller_list, message="All joints are stopped by bumper"):
        # DIAG test
        self.assertTrue(self.wait_for_target_topic(
            DiagnosticStatus.ERROR, message=message))
        # Testing whether Switch_controller is called as expected
        self.assertEqual(set(self.sw_service_req.stop_controllers), set(controller_list))
        self.assertEqual(self.sw_service_req.start_controllers, list([]))

        # Return (RESET) test
        self.switch_service_response = True
        self.reset_service.wait_for_service(timeout_sec=_ROS_CONN_TIMEOUT)
        self.reset_service.call(Empty.Request())

        # DIAG test
        self.assertTrue(self.wait_for_target_topic(DiagnosticStatus.OK))
        # Testing whether Switch_controller is called as expected
        self.assertEqual(len(self.sw_service_req.start_controllers), len(controller_list))
        self.assertEqual(set(self.sw_service_req.start_controllers), set(controller_list))
        self.assertEqual(self.sw_service_req.stop_controllers, list([]))

    def test_normal_with_front_bumper_on(self, target_controller_list):
        u"""Only FR bumper is ON once"""
        rate = self.node.create_rate(10)
        for var in range(0, 6):
            if var == 2:
                self.front_publish(True)
                self.rear_publish(False)
            else:
                self.front_publish(False)
                self.rear_publish(False)
            rate.sleep()

        self._test_normal_impl(target_controller_list)

    def test_normal_with_rear_bumper_on(self, target_controller_list):
        u"""RR bumper only once"""
        rate = self.node.create_rate(10)
        for var in range(0, 6):
            if var == 2:
                self.front_publish(False)
                self.rear_publish(True)
            else:
                self.front_publish(False)
                self.rear_publish(False)
            rate.sleep()

        self._test_normal_impl(target_controller_list)

    def test_normal_with_bumper_chattering(self, target_controller_list):
        u"""FR bumper repeats on and off"""
        rate = self.node.create_rate(10)
        for var in range(0, 6):
            if (var % 2) == 0:
                self.front_publish(True)
                self.rear_publish(False)
            else:
                self.front_publish(False)
                self.rear_publish(False)
            rate.sleep()

        self._test_normal_impl(target_controller_list)

    def test_fail_to_stop_controller(self, target_controller_list):
        u"""Will the diager appear when the Contora stops stopping?"""
        self.switch_service_response = False
        rate = self.node.create_rate(10)
        for var in range(0, 6):
            if var == 2:
                self.front_publish(True)
                self.rear_publish(False)
            else:
                self.front_publish(False)
                self.rear_publish(False)
            rate.sleep()

        self._test_normal_impl(target_controller_list, message="Bumper was pushed")
