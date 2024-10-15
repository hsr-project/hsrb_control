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
u"""Bumper behavior generation node"""
import threading

from controller_manager_msgs.srv import SwitchController
from diagnostic_msgs.msg import DiagnosticStatus
from hsrb_bumper.hsrb_bumper_base import HsrbBumperBase
import hsrb_bumper.hsrb_bumper_utils as bumper_utils
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from std_srvs.srv import Empty


_ROS_CONN_TIMEOUT = 30
_BEST_EFFORT = 1
_STATE_WATCH_RATE = 1
_DEFAULT_ENABLE_EMERGENCY_STOP = True

# Controller list that you want to control with a bumper switch
_TARGET_CONTROLLER_LIST = [
    'omni_base_controller',
    'head_trajectory_controller',
    'arm_trajectory_controller',
    'gripper_controller'
]


class HsrbBumperStopNode(HsrbBumperBase):
    u"""Class to provide bumper behavior"""

    def __init__(self, node):
        self._node = node
        self._diag_pub = bumper_utils.HsrbBumperDiagPublisher(self._node)
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        # Was the bumper pushed?
        # If you push it even once, it will be true and it will not be cleared until the RESET service is called.
        self._is_bumper_on = False
        # Did the controller stop?
        self._is_stop_controller = False
        # Starting flag of controller service
        self._is_controller_service = False

        self._enable_emergency_stop = bumper_utils.get_parameter(
            self._node, "enable_emergency_stop", _DEFAULT_ENABLE_EMERGENCY_STOP)

        self._reset_target_controller_list = bumper_utils.get_parameter(
            self._node, "target_controller_list", _TARGET_CONTROLLER_LIST)
        self._stop_target_controller_list = bumper_utils.get_parameter(
            self._node, "stop_target_controller_list", self._reset_target_controller_list)

        self._controller_switcher = self._node.create_client(
            SwitchController,
            'controller_manager/switch_controller',
            callback_group=self._callback_group)

        if self._controller_switcher.wait_for_service(timeout_sec=_ROS_CONN_TIMEOUT):
            self._is_controller_service = True
        else:
            self._node.get_logger().error('Required service is not available.')
            self._is_controller_service = False

        self._reset_service = self._node.create_service(
            Empty, 'bumper/reset', self._reset, callback_group=self._callback_group)

        self._front_bumper_sub = self._node.create_subscription(
            Bool,
            'base_f_bumper_sensor',
            self._front_bumper_callback,
            1, callback_group=self._callback_group)
        self._rear_bumper_sub = self._node.create_subscription(
            Bool,
            'base_b_bumper_sensor',
            self._rear_bumper_callback,
            1, callback_group=self._callback_group)

    def _front_bumper_callback(self, front_bumper_on):
        u"""Callback to get ON and OFF of front bumper

        Args:
          front_bumper_on (Bool): Front bumper condition
        """
        if self._enable_emergency_stop:
            is_stop = self._is_stop_controller
            if front_bumper_on.data and not is_stop:
                is_stop = self._stop_controller()
            with self._lock:
                self._is_bumper_on = self._is_bumper_on or front_bumper_on.data
                self._is_stop_controller = is_stop

    def _rear_bumper_callback(self, rear_bumper_on):
        u"""Callback to get ON and OFF of rear bumper

        Args:
          rear_bumper_on (Bool): Rear bumper condition
        """
        if self._enable_emergency_stop:
            is_stop = self._is_stop_controller
            if rear_bumper_on.data and not is_stop:
                is_stop = self._stop_controller()
            with self._lock:
                self._is_bumper_on = self._is_bumper_on or rear_bumper_on.data
                self._is_stop_controller = is_stop

    def _send_controller_req(self, start_controllers, stop_controllers, strictness):
        req = SwitchController.Request()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = strictness

        event = threading.Event()

        def done_callback(future):
            nonlocal event
            event.set()

        res = self._controller_switcher.call_async(req)
        res.add_done_callback(done_callback)
        event.wait()

        try:
            res = res.result()
        except Exception as e:
            message = f'Service call failed {e}'
            raise bumper_utils.HsrbBumperError(message)
        return res.ok

    def _stop_controller(self):
        u"""Throw Stop service to Controller_manager"""
        is_ok = self._send_controller_req(
            [], self._stop_target_controller_list, _BEST_EFFORT)

        if is_ok:
            self._node.get_logger().info(
                'Succeeded in stopping controller')
            return True
        else:
            self._node.get_logger().error(
                'Service switch_controller returned with failure')
            return False

    def _reset(self, req, response):
        u"""Callback of service that starts the controller"""
        is_ok = self._send_controller_req(
            self._reset_target_controller_list, [], _BEST_EFFORT)

        if is_ok:
            with self._lock:
                self._is_bumper_on = False
                self._is_stop_controller = False
        else:
            raise bumper_utils.HsrbBumperError(
                'Result of controller_manager service call is failed')
        self._node.get_logger().info('Success to start controller')

        return response

    def _get_status(self):
        if self._is_controller_service:
            with self._lock:
                if self._is_bumper_on:
                    if self._is_stop_controller:
                        message = 'All joints are stopped by bumper. '\
                                  'Please press the emergency switch '\
                                  'to reset the system.'
                    else:
                        message = 'Bumper was pushed. But failed to stop the '\
                                  'controller. Press the emergency switch.'
                    return DiagnosticStatus.ERROR, message
                else:
                    message = 'All joints are active'
                    return DiagnosticStatus.OK, message
        else:
            message = 'Required service is not available.'
            self._node.get_logger().error(message)
            return DiagnosticStatus.ERROR, message

    def run(self):
        u"""Execute the target bumper detection mode"""
        rate = self._node.create_rate(_STATE_WATCH_RATE)
        while rclpy.ok():
            diagnostic_status, message = self._get_status()
            self._diag_pub.publish(diagnostic_status, message)
            rate.sleep()
