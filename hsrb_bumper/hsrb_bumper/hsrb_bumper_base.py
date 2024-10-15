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
"""Hsrb_bumper interfaces."""
from abc import abstractmethod

from diagnostic_msgs.msg import DiagnosticStatus


class HsrbBumperBase(object):
    """Provide behavior bumper collision detection"""

    @abstractmethod
    def run(self):
        """Execute monitoring of bumper collision detection"""
        return (DiagnosticStatus.OK, "hsrb_bumper")

    @abstractmethod
    def front_bumper_callback(self, front_bumper_on):
        """Callback when collision of front bumper is detected

        Args:
            front_bumper_on (bool):
              When the front bumper collides, front_bumper_on is ``True``
        """
        pass

    @abstractmethod
    def rear_bumper_callback(self, rear_bumper_on):
        """Callback when collision of rear bumper is detected

        Args:
            rear_bumper_on (bool):
               When the rear bumper collides, rear_bumper_on is ``True``
        """
        pass

    @abstractmethod
    def shotdown_service(self):
        """Service servers is shutdown when bumper mode is switched"""
        pass
