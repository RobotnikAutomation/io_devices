#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python
import rospy
from rospy.service import ServiceException

from robotnik_signal_msgs.srv import SetSignal, SetSignalResponse
from robotnik_signal_msgs.msg import SignalStatus

from single_output_manager import SingleOutputManager

class SignalSingleOutputManager(SingleOutputManager):
    """
        Class to manage IO Device
    """
    def __init__(self, params, name, node_ns):
        SingleOutputManager.__init__(self, params, name, node_ns)
        
        # Signals stuff
        self.signal_srv = rospy.Service('~' + self._name + '/set_signal', SetSignal, self._set_signal_cb)
        self.signal_status_pub = rospy.Publisher('~' + self._name + '/status', SignalStatus, queue_size=10)

        self.available_signals = []
        for signal in self._params['signals']:
            self.available_signals.append(signal['id'])

        self.active_signals = []
        self.desired_signal = ""


    def execute(self):
        SingleOutputManager.execute(self)

        if self.desired_output_state == self.last_output_state:
            self.active_signals = [self.desired_signal]

        return
    
    def publish(self):
        SingleOutputManager.publish(self)

        msg = SignalStatus()
        msg.node_name = "io_devices/" + self._name
        msg.active_signals = self.active_signals
        self.signal_status_pub.publish(msg)
        return

    def _set_signal_cb(self, request):
        signal = request.signal_id
        enable = request.enable
        
        response = SetSignalResponse()
        if signal not in self.available_signals:
            msg = "Ignoring signal '" + signal + "'. Not included in target signals"
            response.ret.success = False
            response.ret.message = msg
            rospy.logerr_throttle(2,"%s::%s[SignalSingleOutputManager]::_set_signal_cb: %s " \
                % (self._node_ns, self._name, msg))
            return response

        self.des
        self.desired_output_state = request.data
        success = self._set_output()
        
        response = SetBoolResponse()
        response.success = success[0]
        response.message = success[1]

        return 

