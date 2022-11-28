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

from std_msgs.msg import Bool
from std_srvs.srv import SetBoolResponse 

from robotnik_msgs.srv import set_digital_outputRequest
from robotnik_msgs.msg import inputs_outputs

from robotnik_signal_msgs.srv import SetSignal, SetSignalResponse
from robotnik_signal_msgs.msg import SignalStatus

from io_manager import IOManager

class SignalOutputManager(IOManager):
    """
        Class to manage IO Device
    """
    def __init__(self, params, name, node_ns):
        IOManager.__init__(self, params, name, node_ns)
        self.output = self._params['output_number']

        self.last_output_state = False
        self.io_sub = rospy.Subscriber('robotnik_base_hw/io', inputs_outputs, self.io_cb)
        self.output_state_pub = rospy.Publisher('~' + self._name + '/state', Bool, queue_size=10)

        # Signals stuff
        self.signal_srv = rospy.Service('~' + self._name + '/set_signal', SetSignal, self.set_signal_cb)
        self.signal_status_pub = rospy.Publisher('~' + self._name + '/status', SignalStatus, queue_size=10)

        self.available_signals = []
        for signal in self._params['signals']:
            self.available_signals.append(signal['id'])

        self.active_signals = []
        self.desired_signal = ""

    def set_value_cb(self, request):
        io_req = self.__build_io_request(request.data)

        try:
            self.set_output_client.call(io_req)    
        except ServiceException as error:
            response = SetBoolResponse()
            response.message = "Error setting output"
            response.success = False
            rospy.logerr("%s::%s::set_value_cb: %s " % (self._node_ns, self._name, error))
            return response


        response = SetBoolResponse()
        msg = ""
        if request.data == True:
            msg = "Output " +str(self.output)+ " set to True"
        else:
            msg = "Output " +str(self.output)+ " set to False"
        
        rospy.loginfo("%s::%s::set_value_cb: %s " % (self._node_ns, self._name, msg))
        response.message = msg
        response.success = True
        return response
    
    def io_cb(self, msg):
        self.last_output_state = msg.digital_outputs[self.output - 1]

    def set_signal_cb(self, msg):
        signal = msg.signal_id

        response = SetSignalResponse()

        # No activating signal if not a available signal
        if signal not in self.available_signals:
            response = SetSignalResponse()
            response.ret.message = "Signal '" +signal+ " not included in available signals"
            response.ret.success = False
            self.active_signals = []

            io_req = self.__build_io_request(False)
            self.set_output_client.call(io_req)

            
            return response
        
        # No activating singal if it is already active
        if signal in self.active_signals:
            response.ret.message = "Signal '" +signal+ " already active"
            response.ret.success = True
            return response
        
        io_req = self.__build_io_request(msg.enable)

        try:
            self.set_output_client.call(io_req)
            
            response.ret.message = "Output set correctly"
            response.ret.success = True
            self.active_signals = [signal]

        except ServiceException as error:
            response.ret.message = "Error setting output"
            response.ret.success = False
            self.active_signals = []
            rospy.logerr("%s::%s::set_signal_cb: %s " % (self._node_ns, self._name, error))

        return response

    def publish(self):
        msg = Bool(self.last_output_state)
        self.output_state_pub.publish(msg)

        msg = SignalStatus("io_devices/beacon", self.active_signals)
        self.signal_status_pub.publish(msg)


    def __build_io_request(self, value):
        io_req = set_digital_outputRequest()
        io_req.output = self.output
        io_req.value = value
        return  io_req
