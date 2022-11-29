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

from io_manager import IOManager

class SimpleOutputManager(IOManager):
    """
        Class to manage IO Device
    """
    def __init__(self, params, name, node_ns):
        IOManager.__init__(self, params, name, node_ns)

        self.output = self._params['output_number']
        self.desired_output_state = False
        self.last_output_state = False
        self.io_sub = rospy.Subscriber('robotnik_base_hw/io', inputs_outputs, self._io_cb)
        self.output_state_pub = rospy.Publisher('~' + self._name + '/state', Bool, queue_size=10)

    def execute(self):
        if self.desired_output_state == self.last_output_state:
            return

        msg = "Output not in desired state, lets try to switch the output."
        rospy.loginfo_throttle(2, "%s::%s[SimpleOutputManager]::execute: %s" % (self._node_ns, self._name, msg))
        self._set_output()
        return
    
    def publish(self):
        msg = Bool()
        msg.data = self.last_output_state
        self.output_state_pub.publish(msg)

    def _set_value_cb(self, request):
        self.desired_output_state = request.data
        success = self._set_output()
        
        response = SetBoolResponse()
        response.success = success[0]
        response.message = success[1]

        return response
    
    def _set_output(self):
        io_req = set_digital_outputRequest()
        io_req.output = self.output
        io_req.value = self.desired_output_state

        succeeded = False
        msg = ""
        try:
            self.set_output_client.call(io_req)
            succeeded = True
            msg = "Output " + str(self.output)+ " correctly set"
            rospy.loginfo("%s::%s[SimpleOutputManager]::_set_output: %s " % (self._node_ns, self._name, msg))
        except ServiceException as error:
            rospy.logerr_throttle(2,"%s::%s[SimpleOutputManager]::_set_output: %s " % (self._node_ns, self._name, error))
            succeeded = False
            msg = "Error setting output " + str(self.output)

        return (succeeded, msg)        
        
    def _io_cb(self, msg):
        self.last_output_state = msg.digital_outputs[self.output - 1]
