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

from std_srvs.srv import SetBool 
from robotnik_msgs.srv import set_digital_output

class IOManager():
    """
        Class to manage IO Device
    """
    def __init__(self, params, name, node_ns):
        self._params = params
        self._name = name
        self._node_ns = node_ns

        self.set_value_srv = rospy.Service('~' + self._name + '/set_value', SetBool, self._set_value_cb)
        self.set_output_client = rospy.ServiceProxy(self._params['namespace'], set_digital_output)

    def execute(self):
        raise NotImplementedError("Function defined in base clase IOManager but not implemented.")
    
    def publish(self):
        raise NotImplementedError("Function defined in base clase IOManager but not implemented.")
    
    def _set_value_cb(self, request):
        raise NotImplementedError("Function defined in base clase IOManager but not implemented.")
        
