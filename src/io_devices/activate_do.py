#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Robotnik Automation SLL
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

from robotnik_msgs.srv import set_digital_output
from robotnik_msgs.msg import inputs_outputs

from rcomponent.rcomponent import *

class ActivateDO(RComponent):

    def __init__(self):
        self.io_last_stamp = 0
        self.outputs = []
        RComponent.__init__(self)

    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self.io_service = 'robotnik_modbus_io_br/write_digital_output'
        self.io_service = rospy.get_param('~io_service', self.io_service)

        self.io_topic = 'robotnik_modbus_io_br/input_output'
        self.io_topic = rospy.get_param('~io_topic', self.io_topic)

        self.device_output_number = 1
        self.device_output_number = rospy.get_param('~device_output_number', self.device_output_number)
        


    def ros_setup(self):
        RComponent.ros_setup(self)

        # wait until the service is ready, otherwise this node is useless
        rospy.wait_for_service(self.io_service)
        self.io_srv = rospy.ServiceProxy(self.io_service, set_digital_output)

        self.io_sub = rospy.Subscriber(self.io_topic, inputs_outputs, self.io_cb, queue_size=1) 


    def setup(self):
        RComponent.setup(self)

    def shutdown(self):
        RComponent.shutdown(self)    
    
    def switch_to_state(self, new_state):
        return RComponent.switch_to_state(self, new_state)
    
    def ready_state(self):

        if len(self.outputs) == 0:
            return

        try:
            if self.outputs[self.device_output_number - 1] == False:
                self.io_srv(self.device_output_number, True)
                return
        
            if self.outputs[self.device_output_number - 1] == True:
                self.switch_to_state(State.SHUTDOWN_STATE)
                return

        except rospy.service.ServiceException as e:
            rospy.logerr('%s::readyState: ServiceException: That means that I cannot connect to the IO module '% (self.node_name))
        except rospy.exceptions.ROSInterruptException as e: 
            rospy.logerr('%s::readyState: ROSInterruptException: exiting, do not worry')
        except:
            raise

    def io_cb(self, io_msg):

        io_current_stamp = rospy.Time.now()
        if self.io_last_stamp == 0:
            self.io_last_stamp = io_current_stamp
        
        self.outputs = io_msg.digital_outputs

        
def main():

    rospy.init_node("activate_do")
    rc_node = ActivateDO()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()

if __name__ == "__main__":
    main()

