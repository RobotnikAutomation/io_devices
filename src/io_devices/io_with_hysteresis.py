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

class IOWithHysteresis(RComponent):

    def __init__(self):

        self.time_with_signal_ok = 0
        self.time_with_signal_not_ok = 0
        self.has_signal_ok  = False
        self.output_time_on = 0
        self.active_protection = False
        self.io_last_stamp = 0
        self.first_io_cb = True
        self.ready_last_stamp = 0

        RComponent.__init__(self)

    def ros_read_params(self):
        RComponent.ros_read_params(self)

        self.io_service = 'robotnik_modbus_io_br/write_digital_output'
        self.io_service = rospy.get_param('~io_service', self.io_service)

        self.io_topic = 'robotnik_modbus_io_br/input_output'
        self.io_topic = rospy.get_param('~io_topic', self.io_topic)

        self.signal_ok_time_hysteresis = 60
        self.signal_ok_time_hysteresis = rospy.get_param('~signal_ok_time_hysteresis', self.signal_ok_time_hysteresis)

        self.signal_not_ok_time_hysteresis = 5
        self.signal_not_ok_time_hysteresis = rospy.get_param('~signal_not_ok_time_hysteresis', self.signal_not_ok_time_hysteresis)

        self.signal_ok_input_number = 5
        self.signal_ok_input_number = rospy.get_param('~signal_ok_input_number', self.signal_ok_input_number)
        
        self.device_max_time_active = 600
        self.device_max_time_active = rospy.get_param('~device_max_time_active', self.device_max_time_active)
        
        self.device_stop_time_protection = 60
        self.device_stop_time_protection = rospy.get_param('~device_stop_time_protection', self.device_stop_time_protection)
        self.device_stop_time = self.device_stop_time_protection
        
        self.device_output_number = 15
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
        ready_current_stamp = rospy.Time.now()
        if self.ready_last_stamp == 0:
            self.ready_last_stamp = ready_current_stamp
        
        time_elapsed = (ready_current_stamp - self.ready_last_stamp).to_sec()
        self.ready_last_stamp = ready_current_stamp

        try:
            if self.output_time_on > self.device_max_time_active:
                rospy.logwarn("%s::ready_state: Turn on protection!" % (self._node_name))
                self.active_protection = True
                self.device_stop_time = self.device_stop_time_protection
                self.output_time_on = 0

            if self.active_protection and self.device_stop_time < 0:
                rospy.logwarn("%s::ready_state: Turn off protection!" % (self._node_name))
                self.active_protection = False
                self.output_time_on = 0
                self.device_stop_time = 0 

            if self.active_protection == True:
                io_response = self.io_srv(self.device_output_number, False)
                self.device_stop_time -= time_elapsed
                
                time_stopped = self.device_stop_time_protection - self.device_stop_time
                rospy.logwarn("%s::ready_state: Protection active during %f" % (self._node_name, time_stopped))
            
            elif self.has_signal_ok == True:
                rospy.loginfo_throttle(5, "%s::ready_state: Signal ok active" % (self._node_name))
                io_response = self.io_srv(self.device_output_number, False)
                self.output_time_on -= time_elapsed
                if self.output_time_on < 0:
                    self.output_time_on = 0
            else:
                io_response = self.io_srv(self.device_output_number, True)
                self.output_time_on += time_elapsed

                rospy.logwarn_throttle(5, "%s::ready_state: Signal ok not active" % (self._node_name))
                rospy.loginfo_throttle(1, "%s::ready_state: Device active during %f seconds" % (self._node_name, self.output_time_on))


        except rospy.service.ServiceException as e:
            rospy.logerr('%s::readyState: ServiceException: That means that I cannot connect to the IO module '% (self._node_name))
        except rospy.exceptions.ROSInterruptException as e: 
            rospy.logerr('%s::readyState: ROSInterruptException: exiting, do not worry')
        except:
            raise


    def io_srv_callback(self, req):
        rospy.loginfo('set %d to %r' % (req.output, req.value))
        return robotnik_msgs.srv.set_digital_outputResponse(True)

    def io_cb(self, io_msg):

        io_current_stamp = rospy.Time.now()
        if self.io_last_stamp == 0:
            self.io_last_stamp = io_current_stamp

        time_elapsed = (io_current_stamp - self.io_last_stamp).to_sec()
        signal_ok = io_msg.digital_inputs[self.signal_ok_input_number-1]

        if signal_ok == True:
            if self.first_io_cb == True:
                self.time_with_signal_ok = self.signal_ok_time_hysteresis
                self.has_signal_ok = True
            self.time_with_signal_ok += time_elapsed    
            self.time_with_signal_not_ok -= time_elapsed
            #print 'pressure ok' 
        else:
            self.time_with_signal_not_ok += time_elapsed
            self.time_with_signal_ok -= time_elapsed    
            #print 'pressure not ok'

        if self.time_with_signal_ok > self.signal_ok_time_hysteresis:
            self.time_with_signal_ok = self.signal_ok_time_hysteresis
            self.time_with_signal_not_ok = 0
            self.has_signal_ok = True
            #print 'has_signal_ok'
        elif self.time_with_signal_not_ok > self.signal_not_ok_time_hysteresis:
            self.time_with_signal_not_ok = self.signal_not_ok_time_hysteresis
            self.time_with_signal_ok = 0
            self.has_signal_ok = False
            #print 'has_pressure_not_ok'

        self.io_last_stamp = io_current_stamp
        
        if self.first_io_cb == True:
            self.first_io_cb = False


def main():

    rospy.init_node("io_with_hysteresis")
    rc_node = IOWithHysteresis()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()

if __name__ == "__main__":
    main()

