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

import sys
import time, threading

import rospy
import nav_msgs.msg
import robotnik_msgs.msg
import robotnik_msgs.srv
import std_msgs.msg

from rcomponent.rcomponent import *

class IOWithHysteresis(RComponent):

    def __init__(self):

        self.time_with_pressure_ok = 0
        self.time_with_pressure_not_ok = 0
        self.has_pressure_ok  = False
        self.compressor_time_on = 0
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

        self.pressure_ok_time_hysteresis = 60
        self.pressure_ok_time_hysteresis = rospy.get_param('~pressure_ok_time_hysteresis', self.pressure_ok_time_hysteresis)

        self.pressure_not_ok_time_hysteresis = 5
        self.pressure_not_ok_time_hysteresis = rospy.get_param('~pressure_not_ok_time_hysteresis', self.pressure_not_ok_time_hysteresis)

        self.pressure_ok_input_number = 5
        self.pressure_ok_input_number = rospy.get_param('~inputs/pressure_ok/number', self.pressure_ok_input_number)
        
        self.compressor_time_protection = 600
        self.compressor_time_protection = rospy.get_param('~compressor_time_protection', self.compressor_time_protection)
        
        self.compressor_stop_time_protection = 60
        self.compressor_stop_time_protection = rospy.get_param('~compressor_stop_time_protection', self.compressor_stop_time_protection)
        self.compressor_stop_time = self.compressor_stop_time_protection
        
        self.air_pump_output_number = 15
        self.air_pump_output_number = rospy.get_param('~outputs/air_pump/number', self.air_pump_output_number)
        


    def ros_setup(self):
        RComponent.ros_setup(self)

        # wait until the service is ready, otherwise this node is useless
        rospy.wait_for_service(self.io_service)
        self.io_srv = rospy.ServiceProxy(self.io_service, robotnik_msgs.srv.set_digital_output)

        self.io_sub = rospy.Subscriber(self.io_topic, robotnik_msgs.msg.inputs_outputs, self.io_cb, queue_size=1) 


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
            if self.compressor_time_on > self.compressor_time_protection:
                self.active_protection = True
                self.compressor_stop_time = self.compressor_stop_time_protection
                self.compressor_time_on = 0

            if self.active_protection and self.compressor_stop_time < 0:
                self.active_protection = False
                self.compressor_time_on = 0
                self.compressor_stop_time = 0 

            if self.active_protection == True:
                io_response = self.io_srv(self.air_pump_output_number, False)
                self.compressor_stop_time -= time_elapsed
            elif self.has_pressure_ok == True:
                io_response = self.io_srv(self.air_pump_output_number, False)
                self.compressor_time_on -= time_elapsed
                if self.compressor_time_on < 0:
                    self.compressor_time_on = 0
            else:
                io_response = self.io_srv(self.air_pump_output_number, True)
                self.compressor_time_on += time_elapsed

        except rospy.service.ServiceException as e:
            rospy.logerr('%s::readyState: ServiceException: That means that I cannot connect to the IO module '% (self.node_name))
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
        pressure_ok = io_msg.digital_inputs[self.pressure_ok_input_number-1]

        if pressure_ok == True:
            if self.first_io_cb == True:
                self.time_with_pressure_ok = self.pressure_ok_time_hysteresis
                self.has_pressure_ok = True
            self.time_with_pressure_ok += time_elapsed    
            self.time_with_pressure_not_ok -= time_elapsed
            #print 'pressure ok' 
        else:
            self.time_with_pressure_not_ok += time_elapsed
            self.time_with_pressure_ok -= time_elapsed    
            #print 'pressure not ok'

        if self.time_with_pressure_ok > self.pressure_ok_time_hysteresis:
            self.time_with_pressure_ok = self.pressure_ok_time_hysteresis
            self.time_with_pressure_not_ok = 0
            self.has_pressure_ok = True
            #print 'has_pressure_ok'
        elif self.time_with_pressure_not_ok > self.pressure_not_ok_time_hysteresis:
            self.time_with_pressure_not_ok = self.pressure_not_ok_time_hysteresis
            self.time_with_pressure_ok = 0
            self.has_pressure_ok = False
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

