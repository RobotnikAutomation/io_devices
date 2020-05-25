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

from rcomponent.rcomponent import RComponent # esto podria estar como limites


class VulcanoAirpumpManager(RComponent):
    def __init__(self, args):
        RComponent.__init__(self)
        self.io_service = args['io_service']

        self.pressure_ok_time_hysteresis = args['pressure_ok_time_hysteresis']
        self.pressure_not_ok_time_hysteresis = args['pressure_not_ok_time_hysteresis']
        self.pressure_ok_input_number = args['inputs/pressure_ok/number']
        self.compressor_time_protection = args['compressor_time_protection']
        self.compressor_stop_time_protection = args['compressor_stop_time_protection']

        self.time_with_pressure_ok = 0
        self.time_with_pressure_not_ok = 0
        self.has_pressure_ok  = False

        self.compressor_time_on = 0
        self.compressor_stop_time = self.compressor_stop_time_protection
        self.active_protection = False

        self.air_pump_output_number = args['outputs/air_pump/number']

        self.io_topic = args['io_topic']

        self.io_last_stamp = 0
        self.first_io_cb = True
        self.ready_last_stamp = 0

    def setup(self):
        if self.initialized:
            rospy.logwarn("%s::setup: already initialized" % self.node_name)
            return 0

        rospy.loginfo("%s::setup" % self.node_name)

    def rosSetup(self):
        if self.ros_initialized:
            rospy.logwarn("%s::rosSetup: already initialized" % self.node_name)
            return 0

        RComponent.rosSetup(self)

        # TODO: set the service for testing, remove
        # self.io_srv_handle = rospy.Service(self.io_service, robotnik_msgs.srv.set_digital_output, self.io_srv_callback)

        # wait until the service is ready, otherwise this node is useless
        rospy.wait_for_service(self.io_service)
        self.io_srv = rospy.ServiceProxy(self.io_service, robotnik_msgs.srv.set_digital_output)

        self.io_sub = rospy.Subscriber(self.io_topic, robotnik_msgs.msg.inputs_outputs, self.io_cb, queue_size=1) 

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

    def shutdown(self):
        if self.running:
            rospy.logwarn("%s::shutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.initialized:
            rospy.logwarn("%s::shutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.shutdown(self)

        rospy.loginfo("%s::shutdown"  % self.node_name)

        return 0

    def rosShutdown(self):
        try:
            io_response = self.io_srv(self.air_pump_output_number, False) # stop compressor, just in case
        except rospy.exceptions.ROSInterruptException as e: 
            rospy.logerr('%s::readyState: ROSInterruptException: exiting, do not worry')
        except rospy.service.ServiceException as e:
            rospy.logerr('%s::readyState: ServiceException: That means that I cannot connect to the IO module '% (self.node_name))
        except :
            raise
        
        if self.running:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.ros_initialized:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        RComponent.rosShutdown(self)

        #self.io_data_publisher_.unregister()

    def readyState(self):
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

    def rosPublish(self):
        RComponent.rosPublish(self)

def main():
    rospy.init_node("vulcano_airpump_manager")

    node_name = rospy.get_name().replace('/','')

    arg_defaults = {
      'topic_state': 'state',                   # state of this component
      'desired_freq': 1,
      'io_service': 'robotnik_modbus_io_br/write_digital_output',
      'io_topic': 'robotnik_modbus_io_br/input_output',

      'pressure_ok_time_hysteresis': 60,       # seconds
      'pressure_not_ok_time_hysteresis': 5,    # seconds

      'inputs/pressure_ok/number': 5,       # number of the digital input of the pressure sensor, which will be active if the pressure is ok
      'outputs/air_pump/number': 15,        # number of the digital output of the air pump, activated if the pressure is low
      
      'compressor_time_protection': 600,    # seconds
      'compressor_stop_time_protection' : 60 # seconds
    }

    arg_list = {}

    for arg in arg_defaults:
        try:
            param_name = rospy.search_param(arg)
            if param_name != None:
                arg_list[arg] = rospy.get_param(param_name)
            else:
                arg_list[arg] = arg_defaults[arg]
            print arg, param_name, arg_list[arg]
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, node_name))


    vam = VulcanoAirpumpManager(arg_list)

    try:
        vam.start()
    except Exception as e:
        print 'exception', e
        rospy.signal_shutdown("")

if __name__ == "__main__":
    main()

