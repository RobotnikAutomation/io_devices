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
from rcomponent.rcomponent import *


class IODevicesManager(RComponent):

    def __init__(self):
        # Init default values
        
        self.devices = []
        self.device_managers = {}

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.devices = rospy.get_param('~devices', self.devices)


    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.load_devices()

        return 0

    def ready_state(self):
        """Actions performed in ready state"""
        return

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """
        ## DDBBClient stuff here

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def ros_publish(self):
        for device in self.device_managers.keys():
            self.device_managers[device].publish()

    def load_devices(self):
        for device in self.devices:
            if rospy.has_param('~'+device) == False:
                msg = "%s::ros_setup: No parameters found for device '%s'. Abort!"\
                    %(self._node_name, device)
                rospy.logerr(msg)
                rospy.signal_shutdown(msg)
                return
            
            device_manager = self.load_device(device)
            self.device_managers[device] = device_manager

    def load_device(self, device):
        device_params = rospy.get_param('~' + device)
        try:
            type_list = device_params['type'].split('/')
            if len(type_list) == 2:
                package_name = ""
                module_name = type_list[0]
                class_name = type_list[1]
            elif len(type_list) == 3:
                package_name = type_list[0] + "."
                module_name = type_list[1]
                class_name = type_list[2]
            else:
                msg = "%s::load_device: 'type' parameter must be defined by " \
                    + "package_name/module_name/class_name or just module_name/class_name"\
                    + " if the device manager that you want to load is defined " \
                    + "in this package. Current 'type' parameter is '%s' for " \
                    + "the device '%s'"
                msg = msg % (self._node_name, device_params['type'], device)
                rospy.logerr(msg)
                rospy.signal_shutdown(msg)
            
            module_import_name = package_name + "devices." + module_name
            module = __import__(module_import_name, fromlist=['object'])
            object_type = getattr(module, class_name)

            return object_type(device_params, device, self._node_name)


        except KeyError as key_error:
            msg = "%s::load_device: required %s parameter not found for '%s' device manager."\
                 %(self._node_name, key_error, device)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return
        
        except ImportError as import_error:
            msg = "%s::load_device: %s" %(self._node_name, import_error)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return
        
        except AttributeError as attribute_error:
            msg = "%s::load_device: %s" %(self._node_name, attribute_error)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            return