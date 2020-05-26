# IO Devices

Package to manage devices using inputs and outputs.

## Dependencies

* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs)

* [rcomponent](https://github.com/RobotnikAutomation/rcomponent)

## io_with_hysteresis

### Params

* **io_service** (string): Service name to set the digital output.
  
* **io_topic** (string): Topic name to read the input status

* **signal_ok_input_number** (int): Digital input that shows if the device is ok. If not ok, the device needs to be activated using the *device_output_number*.

* **signal_ok_time_hysteresis** (int): Time in seconds. The output of the device will remain active until this time has passed since the input of signal ok is active.
  
* **signal_not_ok_time_hysteresis** (int): Time in seconds. The output of the device will be activated when the input of signal ok has been inactive for this time

* **device_max_time_active** (int): Time in seconds. If the device is active for more than this value, it will be truned off.

* **device_stop_time_protection** (int): Time in seconds. Minimum time that the device will still stop after a protective stop caused by *device_max_time_active*.

* **device_output_number** (int): Digital output that turn on/off the device.
        