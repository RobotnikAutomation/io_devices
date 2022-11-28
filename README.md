# io_devices
Package to interact with different components using IOs

## Installation

* Install robotnik dependencies:
    * robotnik_msgs
    * rcomponent

## Parameters

* **devices**: List with the name of the different device managers to load.

    ```yaml
    devices:
    - buzzer
    - front_light
    - left_light
    - beacon
    ```
### Devices

For each device defined in the devices list is required to define its parameters:
* **type**: The name of the Python module and class
* **namespace**: Name of the server to interact with IOs

```yaml
buzzer:
  type: simple_output_manager/SimpleOutputManager
  namespace: robotnik_base_hw/set_digital_output
  output_number: 4

front_light:
  type: simple_output_manager/SimpleOutputManager
  namespace: robotnik_base_hw/set_digital_output
  output_number: 5

left_light:
  type: simple_output_manager/SimpleOutputManager
  namespace: robotnik_base_hw/set_digital_output
  output_number: 6

beacon:
  type: simple_output_manager/SimpleOutputManager
  namespace: robotnik_base_hw/set_digital_output
  output_number: 7
```

## Services

For each configured device, a service server is started to interact with the device. The behavior of the service will be defined by the type of the manager.

### simple_output_manager

* ~/<device_name>/set_value (std_srvs/SetBool): Sets the value of the output

## Launch

```bash
roslaunch io_device io_device.launch
```