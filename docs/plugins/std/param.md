# param

- File: `mavros/src/plugins/param.cpp`
- Class: `mavros::std_plugins::ParamPlugin`
- Namespace: `mavros::std_plugins`
- Brief: Parameter manipulation plugin


Implements the
[MAVLink Parameter Protocol](https://mavlink.io/en/services/parameter.html).

## Publishers
- `~/event` [type: [mavros_msgs::msg::ParamEvent](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ParamEvent.html), qos: [ParameterEventsQoS](../qos.md#parametereventsqos "ParameterEventsQoS QoS profile")] - Parameter change notifications (new/updated/changed).
- `PSN::events` [type: [rcl_interfaces::msg::ParameterEvent](https://docs.ros.org/en/rolling/p/rcl_interfaces/msg/ParameterEvent.html), qos: [ParameterEventsQoS](../qos.md#parametereventsqos "ParameterEventsQoS QoS profile")] - Standard ROS parameter events (on /parameter_events).

## Subscribers
- None

## Services
- `~/pull` [type: [mavros_msgs::srv::ParamPull](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamPull.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Custom parameter services Fetch all parameters from the device (PARAM_REQUEST_LIST).
- `~/set` [type: [mavros_msgs::srv::ParamSetV2](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamSetV2.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Set a single parameter value (PARAM_SET).
- `PSN::get_parameters` [type: [rcl_interfaces::srv::GetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameters.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Get parameter values from the local cache.
- `PSN::get_parameter_types` [type: [rcl_interfaces::srv::GetParameterTypes](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameterTypes.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Get parameter types from the local cache.
- `PSN::set_parameters` [type: [rcl_interfaces::srv::SetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParameters.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Set parameter values (PARAM_SET for each).
- `PSN::set_parameters_atomically` [type: [rcl_interfaces::srv::SetParametersAtomically](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParametersAtomically.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Unsupported: device-side atomic set, always reports failure.
- `PSN::describe_parameters` [type: [rcl_interfaces::srv::DescribeParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/DescribeParameters.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - Describe parameter descriptors from the local cache.
- `PSN::list_parameters` [type: [rcl_interfaces::srv::ListParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/ListParameters.html), qos: [ParametersQoS](../qos.md#parametersqos "ParametersQoS QoS profile")] - List parameter names from the local cache.

## Clients
- None


## Parameters
- `param_set_timeout` [type: double, default: `param_set_timeout.seconds()`] - Timeout for a single PARAM_SET retry (seconds).
- `param_list_timeout` [type: double, default: `param_list_timeout.seconds()`] - Timeout waiting for a full parameter list pull (seconds).
- `param_retries` [default: `param_retries_count`] - Number of retries before reporting a parameter operation as failed.


## MAVLink Subscriptions
- [`PARAM_VALUE`](https://mavlink.io/en/messages/common.html#PARAM_VALUE) [handler: handle_param_value, dialect: common, msg_id: 22, id: `mavlink::common::msg::PARAM_VALUE::MSG_ID`]


## MAVLink Publications
- [`PARAM_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST) [arg: `msg`, dialect: common, msg_id: 21, id: `mavlink::common::msg::PARAM_REQUEST_LIST::MSG_ID`]
- [`PARAM_REQUEST_READ`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ) [arg: `msg`, dialect: common, msg_id: 20, id: `mavlink::common::msg::PARAM_REQUEST_READ::MSG_ID`]
- [`PARAM_SET`](https://mavlink.io/en/messages/common.html#PARAM_SET) [arg: `msg`, dialect: common, msg_id: 23, id: `mavlink::common::msg::PARAM_SET::MSG_ID`]
