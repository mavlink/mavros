# param

- File: `mavros/src/plugins/param.cpp`
- Class: `mavros::std_plugins::ParamPlugin`
- Namespace: `param`
- Brief: Parameter manipulation plugin


Implements the [MAVLink Parameter Protocol](https://mavlink.io/en/services/parameter.html).

## Publishers
- `~/event` ([mavros_msgs::msg::ParamEvent](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ParamEvent.html)) - Parameter change notifications (new/updated/changed).
- `/parameter_events` ([rcl_interfaces::msg::ParameterEvent](https://docs.ros.org/en/rolling/p/rcl_interfaces/msg/ParameterEvent.html)) - Standard ROS parameter events (on /parameter_events).


## Subscribers
- None


## Services
- `~/pull` ([mavros_msgs::srv::ParamPull](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamPull.html)) - Custom parameter services Fetch all parameters from the device (PARAM_REQUEST_LIST).
- `~/set` ([mavros_msgs::srv::ParamSetV2](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamSetV2.html)) - Set a single parameter value (PARAM_SET).
- `~/get_parameters` ([rcl_interfaces::srv::GetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameters.html)) - Standard parameter services Get parameter values from the local cache.
- `~/get_parameter_types` ([rcl_interfaces::srv::GetParameterTypes](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameterTypes.html)) - Get parameter types from the local cache.
- `~/set_parameters` ([rcl_interfaces::srv::SetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParameters.html)) - Set parameter values (PARAM_SET for each).
- `~/set_parameters_atomically` ([rcl_interfaces::srv::SetParametersAtomically](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParametersAtomically.html)) - Unsupported: device-side atomic set, always reports failure.
- `~/describe_parameters` ([rcl_interfaces::srv::DescribeParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/DescribeParameters.html)) - Describe parameter descriptors from the local cache.
- `~/list_parameters` ([rcl_interfaces::srv::ListParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/ListParameters.html)) - List parameter names from the local cache.


## Clients
- None


## Parameters
- `param_set_timeout` [type: double, default: `param_set_timeout.seconds()`] - Timeout for a single PARAM_SET retry (seconds).
- `param_list_timeout` [type: double, default: `param_list_timeout.seconds()`] - Timeout waiting for a full parameter list pull (seconds).
- `param_retries` [default: `param_retries_count`] - Number of retries before reporting a parameter operation as failed.


## MAVLink Subscriptions
- [`PARAM_VALUE`](https://mavlink.io/en/messages/common.html#PARAM_VALUE) [handler: handle_param_value, dialect: common, msg_id: 22, id: `mavlink::common::msg::PARAM_VALUE::MSG_ID`]


## MAVLink Publications
- [`PARAM_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST) [arg: `rql`, dialect: common, msg_id: 21, id: `mavlink::common::msg::PARAM_REQUEST_LIST::MSG_ID`]
- [`PARAM_REQUEST_READ`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ) [arg: `rqr`, dialect: common, msg_id: 20, id: `mavlink::common::msg::PARAM_REQUEST_READ::MSG_ID`]
- [`PARAM_SET`](https://mavlink.io/en/messages/common.html#PARAM_SET) [arg: `ps`, dialect: common, msg_id: 23, id: `mavlink::common::msg::PARAM_SET::MSG_ID`]
