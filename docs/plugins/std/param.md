# param

- File: `mavros/src/plugins/param.cpp`
- Class: `mavros::std_plugins::ParamPlugin`
- Namespace: `param`
- Brief: Parameter manipulation plugin


## Publishers
- `~/event` ([mavros_msgs::msg::ParamEvent](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ParamEvent.html))
- `/parameter_events` ([rcl_interfaces::msg::ParameterEvent](https://docs.ros.org/en/rolling/p/rcl_interfaces/msg/ParameterEvent.html))


## Subscribers
- None


## Services
- `~/pull` ([mavros_msgs::srv::ParamPull](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamPull.html))
- `~/set` ([mavros_msgs::srv::ParamSetV2](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/ParamSetV2.html))
- `~/get_parameters` ([rcl_interfaces::srv::GetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameters.html)) - Standard parameter services
- `~/get_parameter_types` ([rcl_interfaces::srv::GetParameterTypes](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/GetParameterTypes.html))
- `~/set_parameters` ([rcl_interfaces::srv::SetParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParameters.html))
- `~/set_parameters_atomically` ([rcl_interfaces::srv::SetParametersAtomically](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/SetParametersAtomically.html))
- `~/describe_parameters` ([rcl_interfaces::srv::DescribeParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/DescribeParameters.html))
- `~/list_parameters` ([rcl_interfaces::srv::ListParameters](https://docs.ros.org/en/rolling/p/rcl_interfaces/srv/ListParameters.html))


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`PARAM_VALUE`](https://mavlink.io/en/messages/common.html#PARAM_VALUE) [handler: handle_param_value, dialect: common, msg_id: 22, id: `mavlink::common::msg::PARAM_VALUE::MSG_ID`]


## MAVLink Publications
- [`PARAM_REQUEST_LIST`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST) [arg: `rql`, dialect: common, msg_id: 21, id: `mavlink::common::msg::PARAM_REQUEST_LIST::MSG_ID`]
- [`PARAM_REQUEST_READ`](https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ) [arg: `rqr`, dialect: common, msg_id: 20, id: `mavlink::common::msg::PARAM_REQUEST_READ::MSG_ID`]
- [`PARAM_SET`](https://mavlink.io/en/messages/common.html#PARAM_SET) [arg: `ps`, dialect: common, msg_id: 23, id: `mavlink::common::msg::PARAM_SET::MSG_ID`]
