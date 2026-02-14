# param

- File: `mavros/src/plugins/param.cpp`
- Class: `mavros::std_plugins::ParamPlugin`
- Namespace: `param`
- Brief: Parameter manipulation plugin


## Publishers
- `~/event` (mavros_msgs::msg::ParamEvent)
- `/parameter_events` (rcl_interfaces::msg::ParameterEvent)


## Subscribers
- None


## Services
- `~/pull` (mavros_msgs::srv::ParamPull)
- `~/set` (mavros_msgs::srv::ParamSetV2)
- `~/get_parameters` (rcl_interfaces::srv::GetParameters) - Standard parameter services
- `~/get_parameter_types` (rcl_interfaces::srv::GetParameterTypes)
- `~/set_parameters` (rcl_interfaces::srv::SetParameters)
- `~/set_parameters_atomically` (rcl_interfaces::srv::SetParametersAtomically)
- `~/describe_parameters` (rcl_interfaces::srv::DescribeParameters)
- `~/list_parameters` (rcl_interfaces::srv::ListParameters)


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- `PARAM_VALUE` [handler: handle_param_value, dialect: common, msg_id: 22, id: `mavlink::common::msg::PARAM_VALUE::MSG_ID`]


## MAVLink Publications
- `PARAM_REQUEST_LIST` [arg: `rql`, dialect: common, msg_id: 21, id: `mavlink::common::msg::PARAM_REQUEST_LIST::MSG_ID`]
- `PARAM_REQUEST_READ` [arg: `rqr`, dialect: common, msg_id: 20, id: `mavlink::common::msg::PARAM_REQUEST_READ::MSG_ID`]
- `PARAM_SET` [arg: `ps`, dialect: common, msg_id: 23, id: `mavlink::common::msg::PARAM_SET::MSG_ID`]
