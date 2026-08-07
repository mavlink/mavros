# adsb

- File: `mavros_extras/src/plugins/adsb.cpp`
- Class: `mavros::extra_plugins::ADSBPlugin`
- Namespace: `mavros::extra_plugins`
- Brief: ADS-B Vehicle plugin


Publish/subscribe Automatic dependent surveillance-broadcast data to/from a vehicle. Implements the
[MAVLink Traffic Management (UTM/ADS-B)](https://mavlink.io/en/services/traffic_management.html).

## Publishers
- `~/vehicle` [type: [mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html), qos: [QoS(10)](../qos.md#qos_10_)] - Publish received ADSB_VEHICLE messages (MAVLink traffic management).

## Subscribers
- `~/send` [type: [mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html), qos: [QoS(10)](../qos.md#qos_10_)] - Subscribe to ADSBVehicle messages to send as ADSB_VEHICLE to the FCU.

## Services
- None

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ADSB_VEHICLE`](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) [handler: handle_adsb, dialect: common, msg_id: 246, id: `mavlink::common::msg::ADSB_VEHICLE::MSG_ID`]


## MAVLink Publications
- [`ADSB_VEHICLE`](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) [arg: `msg`, dialect: common, msg_id: 246, id: `mavlink::common::msg::ADSB_VEHICLE::MSG_ID`]
