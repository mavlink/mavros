# adsb

- File: `mavros_extras/src/plugins/adsb.cpp`
- Class: `mavros::extra_plugins::ADSBPlugin`
- Namespace: `adsb`
- Brief: ADS-B Vehicle plugin


Publish/subscribe Automatic dependent surveillance-broadcast data to/from a vehicle.

## Publishers
- `~/vehicle` ([mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html))


## Subscribers
- `~/send` ([mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html))


## Services
- None


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`ADSB_VEHICLE`](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) [handler: handle_adsb, dialect: common, msg_id: 246, id: `mavlink::common::msg::ADSB_VEHICLE::MSG_ID`]


## MAVLink Publications
- [`ADSB_VEHICLE`](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) [arg: `adsb`, dialect: common, msg_id: 246, id: `mavlink::common::msg::ADSB_VEHICLE::MSG_ID`]
