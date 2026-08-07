# adsb

- File: `mavros_extras/src/plugins/adsb.cpp`
- Class: `mavros::extra_plugins::ADSBPlugin`
- Namespace: `adsb`
- Brief: ADS-B Vehicle plugin


Publish/subscribe Automatic dependent surveillance-broadcast data to/from a vehicle. Implements the [MAVLink Traffic Management (UTM/ADS-B)](https://mavlink.io/en/services/traffic_management.html).

## Publishers
- `~/vehicle` ([mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html)) - Publish received ADSB_VEHICLE messages (MAVLink traffic management).


## Subscribers
- `~/send` ([mavros_msgs::msg::ADSBVehicle](https://docs.ros.org/en/rolling/p/mavros_msgs/msg/ADSBVehicle.html)) - Subscribe to ADSBVehicle messages to send as ADSB_VEHICLE to the FCU.


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
