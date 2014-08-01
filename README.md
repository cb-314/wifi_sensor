# Description of wiwi_sensor

ROS package to use the RSSI of a wifi card as a sensor

## Parameters

* `string` adapter
* `int` channel

## Notes

This has to be run as root because it needs access to raw sockets and to be
able to switch the adapter to monitor mode.
