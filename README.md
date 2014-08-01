# Description of wifi_sensor

ROS package to use the RSSI of a wifi card as a sensor

## Parameters

* `string` adapter
* `int` channel
* `int` rate (Hz)

## Notes

This has to be run as root because it needs access to raw sockets and to be
able to switch the adapter to monitor mode. One way to do this, is to use the
`<machine>` xml tag for roslaunch and log in as root and then use the `env.sh`
script to start it with the correct environment.
