# panther_crsf_teleop

This ROS 2 package allows you to control a Panther robot using a CRSF compatible remote control. Receiver should be connected to the robot's computer via USB-UART converter. CRSF is implemented based on the [protocol specification](https://github.com/crsf-wg/crsf/wiki).

## Installation

```ros2
cd ros_ws/src
git clone git@github.com:miloszlagan/panther_crsf_teleop.git
colcon build --packages-select panther_crsf_teleop --symlink-install
```

## Usage

```ros2
ros2 run panther_crsf_teleop panther_crsf_teleop
```

Optional arguments:
- `serial_port` - serial port to which the CRSF receiver is connected (default: `/dev/ttyUSB0`)
- `baudrate` - baudrate of the serial port (default: `576000`)

Node publishes:
- `/cmd_vel` - geometry_msgs/Twist - velocity commands for the robot
- `/hardware/e_stop` - std_msgs/Bool - emergency stop state
- `/link_status` - panther_crsf_teleop_msgs/LinkStatus - link status between the remote control and the robot. See [CRSF_FRAMETYPE_LINK_STATISTICS](https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS)
