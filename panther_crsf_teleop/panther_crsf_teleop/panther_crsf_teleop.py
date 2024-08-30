import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from panther_crsf_interfaces.msg import LinkStatus

import serial
from .crsf.parser import CRSFParser, unpack_channels
from .crsf.message import CRSFMessage, PacketType


class CRSFInterface(Node):
    def __init__(self):
        super().__init__('crsf_interface')
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1
            )
        )
        self.e_stop_publisher = self.create_publisher(
            Bool,
            'hardware/e_stop',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        # self.gps_fix_subscription = self.create_subscription(
        #     NavSatFix,
        #     'gps/fix',
        #     lambda msg: self.send_gps_fix(msg),
        #     QoSProfile(
        #         reliability=QoSReliabilityPolicy.RELIABLE,
        #         durability=QoSDurabilityPolicy.VOLATILE,
        #         depth=1
        #     )
        # )
        
        self.link_status_publisher = self.create_publisher(
            LinkStatus,
            'link_status',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1
            )
        )
        
        self.link_status = LinkStatus()
        
        self.declare_parameter('port', '/dev/ttyUSB0', ParameterDescriptor(description='CRSF receiver serial port'))
        self.declare_parameter('baud', 576000, ParameterDescriptor(description='CRSF receiver baud rate'))
        
        # Initialize serial port
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.serial = serial.Serial(port, baud, timeout=2)
        
        # Setup CRSF parser
        self.parser = CRSFParser()
        self.parser.on_message = lambda msg: self.handle_message(msg)
        
        self.timer = self.create_timer(0.01, self.run)
        # self.telemetry_timer = self.create_timer(1.0, self.senf_periodic_telemetry)
        
        self.rc_estop_state = Bool(data=True)
        self.e_stop_republisher = self.create_timer(1, self.republish_e_stop)
        
    def run(self):
        if self.serial.in_waiting > 0:
            self.parser.parse(self.serial.read(self.serial.in_waiting))
                
    def handle_message(self, msg: CRSFMessage):
        if msg.msg_type == PacketType.RC_CHANNELS_PACKED:
            channels = unpack_channels(msg.payload)
            
            # Map channels from CRSF range [172, 1812] to [-1, 1]
            channels = [(channel - 992)/ 820.0 for channel in channels]
            
            # Handle emergency stop from RC controler
            # Asserted e-stop is retransmitted once per second by republish timer
            # Deasserted e-stop is transmitted only once
            requested_e_stop = channels[4] < 0.5
            if requested_e_stop != self.rc_estop_state.data:
                self.rc_estop_state.data = requested_e_stop
                self.e_stop_publisher.publish(self.rc_estop_state)
            
            # Disable sending cmd_vel if override switch is asserted
            send_cmd_vel = channels[6] < -0.5
            
            if send_cmd_vel:
                t = Twist()
                t.linear.x = channels[1]
                t.angular.z = -channels[3]
                
                self.cmd_vel_publisher.publish(t)
        
        elif msg.msg_type == PacketType.LINK_STATISTICS:
            last_lq = self.link_status.lq
            
            self.link_status.rssi_1 = -msg.payload[0]
            self.link_status.rssi_2 = -msg.payload[1]
            self.link_status.lq = msg.payload[2]
            self.link_status.uplink_snr = int.from_bytes(bytes(msg.payload[3]), byteorder='big', signed=True)
            self.link_status.used_antenna = msg.payload[4]
            self.link_status.mode = msg.payload[5]
            self.link_status.tx_power = msg.payload[6]
            self.link_status.downlink_rssi = -msg.payload[7]
            self.link_status.downlink_lq = msg.payload[8]
            self.link_status.downlink_snr = int.from_bytes(bytes(msg.payload[9]), byteorder='big', signed=True)
            
            # Detect connection status
            if last_lq != 0 and self.link_status.lq == 0:
                self.get_logger().error("Connection lost")
                
            if last_lq == 0 and self.link_status.lq > 0:
                self.get_logger().info("Connected")
            
            # Warn on low link quality
            if last_lq != self.link_status.lq:
                if self.link_status.lq < 15:
                    self.get_logger().warn(f"Very low link quality: {self.link_status.lq}%")
                elif self.link_status.lq < 30:
                    self.get_logger().warn(f"Low link quality: {self.link_status.lq}%")
                elif last_lq < 30 and self.link_status.lq >= 30:
                    self.get_logger().info(f"Link quality restored: {self.link_status.lq}%")
            
            self.link_status_publisher.publish(self.link_status)
            
        else:
            self.get_logger().warn(f"Unknown CRSF message (Type: {msg.msg_type.name}, Length: {msg.length})")
            
    def republish_e_stop(self):
        if self.rc_estop_state.data:
            self.e_stop_publisher.publish(self.rc_estop_state)
            
    def send_gps_fix(self, fix: NavSatFix):
        self.get_logger().info(f"Sending GPS fix: {fix.latitude}, {fix.longitude}")
            
        gps_msg = CRSFMessage()
        gps_msg.msg_type = PacketType.GPS
        
        # Encoded in degrees * 1e7, big-endian
        gps_msg.payload.extend(int.to_bytes(int(fix.latitude * 1e7), 4, byteorder='big', signed=True))
        gps_msg.payload.extend(int.to_bytes(int(fix.longitude * 1e7), 4, byteorder='big', signed=True))
        
        # No ground speed
        gps_msg.payload.extend(int.to_bytes(0, 2, byteorder='big', signed=True))
        
        # No heading
        gps_msg.payload.extend(int.to_bytes(0, 2, byteorder='big', signed=True))
        
        # Altitude in meters, big-endian
        gps_msg.payload.extend(int.to_bytes(int(fix.altitude) + 1000, 2, byteorder='big', signed=True))
        
        # No sats
        gps_msg.payload.extend(int.to_bytes(10, 1, byteorder='big', signed=True))
        
        self.serial.write(gps_msg.encode())
        
    def senf_periodic_telemetry(self):
        msg = CRSFMessage()
        msg.msg_type = PacketType.FLIGHT_MODE
        msg.payload.extend(map(ord, 'ROV\0'))
        
        self.serial.write(msg.encode())
        
        msg = CRSFMessage()
        msg.msg_type = PacketType.BATTERY_SENSOR
        msg.payload.extend(int.to_bytes(1, 2, byteorder='big', signed=True))
        msg.payload.extend(int.to_bytes(2, 2, byteorder='big', signed=True))
        msg.payload.extend(int.to_bytes(3, 3, byteorder='big', signed=True))
        msg.payload.extend(int.to_bytes(4, 1, byteorder='big', signed=True))
        
        self.serial.write(msg.encode())


def main(args=None):
    rclpy.init(args=args)

    crsf_interface = CRSFInterface()
    
    rclpy.spin(crsf_interface)
    
    crsf_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()