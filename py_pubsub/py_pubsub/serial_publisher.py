import rclpy
from rclpy.node import Node
from wheelchair_sensor_msgs.msg import UWB
import serial
import re
import sys

class SerialPublisher(Node):
    def __init__(self, serial_port='/dev/ttyUSB0'):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(UWB, 'uwb_data', 10)

        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            self.get_logger().info(f'Serial port opened: {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
            raise SystemExit

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.dist1 = 0.0
        self.dist2 = 0.0

    def timer_callback(self):
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            match1 = re.match(r'Resp1:\s*([\d.]+)', line)
            match2 = re.match(r'Resp2:\s*([\d.]+)', line)

            if match1:
                self.dist1 = float(match1.group(1))
                self.get_logger().info(f"Resp1: {self.dist1} m")

            if match2:
                self.dist2 = float(match2.group(1))
                self.get_logger().info(f"Resp2: {self.dist2} m")

        msg = UWB()
        msg.dist1 = self.dist1
        msg.dist2 = self.dist2
        msg.dist3 = 0.0
        msg.dist4 = 0.0
        msg.dist5 = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    node = SerialPublisher(serial_port)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
