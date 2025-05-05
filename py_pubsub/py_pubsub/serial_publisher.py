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

        # Timer to call callback at 20Hz (every 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initialize distances
        self.dist1 = 0.0
        self.dist2 = 0.0
        self.dist3 = 0.0

    def timer_callback(self):
        # Read all available lines from serial
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            # Match only floats like 3.25 (two decimals only)
            match1 = re.match(r'Resp1:\s*(\d+\.\d{2})', line)
            match2 = re.match(r'Resp2:\s*(\d+\.\d{2})', line)
            match3 = re.match(r'Resp3:\s*(\d+\.\d{2})', line)

            if match1:
                self.dist1 = float(match1.group(1))
                self.get_logger().info(f"Resp1: {self.dist1:.2f} m")

            elif match2:
                self.dist2 = float(match2.group(1))
                self.get_logger().info(f"Resp2: {self.dist2:.2f} m")

            elif match3:
                self.dist3 = float(match3.group(1))
                self.get_logger().info(f"Resp3: {self.dist3:.2f} m")

            else:
                self.get_logger().warn(f"Respunknown: {line}")

        # Create and publish UWB message
        msg = UWB()
        msg.dist1 = self.dist1
        msg.dist2 = self.dist2
        msg.dist3 = self.dist3
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

