import rclpy
from rclpy.node import Node
from py_pubsub.msg import UWB
import serial
import re
import csv
import os

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(UWB, 'uwb_data', 10)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Serial port opened: /dev/ttyUSB0 at 115200')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise SystemExit

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.dist1 = 0.0
        self.dist2 = 0.0
        self.log_file = os.path.expanduser('~/uwb_log.csv')
        self.write_csv_header()

    def write_csv_header(self):
        if not os.path.isfile(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Dist1 (m)", "Dist2 (m)"])

    def log_to_csv(self, dist1, dist2):
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([dist1, dist2])

    def timer_callback(self):
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            match1 = re.match(r'Resp1:\s*([\d.]+)', line)
            match2 = re.match(r'Resp2:\s*([\d.]+)', line)

            if match1:
                self.dist1 = float(match1.group(1))
                self.get_logger().info(f"Parsed Resp1: {self.dist1} m")

            if match2:
                self.dist2 = float(match2.group(1))
                self.get_logger().info(f"Parsed Resp2: {self.dist2} m")

        msg = UWB()
        msg.dist1 = self.dist1
        msg.dist2 = self.dist2
        msg.dist3 = 0.0
        msg.dist4 = 0.0
        msg.dist5 = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published UWB: [{msg.dist1}, {msg.dist2}]")
        self.log_to_csv(self.dist1, self.dist2)

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
