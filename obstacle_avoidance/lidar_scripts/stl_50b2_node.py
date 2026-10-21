import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time
import struct
import threading

def generate_crc8_table(poly=0x4D):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
        table.append(crc & 0xFF)
    return table

CRC8_TABLE = generate_crc8_table()

def calc_crc8(data):
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc

class STL50B2Driver(Node):
    def __init__(self):
        super().__init__('stl_50b2_driver')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('frame_id', 'laser_link')
        
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        self.serial_conn = None
        self.reading_thread = None
        self.is_running = True
        
        # State for aggregating points into a full 360 scan
        self.scan_points = []
        self.last_angle = 0.0
        
        self.connect_serial()
        
    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f"Connected to Lidar on {self.port} at {self.baud} baud.")
            
            # Send Start Command
            start_cmd = bytes([0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E])
            self.serial_conn.write(start_cmd)
            self.get_logger().info("Sent Start Command to Lidar.")
            
            self.reading_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.reading_thread.start()
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Lidar: {e}")
            
    def read_loop(self):
        # We need to find the header 0x54 0x2C
        buffer = bytearray()
        
        while self.is_running and rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(data)
                    
                    while len(buffer) >= 47:
                        # Find header
                        header_idx = buffer.find(b'\x54\x2C')
                        
                        if header_idx == -1:
                            # Keep the last byte in case it's 0x54
                            buffer = buffer[-1:]
                            break
                        
                        if header_idx > 0:
                            # Drop garbage before header
                            buffer = buffer[header_idx:]
                            
                        if len(buffer) < 47:
                            break
                            
                        # Extract the packet
                        packet = buffer[:47]
                        buffer = buffer[47:]
                        
                        # Verify CRC
                        if calc_crc8(packet[:-1]) == packet[-1]:
                            self.parse_packet(packet)
                        else:
                            self.get_logger().warn("CRC Mismatch! Dropping packet.", throttle_duration_sec=2.0)
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")
                time.sleep(1.0)
                
    def parse_packet(self, packet):
        # packet[0]: 0x54
        # packet[1]: 0x2C
        speed = struct.unpack('<H', packet[2:4])[0] # degrees/sec
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0
        timestamp = struct.unpack('<H', packet[44:46])[0] # ms
        
        diff = end_angle - start_angle
        if diff < 0:
            diff += 360.0
            
        step = diff / 11.0 if diff > 0 else 0.0
        
        for i in range(12):
            point_idx = 6 + i * 3
            # Assuming standard 2 bytes distance, 1 byte intensity
            distance = struct.unpack('<H', packet[point_idx:point_idx+2])[0] / 1000.0 # Convert mm to m
            intensity = packet[point_idx+2]
            
            angle_deg = start_angle + step * i
            if angle_deg >= 360.0:
                angle_deg -= 360.0
            
            angle_rad = math.radians(angle_deg)
            
            # Check for wrap around
            if angle_deg < self.last_angle and abs(angle_deg - self.last_angle) > 180.0:
                self.publish_scan()
                self.scan_points.clear()
                
            if distance > 0:
                self.scan_points.append((angle_rad, distance, intensity))
                
            self.last_angle = angle_deg

    def publish_scan(self):
        if not self.scan_points:
            return
            
        # Sort points by angle
        self.scan_points.sort(key=lambda x: x[0])
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angle_min = self.scan_points[0][0]
        msg.angle_max = self.scan_points[-1][0]
        
        # Calculate average angle increment
        if len(self.scan_points) > 1:
            msg.angle_increment = (msg.angle_max - msg.angle_min) / (len(self.scan_points) - 1)
        else:
            msg.angle_increment = 0.0
            
        msg.time_increment = 0.0
        msg.scan_time = 0.15 # Approx 6.7Hz
        
        msg.range_min = 0.05
        msg.range_max = 12.0
        
        msg.ranges = [p[1] for p in self.scan_points]
        msg.intensities = [float(p[2]) for p in self.scan_points]
        
        self.scan_pub.publish(msg)

    def destroy_node(self):
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            # Send Stop Command
            stop_cmd = bytes([0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A])
            self.serial_conn.write(stop_cmd)
            self.serial_conn.close()
            self.get_logger().info("Sent Stop Command and closed serial port.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STL50B2Driver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
