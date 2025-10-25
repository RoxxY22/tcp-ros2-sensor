import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt16
from std_srvs.srv import SetBool, Empty
import socket
import threading
import struct

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('interval_ms', 1000)
        
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.interval = self.get_parameter('interval_ms').value
        
        # TCP connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'Connected to sensor at {host}:{port}')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            rclpy.shutdown()
        
        # Publishers (topics)
        self.pub_voltage = self.create_publisher(UInt16, '/sensor/supply_voltage', 10)
        self.pub_temp = self.create_publisher(Float32, '/sensor/env_temp', 10)
        self.pub_yaw = self.create_publisher(Float32, '/sensor/yaw', 10)
        self.pub_pitch = self.create_publisher(Float32, '/sensor/pitch', 10)
        self.pub_roll = self.create_publisher(Float32, '/sensor/roll', 10)
        
        # Services for start/stop
        self.srv_start = self.create_service(SetBool, '/start_sensor', self.start_callback)
        self.srv_stop = self.create_service(Empty, '/stop_sensor', self.stop_callback)
        
        # Receiver thread
        self.running = True
        self.thread = threading.Thread(target=self.receive_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Auto-send start on launch
        self.send_start(self.interval)

    def send_start(self, interval):
        # Convert interval to little-endian hex (4 chars)
        low = interval & 0xFF
        high = (interval >> 8) & 0xFF
        payload = f"{low:02X}{high:02X}"
        msg = f"#03{payload}\r\n"
        self.sock.send(msg.encode())
        self.get_logger().info(f'Sent start with interval {interval}ms')

    def send_stop(self):
        msg = "#09\r\n"
        self.sock.send(msg.encode())
        self.get_logger().info('Sent stop')

    def start_callback(self, request, response):
        if request.data:  # True to start
            self.send_start(self.interval)  # Uses current param value
            response.success = True
            response.message = 'Started'
        else:
            response.success = False
            response.message = 'Ignored (data=false)'
        return response

    def stop_callback(self, request, response):
        self.send_stop()
        return response

    def receive_loop(self):
        buffer = ''
        while self.running:
            try:
                data = self.sock.recv(1024).decode()
                buffer += data
                while '\r\n' in buffer:
                    msg, buffer = buffer.split('\r\n', 1)
                    if msg.startswith('$11') and len(msg) == 23:  # $11 + 20 hex
                        payload = msg[3:]
                        # Decode little-endian
                        voltage = struct.unpack('<H', bytes.fromhex(payload[0:4]))[0]
                        temp = struct.unpack('<h', bytes.fromhex(payload[4:8]))[0] / 10.0
                        yaw = struct.unpack('<h', bytes.fromhex(payload[8:12]))[0] / 10.0
                        pitch = struct.unpack('<h', bytes.fromhex(payload[12:16]))[0] / 10.0
                        roll = struct.unpack('<h', bytes.fromhex(payload[16:20]))[0] / 10.0
                        
                        # Publish
                        self.pub_voltage.publish(UInt16(data=voltage))
                        self.pub_temp.publish(Float32(data=temp))
                        self.pub_yaw.publish(Float32(data=yaw))
                        self.pub_pitch.publish(Float32(data=pitch))
                        self.pub_roll.publish(Float32(data=roll))
                        
                        self.get_logger().info(f'Published: V={voltage}, T={temp}, Y={yaw}, P={pitch}, R={roll}')
            except Exception as e:
                self.get_logger().error(f'Receive error: {e}')
                break

    def destroy_node(self):
        self.running = False
        self.send_stop()
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()