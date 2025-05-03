import rclpy
from rclpy.node import Node
import socket
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TFServer(Node):
    def __init__(self):
        super().__init__('tf_server')
        self.declare_parameter('server_ip', '192.168.1.46')  # Change to your PC's IP
        self.declare_parameter('server_port', 5005)

        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.server_ip, self.server_port))
        self.sock.listen(1)
        self.get_logger().info(f'Server listening on {self.server_ip}:{self.server_port}')

        self.client_socket, self.client_address = self.sock.accept()
        self.client_socket.settimeout(0.001)  # short timeout for non-blocking read
        self.get_logger().info(f'Connected to client: {self.client_address}')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.buffer = ""

        self.timer = self.create_timer(0.005, self.receive_tf_data)  # 200 Hz

    def receive_tf_data(self):
        try:
            while True:
                chunk = self.client_socket.recv(1024).decode('utf-8')
                if not chunk:
                    break
                self.buffer += chunk
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    self.publish_tf_from_data(line.strip())
        except socket.timeout:
            pass  # No data, continue timer
        except Exception as e:
            self.get_logger().warn(f'Error receiving data: {e}')

    def publish_tf_from_data(self, data):
        try:
            values = list(map(float, data.split(',')))
            if len(values) != 7:
                return
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "global"
            tf_msg.child_frame_id = "imu"
            tf_msg.transform.translation.x = values[0]
            tf_msg.transform.translation.y = values[1]
            tf_msg.transform.translation.z = values[2]
            tf_msg.transform.rotation.x = values[3]
            tf_msg.transform.rotation.y = values[4]
            tf_msg.transform.rotation.z = values[5]
            tf_msg.transform.rotation.w = values[6]
            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception as e:
            self.get_logger().warn(f'Error parsing TF: {e}')

def main():
    rclpy.init()
    node = TFServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.client_socket.close()
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

