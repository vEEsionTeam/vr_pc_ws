import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import cv2
import numpy as np

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, PointCloud, PointCloud2
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class TFServer(Node):
    def __init__(self):
        super().__init__('tf_server')

        # Declare parameters
        self.declare_parameter('server_ip', '192.168.x.x')
        self.declare_parameter('server_port', 5005)
        self.declare_parameter('img_enable', 1)
        self.declare_parameter('path_enable', 1)
        self.declare_parameter('points_enable', 1)

        # Get parameter values
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.img_enable = self.get_parameter('img_enable').get_parameter_value().integer_value
        self.path_enable = self.get_parameter('path_enable').get_parameter_value().integer_value
        self.points_enable = self.get_parameter('points_enable').get_parameter_value().integer_value

        # === TF SOCKET (5005) ===
        self.sock_tf = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_tf.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_tf.bind((self.server_ip, self.server_port))
        self.sock_tf.listen(1)
        self.get_logger().info(f'TF server listening on {self.server_ip}:{self.server_port}')
        self.client_tf, _ = self.sock_tf.accept()
        self.client_tf.settimeout(0.001)
        self.get_logger().info('TF client connected.')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.buffer_tf = ""
        self.timer = self.create_timer(0.02, self.receive_tf_data)

        # === IMAGE SOCKET (5007) ===
        if self.img_enable:
            self.sock_img = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_img.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_img.bind((self.server_ip, 5007))
            self.sock_img.listen(1)
            self.get_logger().info(f'Image server listening on {self.server_ip}:5007')
            self.client_img, _ = self.sock_img.accept()
            self.get_logger().info('Image client connected.')

            self.bridge = CvBridge()
            self.image_pub = self.create_publisher(Image, '/ov_msckf/trackhist', 10)
            threading.Thread(target=self.receive_images, daemon=True).start()

        # === PATH SOCKET (5008) ===
        if self.path_enable:
            self.sock_path = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_path.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_path.bind((self.server_ip, 5008))
            self.sock_path.listen(1)
            self.get_logger().info(f'Path server listening on {self.server_ip}:5008')
            self.client_path, _ = self.sock_path.accept()
            self.get_logger().info('Path client connected.')

            self.path_pub = self.create_publisher(Path, '/ov_msckf/pathimu', 10)
            threading.Thread(target=self.receive_path, daemon=True).start()

        # === POINTCLOUD SOCKETS (5009, 5010) ===
        if self.points_enable:
            self.sock_pc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_pc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_pc.bind((self.server_ip, 5009))
            self.sock_pc.listen(1)
            self.get_logger().info(f'PointCloud server listening on {self.server_ip}:5009')
            self.client_pc, _ = self.sock_pc.accept()
            self.get_logger().info('PointCloud client connected.')

            self.sock_pc2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_pc2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_pc2.bind((self.server_ip, 5010))
            self.sock_pc2.listen(1)
            self.get_logger().info(f'PointCloud2 server listening on {self.server_ip}:5010')
            self.client_pc2, _ = self.sock_pc2.accept()
            self.get_logger().info('PointCloud2 client connected.')

            self.pc_pub = self.create_publisher(PointCloud, '/ov_msckf/loop_feats', 10)
            self.pc2_pub = self.create_publisher(PointCloud2, '/ov_msckf/points_slam', 10)
            threading.Thread(target=self.receive_pointcloud, daemon=True).start()
            threading.Thread(target=self.receive_pointcloud2, daemon=True).start()

    def receive_tf_data(self):
        try:
            while True:
                chunk = self.client_tf.recv(1024).decode('utf-8')
                if not chunk:
                    break
                self.buffer_tf += chunk
                while '\n' in self.buffer_tf:
                    line, self.buffer_tf = self.buffer_tf.split('\n', 1)
                    self.publish_tf_from_data(line.strip())
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f'Error receiving TF data: {e}')

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

    def receive_images(self):
        try:
            while True:
                length_prefix = self.client_img.recv(4)
                if not length_prefix:
                    break
                img_len = struct.unpack('>I', length_prefix)[0]
                image_data = b''
                while len(image_data) < img_len:
                    chunk = self.client_img.recv(img_len - len(image_data))
                    if not chunk:
                        break
                    image_data += chunk
                np_arr = np.frombuffer(image_data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    cv_image = cv2.flip(cv_image, 0)  # Flip vertically (upside down)
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = 'camera_frame'
                    self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().warn(f'Image receive error: {e}')

    def receive_path(self):
        try:
            while True:
                length = self.client_path.recv(4)
                if not length:
                    break
                msg_len = struct.unpack('>I', length)[0]
                msg_data = b''
                while len(msg_data) < msg_len:
                    chunk = self.client_path.recv(msg_len - len(msg_data))
                    if not chunk:
                        break
                    msg_data += chunk
                msg = deserialize_message(msg_data, get_message('nav_msgs/msg/Path'))
                self.path_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Path receive error: {e}')

    def receive_pointcloud(self):
        try:
            while True:
                length = self.client_pc.recv(4)
                if not length:
                    break
                msg_len = struct.unpack('>I', length)[0]
                msg_data = b''
                while len(msg_data) < msg_len:
                    chunk = self.client_pc.recv(msg_len - len(msg_data))
                    if not chunk:
                        break
                    msg_data += chunk
                msg = deserialize_message(msg_data, get_message('sensor_msgs/msg/PointCloud'))
                self.pc_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'PointCloud receive error: {e}')

    def receive_pointcloud2(self):
        try:
            while True:
                length = self.client_pc2.recv(4)
                if not length:
                    break
                msg_len = struct.unpack('>I', length)[0]
                msg_data = b''
                while len(msg_data) < msg_len:
                    chunk = self.client_pc2.recv(msg_len - len(msg_data))
                    if not chunk:
                        break
                    msg_data += chunk
                msg = deserialize_message(msg_data, get_message('sensor_msgs/msg/PointCloud2'))
                self.pc2_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'PointCloud2 receive error: {e}')


def main():
    rclpy.init()
    node = TFServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.client_tf.close()
    node.sock_tf.close()
    if hasattr(node, 'client_img'):
        node.client_img.close()
        node.sock_img.close()
    if hasattr(node, 'client_path'):
        node.client_path.close()
        node.sock_path.close()
    if hasattr(node, 'client_pc'):
        node.client_pc.close()
        node.sock_pc.close()
    if hasattr(node, 'client_pc2'):
        node.client_pc2.close()
        node.sock_pc2.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

