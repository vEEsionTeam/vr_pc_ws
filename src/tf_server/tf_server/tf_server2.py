import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import cv2
import numpy as np

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TFServer(Node):
    def __init__(self):
        super().__init__('tf_server')

        # Declare parameters
        self.declare_parameter('server_ip', '192.168.x.x')  # Replace with your PC's IP
        self.declare_parameter('server_port', 5005)
        self.declare_parameter('img_enable', 1)

        # Get parameter values
        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.img_enable = self.get_parameter('img_enable').get_parameter_value().integer_value

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

        # === IMAGE SOCKET (5007) IF ENABLED ===
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
            self.img_thread = threading.Thread(target=self.receive_images, daemon=True)
            self.img_thread.start()

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

                # Decode JPEG to OpenCV image
                np_arr = np.frombuffer(image_data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if cv_image is not None:
                    ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = 'camera_frame'
                    self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().warn(f'Image receive error: {e}')


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
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

