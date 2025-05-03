import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoVRMock(Node):
    def __init__(self):
        super().__init__('stereo_vr_mock')
        self.sub = self.create_subscription(Image, '/head_camera/image_raw', self.callback, 10)
        self.pub = self.create_publisher(Image, '/head_camera/vr_view', 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = img.shape[:2]

            # Extract center crop (512x512) or use original
            if w >= 512:
                center = img[:, w//2 - 256 : w//2 + 256]
            else:
                center = cv2.resize(img, (512, h))

            # Create left and right eye views
            left_eye = center.copy()
            right_eye = center.copy()

            # Combine without resizing
            vr_view = np.hstack((left_eye, right_eye))  # Final size: height x 1024

            # Publish the full-resolution VR image
            out_msg = self.bridge.cv2_to_imgmsg(vr_view, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"VR view generation failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoVRMock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
