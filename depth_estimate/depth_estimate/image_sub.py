import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs import RGBD
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

class RGBDSubscriber(Node):
    def __init__(self):
        super().__init__('rgbd_subscriber')
        self.subscription = self.create_subscription(
            RGBD,
            'rgbd_topic',
            self.listener_callback,
            10
        )
        self.cv_bridge = CvBridge()

    def listener_callback(self, msg):
        header = msg.header
        rgb_camera_info = msg.rgb_camera_info
        depth_camera_info = msg.depth_camera_info
        rgb_image = msg.rgb
        depth_image = msg.depth

        # Convert RGB image to OpenCV Mat
        rgb_cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_image, 'bgr8')

        # Process the RGB image as needed
        # ...

def main(args=None):
    rclpy.init(args=args)
    rgbd_subscriber = RGBDSubscriber()
    rclpy.spin(rgbd_subscriber)
    rgbd_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()