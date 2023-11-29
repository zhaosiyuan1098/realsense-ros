import rclpy
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Header
from cv_bridge import CvBridge


class RGBDSubscriber(Node):
    def __init__(self):
        super().__init__('rgbd_subscriber')
        self.subscription = self.create_subscription(
            RGBD,
            '/camera/camera/rgbd',
            self.listener_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.get_logger().info('finish init')
        self.aruco_dict_4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_dict_5 = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.transform_matrix_calculator = TransformMatrixCalculator()
        
    def tranform_matrix_calculate(self,image):
        self.transform_matrix_calculator.calculate_transform_matrix(aruco_dict_4)
        self.transform_matrix_calculator.calculate_transform_matrix(aruco_dict_5)
    def listener_callback(self, msg):
        header = msg.header
        rgb_camera_info = msg.rgb_camera_info
        depth_camera_info = msg.depth_camera_info
        rgb_image = msg.rgb
        depth_image = msg.depth

        # Convert RGB image to OpenCV Mat
        rgb_cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_image, 'bgr8')
        self.get_logger().info('get realsense frame')

        # Process the RGB image as needed
        # ...
        cv2.imshow("object", rgb_cv_image)                             # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(1)
        self.trasform_matrix_calculate(rgb_cv_image)
        
class TransformMatrixCalculator():
    def __init__(self) -> None:
        """
        Initializes the TransformMatrixCalculator object.

        This constructor sets up the pipeline and configuration for the Intel RealSense camera,
        and initializes various parameters used for marker detection.

        Args:
            None

        Returns:
            None
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.parameters = aruco.DetectorParameters()
        self.transform_matrix_4 = None
        self.transform_matrix_5 = None
        
    def get_aligned_images(self):
        """
        Retrieves aligned color and depth images along with their corresponding intrinsics and depth scaling.

        Returns:
            color_image (numpy.ndarray): Aligned color image.
            depth_image (numpy.ndarray): Aligned depth image.
            depth_image_8bit (numpy.ndarray): Aligned depth image converted to 8-bit.
            intr_matrix (numpy.ndarray): Intrinsic matrix of the color camera.
            coeffs (numpy.ndarray): Distortion coefficients of the color camera.
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        intr_matrix = np.array([
            [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
        ])
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)
        pos = np.where(depth_image_8bit == 0)
        depth_image_8bit[pos] = 255
        color_image = np.asanyarray(color_frame.get_data())
        self.intr_matrix = intr_matrix
        return color_image, depth_image, depth_image_8bit, intr_matrix, np.array(intr.coeffs)

    def calculate_transform_matrix(self,aruco_dict):
        """
        Calculates the transformation matrix based on the detected ArUco markers.

        Args:
            aruco_dict (cv2.aruco.Dictionary): ArUco dictionary used for marker detection.

        Returns:
            transform_matrix (numpy.ndarray): Transformation matrix.
        """
        rgb, depth, depth_8bit, intr_matrix, intr_coeffs = self.get_aligned_images()
        corners, ids, rejected_img_points = aruco.detectMarkers(
            rgb, aruco_dict, parameters=self.parameters)
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
            corners, 0.045, intr_matrix, intr_coeffs)
        try:
            aruco.drawDetectedMarkers(rgb, corners)
            cv2.drawFrameAxes(rgb, intr_matrix, intr_coeffs, rvec, tvec, 0.05)

            if ids is not None and len(ids) > 0:
                # Get the transformation matrix of the first detected ArUco marker
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.045, intr_matrix, intr_coeffs)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rotation_matrix
                transform_matrix[:3, 3] = tvec[0]
                
                
                if aruco_dict.markerSize == 4:
                    self.transform_matrix_4 = transform_matrix
                    self.get_transformed_needle_points(aruco_dict=aruco_dict)
                    print("transform_matrix_4:",transform_matrix)
                    
                if aruco_dict.markerSize == 5:
                    self.transform_matrix_5 = transform_matrix
                    print("transform_matrix_5:",transform_matrix)
                    
                else:
                    print("aruco_dict.markerSize is not 4 or 5")
                    return
            else:
                print("no change")

        except:
            pass
        return transform_matrix

    def show_detected_marker(self,aruco_dict):
        """
        Displays the detected ArUco markers on the color image.

        Args:
            aruco_dict (cv2.aruco.Dictionary): ArUco dictionary used for marker detection.

        Returns:
            None
        """
        rgb, depth, depth_8bit, intr_matrix, intr_coeffs = self.get_aligned_images()
        corners, ids, rejected_img_points = aruco.detectMarkers(
            rgb, aruco_dict, parameters=self.parameters)
        aruco.drawDetectedMarkers(rgb, corners)
        cv2.imshow('rgb',rgb)
    
    def get_transformed_needle_points(self,aruco_dict):
        """
        Calculates the transformed needle points based on the transformation matrix.

        Args:
            aruco_dict (cv2.aruco.Dictionary): ArUco dictionary used for marker detection.

        Returns:
            None
        """
        if aruco_dict.markerSize == 4:
            transformation_matrix = self.transform_matrix_4
            
        if aruco_dict.markerSize == 5:
            transformation_matrix = self.transform_matrix_5
            
        else:
            # Handle the case when aruco_dict.markerSize is neither 4 nor 5
            return
            print("aruco_dict.markerSize is not 4 or 5")
            return
            
        
        # Define the start point and direction of the line segment
        start_point = np.array([0, 0, -0.017, 1])  # 1 cm along -z
        end_point = np.array([0, -0.3, -0.017, 1])  # 20 cm along -y

        # Transform the points to the camera frame
        start_point_cam = np.dot(transformation_matrix, start_point)
        end_point_cam = np.dot(transformation_matrix, end_point)

        # Project the 3D points onto the 2D image coordinates
        start_point_img = np.dot(self.intr_matrix, start_point_cam[:3])
        start_point_img = start_point_img / start_point_img[2]
        end_point_img = np.dot(self.intr_matrix, end_point_cam[:3])
        end_point_img = end_point_img / end_point_img[2]
        self.start_point_img = start_point_img
        self.end_point_img = end_point_img
        
        print("start_point_img:",start_point_img,"end_point_img:",end_point_img)

        return start_point_img, end_point_img

def main(args=None):
    rclpy.init(args=args)
    rgbd_subscriber = RGBDSubscriber()
    rclpy.spin(rgbd_subscriber)
    rgbd_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()