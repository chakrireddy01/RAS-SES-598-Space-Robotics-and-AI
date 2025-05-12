#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from transforms3d.euler import mat2euler
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArucoTracker(Node):

    def __init__(self):
        super().__init__('aruco_tracker')

        self.cv_bridge = CvBridge()
        self.frame_counter = 0
        self.marker_positions = {}  # <- Store marker ID -> [x, y, z]

        self.get_logger().info(f'OpenCV version: {cv2.__version__}')
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using OpenCV 4.7+ ArUco API')
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.get_logger().info('Using older OpenCV ArUco API')

        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)
        self.calibration_received = False

        self.marker_size = 0.8

        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Image, '/mono_camera', self.image_callback, 10)
        self.create_subscription(
            CameraInfo, '/mono_camera/camera_info',
            self.camera_info_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
        )
        self.get_logger().info('ArUco tracker node initialized')

    def detect_markers(self, image):
        try:
            if self.detector:
                corners, ids, _ = self.detector.detectMarkers(image)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    image, self.aruco_dict, parameters=self.aruco_params)
            return corners, ids
        except Exception as e:
            self.get_logger().error(f'Marker detection error: {str(e)}')
            return [], None

    def draw_crosshair(self, image, center, size=20, color=(0, 0, 255), thickness=2):
        x, y = int(center[0]), int(center[1])
        cv2.line(image, (x - size, y), (x + size, y), color, thickness)
        cv2.line(image, (x, y - size), (x, y + size), color, thickness)
        cv2.circle(image, (x, y), 2, color, thickness)

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default camera calibration')

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            corners, ids = self.detect_markers(cv_image)
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self.marker_size, self.camera_matrix, self.dist_coeffs)
                    cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs,
                                      rvec[0], tvec[0], self.marker_size / 2)

                    marker_position = tvec[0][0]
                    rot_matrix, _ = cv2.Rodrigues(rvec[0])
                    euler_angles = mat2euler(rot_matrix)
                    quat = self.euler_to_quaternion(*euler_angles)

                    marker_id = ids[i][0]
                    self.marker_positions[marker_id] = marker_position.tolist()

                    transform = TransformStamped()
                    transform.header.stamp = self.get_clock().now().to_msg()
                    transform.header.frame_id = 'camera_frame'
                    transform.child_frame_id = f'aruco_marker_{marker_id}'
                    transform.transform.translation.x = marker_position[0]
                    transform.transform.translation.y = marker_position[1]
                    transform.transform.translation.z = marker_position[2]
                    transform.transform.rotation.x = quat[0]
                    transform.transform.rotation.y = quat[1]
                    transform.transform.rotation.z = quat[2]
                    transform.transform.rotation.w = quat[3]
                    self.tf_broadcaster.sendTransform(transform)

                    pose_msg = String()
                    pose_msg.data = f"Marker {marker_id} at x:{marker_position[0]:.2f}, y:{marker_position[1]:.2f}, z:{marker_position[2]:.2f}"
                    self.marker_pose_pub.publish(pose_msg)

                    center = corners[i][0].mean(axis=0)
                    self.draw_crosshair(debug_image, center)
                    cv2.putText(debug_image,
                                pose_msg.data,
                                (int(center[0]), int(center[1] - 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.get_logger().info(f"Tracked Marker Positions: {self.marker_positions}")

                # ➕ Tallest cylinder logic
                if len(self.marker_positions) >= 2:
                    tallest_id = max(self.marker_positions, key=lambda k: self.marker_positions[k][2])
                    tallest_pos = self.marker_positions[tallest_id]
                    self.get_logger().info(f"Tallest marker ID: {tallest_id} at position {tallest_pos}")

            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info('Camera calibration received')
        except Exception as e:
            self.get_logger().error(f'Camera calibration error: {str(e)}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]

def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

