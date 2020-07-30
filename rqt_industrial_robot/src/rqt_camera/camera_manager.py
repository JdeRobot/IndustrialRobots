#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
# import pcl


class CameraManager:
    def __init__(self):
        self.cam_robot_rgb = rospy.Subscriber("kinect_camera_robot/rgb/image_raw", Image, self.cam_robot_rgb_callback)
        self.cam_fixed_rgb = rospy.Subscriber("kinect_camera_fixed/rgb/image_raw", Image, self.cam_fixed_rgb_callback)
        self.cam_robot_depth = rospy.Subscriber("kinect_camera_robot/depth/points", PointCloud2, self.cam_robot_depth_callback)
        self.cam_fixed_depth = rospy.Subscriber("kinect_camera_fixed/depth/points", PointCloud2, self.cam_fixed_depth_callback)

        self.cam_robot_filtered_pub = rospy.Publisher("kinect_camera_robot/rgb/filtered/image_raw", Image, queue_size=0)
        self.cam_fixed_filtered_pub = rospy.Publisher("kinect_camera_fixed/rgb/filtered/image_raw", Image, queue_size=0)
        self.bridge = CvBridge()

    def cam_robot_rgb_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        img_msg = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")
        self.cam_robot_filtered_pub.publish(img_msg)

    def cam_fixed_rgb_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        img_msg = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")
        self.cam_fixed_filtered_pub.publish(img_msg)

    def cam_robot_depth_callback(self, msg):
        pass

    def cam_fixed_depth_callback(self, msg):
        pass


if __name__ == "__main__":
    rospy.init_node("camera_manager")
    cam_manager = CameraManager()

    rospy.spin()