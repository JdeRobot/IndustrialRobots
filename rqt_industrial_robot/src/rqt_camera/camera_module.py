#!/usr/bin/env python

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon, QPixmap, QImage

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraViewer(Plugin):
	def __init__(self, context):
		super(CameraViewer, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('CameraViewer')

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
						dest="quiet",
						help="Put plugin in silent mode")
		# args, unknowns = parser.parse_known_args(context.argv())
		# if not args.quiet:
		# 	print 'arguments: ', args
		# 	print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which should be in the "resource" folder of this package
		ui_file = os.path.join(rospkg.RosPack().get_path(
			'rqt_industrial_robot'), 'resources', 'CameraViewer.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('CameraViewerUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(
				self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)
		self.bridge = CvBridge()

		image_topics = ["kinect_camera_robot/rgb/image_raw",
						"kinect_camera_fixed/rgb/image_raw",
						"red_filtered_image",
						"green_filtered_image",
						"blue_filtered_image",
						"yellow_filtered_image",
						"red_sphere_image",
						"green_sphere_image",
						"blue_sphere_image",
						"yellow_sphere_image",
						"red_cylinder_image",
						"green_cylinder_image",
						"blue_cylinder_image",
						"yellow_cylinder_image"]
		self._widget.camera_comboBox.addItems(image_topics)
		self._widget.camera_comboBox.currentIndexChanged.connect(self.choose_camera1)
		self._widget.camera_comboBox_2.addItems(image_topics)
		self._widget.camera_comboBox_2.currentIndexChanged.connect(self.choose_camera2)

		self.image_topics_length = len(image_topics)
		self.camera1_on = [False for _ in range(self.image_topics_length)]
		self.camera1_on[0] = True
		self.camera2_on = [False for _ in range(self.image_topics_length)]
		self.camera2_on[1] = True

		# Add Subscibers
		rospy.Subscriber("kinect_camera_robot/rgb/image_raw", Image, self.cam_viewer_robot)
		rospy.Subscriber("kinect_camera_fixed/rgb/image_raw", Image, self.cam_viewer_fixed)
		rospy.Subscriber("red_filtered_image", Image, self.cam_viewer_rf)
		rospy.Subscriber("green_filtered_image", Image, self.cam_viewer_gf)
		rospy.Subscriber("blue_filtered_image", Image, self.cam_viewer_bf)
		rospy.Subscriber("yellow_filtered_image", Image, self.cam_viewer_yf)
		rospy.Subscriber("green_sphere_image", Image, self.cam_viewer_gs)
		rospy.Subscriber("red_sphere_image", Image, self.cam_viewer_rs)
		rospy.Subscriber("yellow_sphere_image", Image, self.cam_viewer_ys)
		rospy.Subscriber("blue_sphere_image", Image, self.cam_viewer_bs)
		rospy.Subscriber("green_cylinder_image", Image, self.cam_viewer_gc)
		rospy.Subscriber("red_cylinder_image", Image, self.cam_viewer_rc)
		rospy.Subscriber("yellow_cylinder_image", Image, self.cam_viewer_yc)
		rospy.Subscriber("blue_cylinder_image", Image, self.cam_viewer_bc)

	def rgb_msg_to_pixmap(self, msg):
		# print("before",msg.msg.encoding)
		msg.encoding = "bgr8"
		# print("after",msg.encoding)
		cv_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
		shape = cv_img.shape
		if len(shape) == 3 and shape[2] == 3:  # RGB888
			h, w, _ = shape
			bytesPerLine = 3 * w
			img_format = QImage.Format_RGB888
		else:  # Grayscale8
			h, w = shape[0], shape[1]
			bytesPerLine = 1 * w
			img_format = QImage.Format_Grayscale8
		q_img = QImage(cv_img.data, w, h, bytesPerLine, img_format)
		return QPixmap.fromImage(q_img).scaled(320, 240)

	def depth_msg_to_pixmap(self, msg):
		msg.encoding = "mono8"
		cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
		shape = cv_img.shape
		if len(shape) == 3 and shape[2] == 3:  # RGB888
			h, w, _ = shape
			bytesPerLine = 3 * w
			img_format = QImage.Format_RGB888
		else:  # Grayscale8
			h, w = shape[0], shape[1]
			bytesPerLine = 1 * w
			img_format = QImage.Format_Grayscale8
		q_img = QImage(cv_img.data, w, h, bytesPerLine, img_format)
		return QPixmap.fromImage(q_img).scaled(320, 240)

	def cam_viewer_robot(self, msg):
		if self.camera1_on[0]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[0]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_fixed(self, msg):
		if self.camera1_on[1]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[1]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_rf(self, msg):
		if self.camera1_on[2]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[2]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_gf(self, msg):
		if self.camera1_on[3]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[3]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_bf(self, msg):
		if self.camera1_on[4]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[4]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_yf(self, msg):
		if self.camera1_on[5]:
			self._widget.image1.setPixmap(self.rgb_msg_to_pixmap(msg))
		if self.camera2_on[5]:
			self._widget.image2.setPixmap(self.rgb_msg_to_pixmap(msg))

	def cam_viewer_gs(self, msg):
		if self.camera1_on[7]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[7]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_rs(self, msg):
		if self.camera1_on[6]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[6]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_ys(self, msg):
		if self.camera1_on[9]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[9]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_bs(self, msg):
		if self.camera1_on[8]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[8]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_gc(self, msg):
		if self.camera1_on[11]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[11]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_rc(self, msg):
		if self.camera1_on[10]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[10]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_yc(self, msg):
		if self.camera1_on[13]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[13]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def cam_viewer_bc(self, msg):
		if self.camera1_on[12]:
			self._widget.image1.setPixmap(self.depth_msg_to_pixmap(msg))
		if self.camera2_on[12]:
			self._widget.image2.setPixmap(self.depth_msg_to_pixmap(msg))

	def choose_camera1(self):
		id = self._widget.camera_comboBox.currentIndex()
		self.camera1_on = [False for _ in range(self.image_topics_length)]
		self.camera1_on[id] = True

	def choose_camera2(self):
		id = self._widget.camera_comboBox_2.currentIndex()
		self.camera2_on = [False for _ in range(self.image_topics_length)]
		self.camera2_on[id] = True