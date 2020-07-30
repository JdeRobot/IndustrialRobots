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

		self.choose_camera_on_robot = True
		self.choose_camera_fixed = False
		self.camera_on_robot = "kinect_camera_robot"
		self.camera_fixed = "kinect_camera_fixed"
		self._widget.camera_comboBox.addItems(["kinect_camera_robot", "kinect_camera_fixed"])
		self._widget.camera_comboBox.currentIndexChanged.connect(self.choose_camera)

		# Add Subscibers
		rospy.Subscriber("kinect_camera_robot/rgb/image_raw", Image, self.cam_on_robot_viewer)
		rospy.Subscriber("kinect_camera_fixed/rgb/image_raw", Image, self.cam_fixed_viewer)
		rospy.Subscriber("kinect_camera_robot/rgb/filtered/image_raw", Image, self.cam_on_robot_filtered_viewer)
		rospy.Subscriber("kinect_camera_fixed/rgb/filtered/image_raw", Image, self.cam_fixed_filtered_viewer)

	def msg_to_pixmap(self, msg):
		# print("before",msg.msg.encoding)
		#msg.encoding = "rgb8"
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

	def cam_on_robot_viewer(self, msg):
		if self.choose_camera_on_robot:
			self._widget.image_raw.setPixmap(self.msg_to_pixmap(msg))

	def cam_fixed_viewer(self, msg):
		if self.choose_camera_fixed:
			self._widget.image_raw.setPixmap(self.msg_to_pixmap(msg))

	def cam_on_robot_filtered_viewer(self, msg):
		if self.choose_camera_on_robot:
			self._widget.image_filtered.setPixmap(self.msg_to_pixmap(msg))

	def cam_fixed_filtered_viewer(self, msg):
		if self.choose_camera_fixed:
			self._widget.image_filtered.setPixmap(self.msg_to_pixmap(msg))

	def choose_camera(self):
		camera = self._widget.camera_comboBox.currentText()
		if camera == self.camera_on_robot:
			self.choose_camera_on_robot = True
			self.choose_camera_fixed = False
		else:
			self.choose_camera_on_robot = False
			self.choose_camera_fixed = True