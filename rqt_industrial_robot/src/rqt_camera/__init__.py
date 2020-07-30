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
			'rqt_drone_teleop'), 'resource', 'CamViewer.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('CamViewerUi')
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

		# Add Subscibers
		cam_frontal_topic = rospy.get_param('cam_frontal_topic', '/iris/cam_frontal/image_raw')
		cam_ventral_topic = rospy.get_param('cam_ventral_topic', '/iris/cam_ventral/image_raw')
		rospy.Subscriber(cam_frontal_topic, Image, self.cam_frontal_cb)
		rospy.Subscriber(cam_ventral_topic, Image, self.cam_ventral_cb)
		rospy.Subscriber('interface/filtered_img', Image, self.filtered_img_cb)
		rospy.Subscriber('interface/threshed_img', Image, self.threshed_img_cb)

	def msg_to_pixmap(self, msg):
		cv_img = self.bridge.imgmsg_to_cv2(msg)
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

	def cam_frontal_cb(self, msg):
		self._widget.img_frontal.setPixmap(self.msg_to_pixmap(msg))

	def cam_ventral_cb(self, msg):
		self._widget.img_ventral.setPixmap(self.msg_to_pixmap(msg))

	def threshed_img_cb(self, msg):
		self._widget.img_threshed.setPixmap(self.msg_to_pixmap(msg))

	def filtered_img_cb(self, msg):
		self._widget.img_filtered.setPixmap(self.msg_to_pixmap(msg))

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	# def trigger_configuration(self):
	# Comment in to signal that the plugin has a way to configure
	# This will enable a setting button (gear icon) in each dock widget title bar
	# Usually used to open a modal configuration dialog