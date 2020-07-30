import os
import rospy
import rospkg
import numpy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtGui
from python_qt_binding.QtCore import pyqtSignal
from interfaces.robot_wrapper import RobotWrapper
import threading
import time
import resources_rc
from std_msgs.msg import String, Bool

import yaml

DEG = u"\u00b0"


class Kinematics(Plugin):
    def __init__(self, context):
        super(Kinematics, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Kinematics')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        rospy.loginfo("Opening GUI")

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'resources', 'KinematicsPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.robot = RobotWrapper()

        filename = os.path.join(rospkg.RosPack().get_path('rqt_industrial_robot'), 'src','rqt_kinematics', 'joints_setup.yaml')
        with open(filename) as file:
            joints_setup = yaml.load(file)
            jointslimit = joints_setup["joints_limit"]

            home_value = joints_setup["home_value"]
            j1 = home_value["joint_1"]
            j2 = home_value["joint_2"]
            j3 = home_value["joint_3"]
            j4 = home_value["joint_4"]
            j5 = home_value["joint_5"]
            j6 = home_value["joint_6"]
            g = home_value["gripper"]
            self.robot.set_home_value([j1, j2, j3, j4, j5, j6, g])

        self._widget.xEdit.editingFinished.connect(self.set_x)
        self._widget.yEdit.editingFinished.connect(self.set_y)
        self._widget.zEdit.editingFinished.connect(self.set_z)
        self._widget.rollEdit.editingFinished.connect(self.set_roll)
        self._widget.pitchEdit.editingFinished.connect(self.set_pitch)
        self._widget.yawEdit.editingFinished.connect(self.set_yaw)

        self._widget.planButton.clicked.connect(self.plan)
        self._widget.executeButton.clicked.connect(self.execute)
        self._widget.planexeButton.clicked.connect(self.planexe)
        self._widget.stopexeButton.clicked.connect(self.stopexe)
        self._widget.homeButton.clicked.connect(self.backtohome)

        self._widget.jointSlider_1.sliderReleased.connect(self.setjoint1)
        self._widget.jointSlider_1.valueChanged.connect(self.viewjoint1)
        self._widget.jointSlider_1.setMinimum(jointslimit["joint_1"]["low"])
        self._widget.jointSlider_1.setMaximum(jointslimit["joint_1"]["high"])
        self._widget.joint1Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(1)),2))+DEG)
        self._widget.joint1_low.setText(str(jointslimit["joint_1"]["low"])+DEG)
        self._widget.joint1_high.setText(str(jointslimit["joint_1"]["high"])+DEG)

        self._widget.jointSlider_2.sliderReleased.connect(self.setjoint2)
        self._widget.jointSlider_2.valueChanged.connect(self.viewjoint2)
        self._widget.jointSlider_2.setMinimum(jointslimit["joint_2"]["low"])
        self._widget.jointSlider_2.setMaximum(jointslimit["joint_2"]["high"])
        self._widget.joint2Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(2)),2))+DEG)
        self._widget.joint2_low.setText(str(jointslimit["joint_2"]["low"])+DEG)
        self._widget.joint2_high.setText(str(jointslimit["joint_2"]["high"])+DEG)

        self._widget.jointSlider_3.sliderReleased.connect(self.setjoint3)
        self._widget.jointSlider_3.valueChanged.connect(self.viewjoint3)
        self._widget.jointSlider_3.setMinimum(jointslimit["joint_3"]["low"])
        self._widget.jointSlider_3.setMaximum(jointslimit["joint_3"]["high"])
        self._widget.joint3Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(3)),2))+DEG)
        self._widget.joint3_low.setText(str(jointslimit["joint_3"]["low"])+DEG)
        self._widget.joint3_high.setText(str(jointslimit["joint_3"]["high"])+DEG)

        self._widget.jointSlider_4.sliderReleased.connect(self.setjoint4)
        self._widget.jointSlider_4.valueChanged.connect(self.viewjoint4)
        self._widget.jointSlider_4.setMinimum(jointslimit["joint_4"]["low"])
        self._widget.jointSlider_4.setMaximum(jointslimit["joint_4"]["high"])
        self._widget.joint4Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(4)),2))+DEG)
        self._widget.joint4_low.setText(str(jointslimit["joint_4"]["low"])+DEG)
        self._widget.joint4_high.setText(str(jointslimit["joint_4"]["high"])+DEG)

        self._widget.jointSlider_5.sliderReleased.connect(self.setjoint5)
        self._widget.jointSlider_5.valueChanged.connect(self.viewjoint5)
        self._widget.jointSlider_5.setMinimum(jointslimit["joint_5"]["low"])
        self._widget.jointSlider_5.setMaximum(jointslimit["joint_5"]["high"])
        self._widget.joint5Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(5)),2))+DEG)
        self._widget.joint5_low.setText(str(jointslimit["joint_5"]["low"])+DEG)
        self._widget.joint5_high.setText(str(jointslimit["joint_5"]["high"])+DEG)

        self._widget.jointSlider_6.sliderReleased.connect(self.setjoint6)
        self._widget.jointSlider_6.valueChanged.connect(self.viewjoint6)
        self._widget.jointSlider_6.setMinimum(jointslimit["joint_6"]["low"])
        self._widget.jointSlider_6.setMaximum(jointslimit["joint_6"]["high"])
        self._widget.joint6Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(6)),2))+DEG)
        self._widget.joint6_low.setText(str(jointslimit["joint_6"]["low"])+DEG)
        self._widget.joint6_high.setText(str(jointslimit["joint_6"]["high"])+DEG)

        self._widget.gripperSlider.sliderReleased.connect(self.setgripper)
        self._widget.gripperSlider.valueChanged.connect(self.viewgripper)
        self._widget.gripperSlider.setMinimum(jointslimit["gripper"]["low"]*100)
        self._widget.gripperSlider.setMaximum(jointslimit["gripper"]["high"]*100)
        self._widget.gripperBrowser.append(str(round(self.robot.get_gripper_joint_value(),3)))
        self._widget.gripper_low.setText(str(jointslimit["gripper"]["low"]))
        self._widget.gripper_high.setText(str(jointslimit["gripper"]["high"]))

        self._widget.rvizButton.clicked.connect(self.launchrviz)
        self._widget.start_button.clicked.connect(self.playClicked)
        self._widget.stop_button.clicked.connect(self.stopClicked)
        self._widget.pause_button.clicked.connect(self.pauseClicked)
        self._widget.restart_button.clicked.connect(self.restartClicked)

        self._widget.updatefkButton.clicked.connect(self.updatefk)
        self._widget.updateikButton.clicked.connect(self.updateik)

        self.updatefk()
        self.updateik()

        self._widget.respawnButton.clicked.connect(self.respawn_all_objects)

        rospy.Subscriber("/updatepose", Bool, self.updatepose)
        rospy.Subscriber("/gui_message", String, self.browser_callback)
        self.message_pub = rospy.Publisher("/gui_message", String, queue_size=0)
        self.updatepose_pub = rospy.Publisher("/updatepose", Bool, queue_size=0)

        self.startalgorithm_pub = rospy.Publisher("/start_algorithm", Bool, queue_size=0)
        self.stopalgorithm_pub = rospy.Publisher("/stop_algorithm", Bool, queue_size=0)
        self.pausealgorithm_pub = rospy.Publisher("/pause_algorithm", Bool, queue_size=0)
        self.startalgorithm_sub = rospy.Subscriber("/start_algorithm", Bool, self.startalgorithm_callback)
        self.stopalgorithm_sub = rospy.Subscriber("/stop_algorithm", Bool, self.stopalgorithm_callback)
        self.algorithm_is_on = False

    def startalgorithm_callback(self, msg):
        self.updatepose_trigger(True)
        if msg.data == True:
            self.algorithm_is_on = True
            last_time = rospy.Time.now().to_sec()
            while self.algorithm_is_on:
                if rospy.Time.now().to_sec()-last_time > 0.1:
                    self.updatepose_trigger(True)
                    last_time = rospy.Time.now().to_sec()

    def stopalgorithm_callback(self, msg):
        self.algorithm_is_on = False

    def browser_callback(self, msg):
        self._widget.browser.append(msg.data)

    def send_message(self, msg):
        message = String()
        message.data = msg
        self.message_pub.publish(message)

    def updatepose_trigger(self, value):
        msg = Bool()
        msg.data = value
        self.updatepose_pub.publish(msg)

    def respawn_all_objects(self):
        self.robot.modelmanager.respawn_all_objects()

    def updatepose(self, msg):
        if msg.data == True:
            self.updateik()

    def updatefk(self):
        self._widget.joint1Browser.clear()
        self._widget.joint1Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(1)),2))+DEG)
        self._widget.jointSlider_1.setValue(numpy.rad2deg(self.robot.get_joint_value(1)))
        self._widget.joint2Browser.clear()
        self._widget.joint2Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(2)),2))+DEG)
        self._widget.jointSlider_2.setValue(numpy.rad2deg(self.robot.get_joint_value(2)))
        self._widget.joint3Browser.clear()
        self._widget.joint3Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(3)),2))+DEG)
        self._widget.jointSlider_3.setValue(numpy.rad2deg(self.robot.get_joint_value(3)))
        self._widget.joint4Browser.clear()
        self._widget.joint4Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(4)),2))+DEG)
        self._widget.jointSlider_4.setValue(numpy.rad2deg(self.robot.get_joint_value(4)))
        self._widget.joint5Browser.clear()
        self._widget.joint5Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(5)),2))+DEG)
        self._widget.jointSlider_5.setValue(numpy.rad2deg(self.robot.get_joint_value(5)))
        self._widget.joint6Browser.clear()
        self._widget.joint6Browser.append(str(round(numpy.rad2deg(self.robot.get_joint_value(6)),2))+DEG)
        self._widget.jointSlider_6.setValue(numpy.rad2deg(self.robot.get_joint_value(6)))
        self._widget.gripperBrowser.clear()
        self._widget.gripperBrowser.append(str(round(self.robot.get_gripper_joint_value(),3)))
        self._widget.gripperSlider.setValue(self.robot.get_gripper_joint_value()*100)

    def updateik(self):
        roll, pitch, yaw, x, y, z = self.robot.get_arm_pose()
        self._widget.xEdit.setText(str(round(x,4)))
        self._widget.yEdit.setText(str(round(y,4)))
        self._widget.zEdit.setText(str(round(z,4)))
        self._widget.rollEdit.setText(str(round(numpy.rad2deg(roll),2)))
        self._widget.pitchEdit.setText(str(round(numpy.rad2deg(pitch),2)))
        self._widget.yawEdit.setText(str(round(numpy.rad2deg(yaw),2)))

    def launchrviz(self):
        os.system("gnome-terminal -x sh -c \"roslaunch rqt_industrial_robot rviz.launch\"")

    def algorithm_trigger(self, pub, value):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def playClicked(self):
        self.send_message("Start Algorithm")
        self.algorithm_trigger(self.startalgorithm_pub, True)

    def stopClicked(self):
        self.send_message("Stopping Algorithm")
        self.algorithm_trigger(self.stopalgorithm_pub, True)
        self.algorithm_trigger(self.startalgorithm_pub, False)

    def pauseClicked(self):
        self.send_message("Pausing Algorithm")
        self.algorithm_trigger(self.pausealgorithm_pub, True)

    def restartClicked(self):
        self.send_message("Retart Algorithm")
        self.algorithm_trigger(self.pausealgorithm_pub, False)

    def update(self):
        while True:
            self.updatefk()
            self.updateik()
            print("updating")
            time.sleep(1)

    def plan(self):
        self.set_x()
        self.set_y()
        self.set_z()
        self.set_roll()
        self.set_pitch()
        self.set_yaw()

        self.robot.plan()

    def execute(self):
        last_joints = self.robot.get_joints_value()
        self.robot.execute()

        # print("start moving")
        # update after robot stops moving
        while abs(sum(last_joints)-sum(self.robot.get_joints_value()) > 1e-10):
            last_joints = self.robot.get_joints_value()
            #print(last_joints)
            time.sleep(0.5)

        self.updatefk()
        self.updateik()

        time.sleep(1.5)
        # print("double check movement")

        # update after robot stops moving
        while abs(sum(last_joints)-sum(self.robot.get_joints_value()) > 1e-10):
            last_joints = self.robot.get_joints_value()
            #print(last_joints)
            time.sleep(0.5)

        self.updatefk()
        self.updateik()
    
    def planexe(self):
        last_joints = self.robot.get_joints_value()

        self.set_x()
        self.set_y()
        self.set_z()
        self.set_roll()
        self.set_pitch()
        self.set_yaw()
        self.robot.plan()
        self.robot.execute()

        # print("start moving")
        # update after robot stops moving
        while abs(sum(last_joints)-sum(self.robot.get_joints_value()) > 1e-10):
            last_joints = self.robot.get_joints_value()
            #print(last_joints)
            time.sleep(0.5)

        self.updatefk()
        self.updateik()
        
        time.sleep(1.5)
        # print("double check movement")

        # update after robot stops moving
        while abs(sum(last_joints)-sum(self.robot.get_joints_value()) > 1e-10):
            last_joints = self.robot.get_joints_value()
            #print(last_joints)
            time.sleep(0.5)

        self.updatefk()
        self.updateik()

    def stopexe(self):
        self.robot.stop_execution()

        self.updatefk()
        self.updateik()

    def backtohome(self):
        self.robot.back_to_home()
        
        self.updatefk()
        self.updateik()

    def set_x(self):
        self.robot.set_x(float(self._widget.xEdit.text()))

    def set_y(self):
        self.robot.set_y(float(self._widget.yEdit.text()))

    def set_z(self):
        self.robot.set_z(float(self._widget.zEdit.text()))

    def set_roll(self):
        self.robot.set_roll(numpy.deg2rad(float(self._widget.rollEdit.text())))

    def set_pitch(self):
        self.robot.set_pitch(numpy.deg2rad(float(self._widget.pitchEdit.text())))

    def set_yaw(self):
        self.robot.set_yaw(numpy.deg2rad(float(self._widget.yawEdit.text())))

    def viewjoint1(self):
        self._widget.joint1Browser.clear()
        self._widget.joint1Browser.append(str(self._widget.jointSlider_1.value())+DEG)

    def viewjoint2(self):
        self._widget.joint2Browser.clear()
        self._widget.joint2Browser.append(str(self._widget.jointSlider_2.value())+DEG)

    def viewjoint3(self):
        self._widget.joint3Browser.clear()
        self._widget.joint3Browser.append(str(self._widget.jointSlider_3.value())+DEG)

    def viewjoint4(self):
        self._widget.joint4Browser.clear()
        self._widget.joint4Browser.append(str(self._widget.jointSlider_4.value())+DEG)

    def viewjoint5(self):
        self._widget.joint5Browser.clear()
        self._widget.joint5Browser.append(str(self._widget.jointSlider_5.value())+DEG)

    def viewjoint6(self):
        self._widget.joint6Browser.clear()
        self._widget.joint6Browser.append(str(self._widget.jointSlider_6.value())+DEG)

    def viewgripper(self):
        self._widget.gripperBrowser.clear()
        self._widget.gripperBrowser.append(str(self._widget.gripperSlider.value()/100.0))

    def setjoint1(self):
        angle = numpy.deg2rad(self._widget.jointSlider_1.value())
        self.robot.set_arm_joint(1, angle)
        self.updateik()

    def setjoint2(self):
        angle = numpy.deg2rad(self._widget.jointSlider_2.value())
        self.robot.set_arm_joint(2, angle)
        self.updateik()

    def setjoint3(self):
        angle = numpy.deg2rad(self._widget.jointSlider_3.value())
        self.robot.set_arm_joint(3, angle)
        self.updateik()

    def setjoint4(self):
        angle = numpy.deg2rad(self._widget.jointSlider_4.value())
        self.robot.set_arm_joint(4, angle)
        self.updateik()

    def setjoint5(self):
        angle = numpy.deg2rad(self._widget.jointSlider_5.value())
        self.robot.set_arm_joint(5, angle)
        self.updateik()

    def setjoint6(self):
        angle = numpy.deg2rad(self._widget.jointSlider_6.value())
        self.robot.set_arm_joint(6, angle)
        self.updateik()

    def setgripper(self):
        self.robot.move_joint_hand(self._widget.gripperSlider.value()/100.0)

    def getgrippervalue(self):
        return self.robot.get_gripper_joint_value()*100

    def setRobotWrapper(self, robot):
        self.robot = robot