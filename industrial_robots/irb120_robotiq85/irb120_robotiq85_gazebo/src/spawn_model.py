#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose2msg(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

if __name__ == "__main__":
    print("start")
    rospy.init_node('spawn_model')
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    model_name = "red_box"

    with open("$GAZEBO_MODEL_PATH/{}/model.sdf".format(model_name), "r") as f:
            model_xml = f.read()

    model_pose = pose2msg(1, 1, 1, 0, 0, 0)

    spawn_model(model_name, model_xml, "", model_pose, "world")