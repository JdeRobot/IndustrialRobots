#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import os
import copy
import yaml
import rospkg

from moveit_commander import RobotCommander, PlanningSceneInterface
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class Object:
    def __init__(self, relative_pose, abs_pose, height, width, length, shape, color):
        self.relative_pose = relative_pose
        self.abs_pose = abs_pose
        self.height = height
        self.width = width
        self.length = length
        self.shape = shape
        self.color = color


class ModelManager(Node):
    def __init__(self):
        super().__init__('spawn_model')

        self.cli_spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.cli_delete = self.create_client(DeleteEntity, '/delete_entity')

        while not self.cli_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity...')
        while not self.cli_delete.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity...')

        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        rclpy.sleep(1)

        self.object_list = {}

        self.spawn_all_model()

    def pose2msg(self, x, y, z, roll, pitch, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def msg2pose(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        quaternion = (pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw, x, y, z

    def spawn_model(self, model_name, model_pose):
        with open(os.path.join(rospkg.RosPack().get_path('ros2srrc_ur5_gazebo'), 'models', model_name, 'model.sdf'), "r") as f:
            model_xml = f.read()

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.robot_namespace = ""
        req.initial_pose = model_pose
        req.reference_frame = "world"

        self.cli_spawn.call_async(req)

    def delete_model(self, model_name):
        req = DeleteEntity.Request()
        req.name = model_name
        self.cli_delete.call_async(req)

    def clean_scene(self, object_name):
        self.scene.remove_world_object(object_name)

    def respawn_all_objects(self):
        objects_name = self.object_list.keys()
        for object_name in objects_name:
            this_object = self.object_list[object_name]
            print("Respawning {}".format(object_name))

            self.delete_model(object_name)

            roll, pitch, yaw, x, y, z = self.msg2pose(this_object.abs_pose)
            object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)
            self.spawn_model(object_name, object_pose)

            p = PoseStamped()
            p.header.frame_id = self.robot.get_planning_frame()
            p.header.stamp = self.get_clock().now().to_msg()

            self.clean_scene(object_name)
            p.pose = copy.copy(this_object.relative_pose)
            shape = this_object.shape

            if shape == "box":
                x = this_object.length
                y = this_object.width
                z = this_object.height
                size = (x, y, z)
                self.scene.add_box(object_name, p, size)

            elif shape == "cylinder":
                height = this_object.height
                radius = this_object.width / 2
                self.scene.add_cylinder(object_name, p, height, radius)

            elif shape == "sphere":
                radius = this_object.width / 2
                self.scene.add_sphere(object_name, p, radius)

            rclpy.sleep(0.5)

    def spawn_all_model(self):
        filename = os.path.join(rospkg.RosPack().get_path('ros2srrc_ur5_gazebo'), 'config', 'models_info.yaml')
        with open(filename) as file:
            objects_info = yaml.load(file, Loader=yaml.SafeLoader)
            robot_x = objects_info["robot"]["pose"]["x"]
            robot_y = objects_info["robot"]["pose"]["y"]
            robot_z = objects_info["robot"]["pose"]["z"]
            robot_roll = objects_info["robot"]["pose"]["roll"]
            robot_pitch = objects_info["robot"]["pose"]["pitch"]
            robot_yaw = objects_info["robot"]["pose"]["yaw"]

            self.get_logger().info("Spawning Objects in Gazebo")
            objects = objects_info["objects"]
            objects_name = objects.keys()
            for object_name in objects_name:
                name = object_name
                shape = objects[name]["shape"]
                color = objects[name]["color"]

                x = objects[name]["pose"]["x"]
                y = objects[name]["pose"]["y"]
                z = objects[name]["pose"]["z"]
                roll = objects[name]["pose"]["roll"]
                pitch = objects[name]["pose"]["pitch"]
                yaw = objects[name]["pose"]["yaw"]
                object_pose = self.pose2msg(x, y, z, roll, pitch, yaw)
                self.spawn_model(object_name, object_pose)

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = self.get_clock().now().to_msg()

                self.clean_scene(name)
                p.pose.position.x = x - robot_x
                p.pose.position.y = y - robot_y
                p.pose.position.z = z - robot_z

                q = quaternion_from_euler(roll, pitch, yaw)
                p.pose.orientation = Quaternion(*q)

                if shape == "box":
                    x = objects[name]["size"]["x"]
                    y = objects[name]["size"]["y"]
                    z = objects[name]["size"]["z"]
                    p.pose.position.z += z / 2
                    height = z
                    width = y
                    length = x
                    self.object_list[name] = Object(p.pose, object_pose, height, width, length, shape, color)

                elif shape == "cylinder":
                    height = objects[name]["size"]["height"]
                    radius = objects[name]["size"]["radius"]
                    p.pose.position.z += height / 2
                    self.scene.add_cylinder(name, p, height, radius)
                    self.object_list[name] = Object(p.pose, object_pose, height, radius * 2, radius * 2, shape, color)

                elif shape == "sphere":
                    radius = objects[name]["size"]
                    p.pose.position.z += radius
                    self.scene.add_sphere(name, p, radius)
                    self.object_list[name] = Object(p.pose, object_pose, radius * 2, radius * 2, radius * 2, shape, color)

                rclpy.sleep(0.5)

            self.get_logger().info("Spawning Obstacles in planning scene")
            obstacles = objects_info["obstacles"]
            obstacles_name = obstacles.keys()
            for obstacle_name in obstacles_name:
                name = obstacle_name

                p = PoseStamped()
                p.header.frame_id = self.robot.get_planning_frame()
                p.header.stamp = self.get_clock().now().to_msg()

                self.clean_scene(name)
                p.pose.position.x = obstacles[name]["pose"]["x"] - robot_x
                p.pose.position.y = obstacles[name]["pose"]["y"] - robot_y
                p.pose.position.z = obstacles[name]["pose"]["z"] - robot_z

                q = quaternion_from_euler(robot_roll, robot_pitch, robot_yaw)
                p.pose.orientation = Quaternion(*q)

                x = obstacles[name]["size"]["x"]
                y = obstacles[name]["size"]["y"]
                z = obstacles[name]["size"]["z"]
                size = (x, y, z)
                self.scene.add_box(name, p, size)

                rclpy.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    modelmangager = ModelManager()
    rclpy.spin(modelmangager)
    modelmangager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
