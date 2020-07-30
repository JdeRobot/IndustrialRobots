# API

## Object Class:
- pose
- height
- width
- shape
- color

## Pick_Place Class

### Integrated Solution
- self.get_object_pose(object_name)
- self.pickup(object_name, pose)
- self.place(pose)
- self.back_to_home()

### Basic functions
- self.get_object_width(object_name)  
- self.move_pose_arm(pose)  
- self.move_joint_arm(pose)  
- self.move_joint_hand(joint_value)
- self.generate_grasps(object_name, pose)
- self.pose2msg(roll, pitch, yaw, x, y, z)
- self.msg2pose(pose)

## move_group API:
- self.arm.stop()  
- self.arm.plan()  
- self.arm.pick(object_name, grasp)  
- self.arm.place(placelocation)  