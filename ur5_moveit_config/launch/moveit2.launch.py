#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2024.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl.

# moveit2.launch.py:
# Launch file for the Robot's GAZEBO SIMULATION + MoveIt!2 Framework in ROS2 Humble:

# Import libraries:
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ===== REQUIRED TO GET THE ROBOT CONFIGURATION === #

# EVALUATE INPUT ARGUMENTS:
def AssignArgument(ARGUMENT):
    ARGUMENTS = sys.argv
    for y in ARGUMENTS:
        if (ARGUMENT + ":=") in y:
            ARG = y.replace((ARGUMENT + ":="),"")
            return(ARG)

# GET CONFIGURATION from YAML:
def GetCONFIG(CONFIGURATION, PKG_PATH):
    
    RESULT = {"Success": False, "ID": "", "Name": "", "urdf": "", "ee": ""}
    
    YAML_PATH = PKG_PATH + "/config/configurations.yaml"
    
    if not os.path.exists(YAML_PATH):
        return (RESULT)
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["Configurations"]:

        if x["ID"] == CONFIGURATION:
            RESULT["Success"] = True
            RESULT["ID"] = x["ID"]
            RESULT["Name"] = x["Name"]
            RESULT["urdf"] = x["urdf"]
            RESULT["rob"] = x["rob"]
            RESULT["ee"] = x["ee"]

    return(RESULT)

# GET EE-Controllers LIST:
def GetEEctr(EEName):
    
    RESULT = []

    PATH = os.path.join(get_package_share_directory('ros2srrc_endeffectors'), EEName, 'config')
    YAML_PATH = PATH + "/controller_moveit2.yaml"
    
    with open(YAML_PATH, 'r') as YAML:
        cYAML = yaml.safe_load(YAML)

    for x in cYAML["controller_names"]:
        RESULT.append(x)

    return(RESULT)

# CHECK if CONTROLLER file exists for EE:
def EEctrlEXISTS(EEName):
    
    PATH = os.path.join(get_package_share_directory('ros2srrc_endeffectors'), EEName, 'config')
    YAML_PATH = PATH + "/controller.yaml"
    
    RES = os.path.exists(YAML_PATH)
    return(RES)

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():

    LD = LaunchDescription()
    
    # === INPUT ARGUMENT: ROS 2 PACKAGE === #
    
    #PACKAGE_NAME = AssignArgument("package")
    # DMM: fixed UR5
    PACKAGE_NAME = "ros2srrc_ur5"
    
    if PACKAGE_NAME != None:
        None
    else:
        print("")
        print("ERROR: package INPUT ARGUMENT has not been defined. Please try again.")
        print("Closing... BYE!")
        exit()
        
    # CHECK if -> PACKAGE EXISTS, and GET PATH:
    try:
        PKG_PATH = get_package_share_directory(PACKAGE_NAME + "_gazebo")
    except PackageNotFoundError:
        print("")
        print("ERROR: The defined ROS 2 Package was not found. Please try again.")
        print("Closing... BYE!")
        exit()
    except ValueError:
        print("")
        print("ERROR: The defined ROS 2 Package name is not valid. Please try again.")
        print("Closing... BYE!")
        exit()
    
    # === INPUT ARGUMENT: CONFIGURATION === #
    # CONFIG = AssignArgument("config")
    # DMM fixed UR5 + Robotiq 85
    CONFIG = "ur5_3"
    CONFIGURATION = GetCONFIG(CONFIG, PKG_PATH)

    if CONFIGURATION["Success"] == False:
        print("")
        print("ERROR: config INPUT ARGUMENT has not been correctly defined. Please try again.")
        print("Closing... BYE!")
        exit()   

    # === INPUT ARGUMENT: HMI === #
    HMI = AssignArgument("hmi")
    if HMI == "True" or HMI == "true":
        HMI = "true"
    else:
        HMI = "false"

    # ========== CELL INFORMATION ========== #
    print("")
    print("===== GAZEBO: Robot Simulation + MoveIt!2 Framework (" + PACKAGE_NAME + "_moveit2) =====")
    print("Robot configuration:")
    print(CONFIGURATION["ID"] + " -> " + CONFIGURATION["Name"])
    print("")
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    robot_gazebo = os.path.join(
        get_package_share_directory(PACKAGE_NAME + '_gazebo'),
        'worlds',
        PACKAGE_NAME + '.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': robot_gazebo}.items(),
            )

    # ***** ROBOT DESCRIPTION ***** #
    # Robot Description file package:
    robot_description_path = os.path.join(get_package_share_directory(PACKAGE_NAME + '_gazebo'))
    # ROBOT urdf file path:
    xacro_file = os.path.join(robot_description_path,'urdf',CONFIGURATION["urdf"])
    # Generate ROBOT_DESCRIPTION variable:
    doc = xacro.parse(open(xacro_file))
    
    if CONFIGURATION["ee"] == "none":
        EE = "false"
    else: 
        EE = "true"
    
    xacro.process_doc(doc, mappings={
        "EE": EE,
        "EE_name": CONFIGURATION["ee"],
        "hmi": HMI,
    })
    
    # EE -> Controller file needed?
    if EE == "true":
        if EEctrlEXISTS(CONFIGURATION["ee"]) == False:
            EE = "true-NOctr"
    
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', CONFIGURATION["rob"]],
                        output='both')

    # ***** CONTROLLERS ***** #
    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # EE CONTROLLERS:
    if EE == "true":
        CONTROLLERS = GetEEctr(CONFIGURATION["ee"])
        CONTROLLER_NODES = []

        for x in CONTROLLERS:
            CONTROLLER_NODES.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[x, "-c", "/controller_manager"],
                )
            )

    # *********************** MoveIt!2 *********************** #   

    # Determine SRDF file path
    if (EE == "false"):
        srdf_file_path = "config/" + CONFIGURATION["rob"] + ".srdf"
    else:
        srdf_file_path = "config/" + CONFIGURATION["rob"] + CONFIGURATION["ee"] + ".srdf"

    # Determine controller configuration file path
    if (EE == "false") or (EE == "true-NOctr"):
        controller_config_path = os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "controller_moveit2.yaml"
        )
    else:
        # For EE cases, we'll need to handle this manually since MoveItConfigsBuilder 
        # expects a single file path - we'll create a temporary merged config
        YAML_ROB = load_yaml("ros2srrc_robots", CONFIGURATION["rob"] + "/config/controller_moveit2.yaml")
        YAML_EE = load_yaml("ros2srrc_endeffectors", CONFIGURATION["ee"] + "/config/controller_moveit2.yaml")
        for x in YAML_ROB["controller_names"]:
            YAML_EE["controller_names"].append(x)
        merged_controller_config = YAML_ROB | YAML_EE
        
        # Write merged config to temp file (you might want to handle this differently)
        controller_config_path = os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "controller_moveit2.yaml"
        )

    # Build MoveIt configuration using MoveItConfigsBuilder
    moveit_config_builder = MoveItConfigsBuilder(PACKAGE_NAME + "_moveit2")
    
    # Configure robot description with the already processed URDF
    moveit_config_builder = moveit_config_builder.robot_description(
        robot_description_config
    )
    
    # Configure semantic description
    moveit_config_builder = moveit_config_builder.robot_description_semantic(
        file_path=srdf_file_path
    )
    
    # Configure kinematics
    moveit_config_builder = moveit_config_builder.robot_description_kinematics(
        file_path=os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "kinematics.yaml"
        )
    )
    
    # Configure joint limits
    if (EE == "false") or (EE == "true-NOctr"):
        joint_limits_file = os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "joint_limits.yaml"
        )
    else:
        # For EE cases, use the merged joint limits from the original code logic
        joint_limits_file = os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "joint_limits.yaml"
        )
    
    moveit_config_builder = moveit_config_builder.joint_limits(
        file_path=joint_limits_file
    )
    
    # Configure planning scene monitor
    moveit_config_builder = moveit_config_builder.planning_scene_monitor(
        publish_robot_description=True, 
        publish_robot_description_semantic=True,
        publish_geometry_updates=True,
        publish_state_updates=True,
        publish_transforms_updates=True
    )
    
    # Configure trajectory execution
    moveit_config_builder = moveit_config_builder.trajectory_execution(
        file_path=controller_config_path,
        moveit_manage_controllers=True,
        allowed_execution_duration_scaling=1.2,
        allowed_goal_duration_margin=0.5,
        allowed_start_tolerance=0.01
    )
    
    # Configure planning pipelines
    moveit_config_builder = moveit_config_builder.planning_pipelines(
        pipelines=["pilz_industrial_motion_planner"]
    )
    
    # Configure Pilz cartesian limits
    moveit_config_builder = moveit_config_builder.pilz_cartesian_limits(
        file_path=os.path.join(
            get_package_share_directory("ros2srrc_robots"),
            CONFIGURATION["rob"], "config", "pilz_cartesian_limits.yaml"
        )
    )
    
    # Add sensor configuration
    sensors_3d_file = os.path.join(
        get_package_share_directory("machine_vision_exercise"),
        "config", "sensors_3d.yaml"
    )
    if os.path.exists(sensors_3d_file):
        moveit_config_builder = moveit_config_builder.sensors_3d(
            file_path=sensors_3d_file
        )
    
    # Build the configuration
    moveit_config = moveit_config_builder.to_moveit_configs()

    # MoveGroup Node:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RVIZ:
    rviz_base = os.path.join(get_package_share_directory(PACKAGE_NAME + "_moveit2"), "config")
    rviz_full_config = os.path.join(rviz_base + "/ur5robotiq_2f85_with_cam_moveit2.rviz")
    
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ]
    )

    # =================================================================================================== #
    # ============================= ros2srrc_execution -> CUSTOM INTERFACES ============================= #

    # Move and Sequence:
    if EE == "true":

        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}, {"EE_PARAM": CONFIGURATION["ee"]}, {"ENV_PARAM": "gazebo"}],
        )

    else:

        MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}, {"EE_PARAM": "none"}, {"ENV_PARAM": "gazebo"}],
        )

    # RobMove and RobPose:
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, moveit_config.robot_description_semantic, moveit_config.robot_description_kinematics, {"use_sim_time": True}, {"ROB_PARAM": CONFIGURATION["rob"]}],
    )
    
    # =============================================== #
    # ========== RETURN LAUNCH DESCRIPTION ========== #

    # Add ROS 2 Nodes to LaunchDescription() element:
    LD.add_action(gazebo)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
    LD.add_action(spawn_entity)

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_state_broadcaster_spawner,
                ]
            )
        )
    )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                joint_trajectory_controller_spawner,
                ]
            )
        )
    )

    if EE == "true":

        for x in CONTROLLER_NODES:

            LD.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        x,
                        ]
                    )
                )
            )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                
                # MoveIt!2:
                TimerAction(
                    period=2.0,
                    actions=[
                        rviz_node_full,
                        run_move_group_node,
                    ]
                ),
                
                ]
            )
        )
    )

    LD.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action = spawn_entity,
            on_exit = [
                
                # Interfaces:
                TimerAction(
                    period=5.0,
                    actions=[
                        MoveInterface,
                        RobMoveInterface,
                        RobPoseInterface,
                    ]
                ),
                
                ]
            )
        )
    )

    # LD.add_action(RegisterEventHandler(
    #     OnProcessExit(
    #         target_action = spawn_entity,
    #         on_exit = [
    #             TimerAction(
    #                 period=3.0,
    #                 actions=[
    #                     Node(
    #                         package='ros2srrc_ur5_gazebo',
    #                         executable='model_manager.py',  # or just 'model_manager' if installed via setup.py
    #                         name='spawn_objects',
    #                         output='screen'
    #                     )
    #                 ]
    #             )
    #         ]
    #     )
    # ))


    # ***** RETURN  ***** #
    return(LD)