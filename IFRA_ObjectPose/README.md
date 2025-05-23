<!-- 

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
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) ObjectPose Plugin for ROS2-Gazebo Simulation. URL: https://github.com/IFRA-Cranfield/IFRA_ObjectPose.

-->

<div id="top"></div>



<br />
<div align="center">

  <h2 align="center">IFRA_ObjectPose - Gazebo-ROS2 Plugin</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p>
</div>

<br />

## ABOUT

### Intelligent Flexible Robotics and Assembly Group

The IFRA (Intelligent Flexible Robotics and Assembly) Group is part of the Centre for Robotics and Assembly at Cranfield University.

IFRA Group pushes technical boundaries. At IFRA we provide high tech automation & assembly solutions, and we support smart manufacturing with Smart Industry technologies and solutions. Flexible Manufacturing Systems (FMS) are a clear example. They can improve overall operations and throughput quality by adapting to real-time changes and situations, supporting and pushing the transition towards flexible, intelligent and responsive automation, which we continuously seek and support.

The IFRA Group undertakes innovative research to design, create and improve Intelligent, Responsive and Flexible automation & assembly solutions, and this series of GitHub repositories provide background information and resources of how these developments are supported.

__SOCIAL MEDIA__:

IFRA-Cranfield:
- YouTube: https://www.youtube.com/@IFRACranfield
- LinkedIn: https://www.linkedin.com/in/ifra-cranfield/

Centre for Robotics and Assembly:
- Instagram: https://www.instagram.com/cranfieldrobotics/
- Facebook: https://www.facebook.com/cranfieldunirobotics/
- YouTube: https://www.youtube.com/@CranfieldRobotics
- LinkedIn: https://www.linkedin.com/company/cranfieldrobotics/
- Website: https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly 


### IFRA_ObjectPose Repository

IFRA_ObjectPose is a simple ROS2-Gazebo plugin that publishes the exact WorldPose of any model in Gazebo to a ROS2 Topic. Thanks to this plugin, it is possible to get the (x y z)(yaw pitch roll) information of any object from Gazebo in order to use it within the ROS2 Source Code. The repository contains a Gazebo package as well, where the plugin can be easily tested and verified with a simple box.

<br />

## INSTALLATION

All ROS2 Packages in this GitHub repository have been tested and verified in ROS2 Foxy and Humble (the plugin has not been tested in other ROS2 distributions yet). It can be easily downloaded and installed by executing the following commands:

```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/IFRA_ObjectPose.git
cd ~/dev_ws
colcon build
```

<br />

## USAGE

The "ros2_objectpose" Gazebo plugin is a MODEL PLUGIN. Thus, it has to be defined inside a model (.urdf or .sdf), and it is loaded when the model is spawned into the Gazebo World. 

The following tag must be added to the .urdf file of any model that wants to be monitored:

```sh
<!-- Input parameters: -->
<xacro:arg name="name" default="none"/>
<xacro:property name="name" value="$(arg name)"/>

<!-- LOAD ObjectPose Gazebo (ROS2) PLUGIN: -->
<gazebo>
  <plugin name="ros2_objectpose_plugin" filename="libros2_objectpose_plugin.so" >
    <ros>
      <namespace>${name}</namespace>
    </ros>
  </plugin>
</gazebo>
```

The object pose will be published to the /{MODELNAME}/ObjectPose topic as soon as the object is spawned to the Gazebo environment. {MODELNAME} is defined in the CommandLine, when spawning the object to gazebo using the [SpawnObject.py](https://github.com/IFRA-Cranfield/IFRA_ObjectPose/blob/main/ros2_objectpose/python/SpawnObject.py) script.

The following command must be executed in order to spawn any object to the Gazebo World:
```sh
  ros2 run ros2_objectpose SpawnObject.py --package "objectpose_gazebo" --urdf "{.urdf FILE}" --name "{MODELNAME}" --x {x} --y {y} --z {z}
```

NOTE: It is assumed that the .urdf file of the object is contained inside the /urdf folder.

__EXAMPLE: box.urdf__

1. Launch the ObjectPose Gazebo environment:
    ```sh
    ros2 launch objectpose_gazebo objectpose.launch.py
    ```

2. Spawn the BOX to the Gazebo world:
    ```sh
    ros2 run ros2_objectpose SpawnObject.py --package "objectpose_gazebo" --urdf "box.urdf" --name "box" --x 0.5 --y -0.5 --z 0.5
    ```

3. Check the ObjectPose in the ROS2 Terminal shell:
    ```sh
    ros2 topic echo /box/ObjectPose
    ```

4. Manually move the box in Gazebo, and check how the pose information is automatically updated.

<br />

## License

<p>
  Intelligent Flexible Robotics and Assembly Group
  <br />
  Created on behalf of the IFRA Group at Cranfield University, United Kingdom
  <br />
  E-mail: IFRA@cranfield.ac.uk 
  <br />
  <br />
  Licensed under the Apache-2.0 License.
  <br />
  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
  <br />
  <br />
  <a href="https://www.cranfield.ac.uk/">Cranfield University</a>
  <br />
  School of Aerospace, Transport and Manufacturing (SATM)
  <br />
    <a href="https://www.cranfield.ac.uk/centres/centre-for-robotics-and-assembly">Centre for Robotics and Assembly</a>
  <br />
  College Road, Cranfield
  <br />
  MK43 0AL, Bedfordshire, UK
  <br />
</p>

<br />

## Cite our work

<p>
  You can cite our work with the following statement:
  <br />
  IFRA-Cranfield (2023) ObjectPose Plugin for ROS2-Gazebo Simulation. URL: https://github.com/IFRA-Cranfield/IFRA_ObjectPose.
</p>

<br />

## Contact

<p>
  Mikel Bueno Viso - Research Assistant in Intelligent Automation at Cranfield University
  <br />
  E-mail: Mikel.Bueno-Viso@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/mikel-bueno-viso/
  <br />
  Profile: https://www.cranfield.ac.uk/people/mikel-bueno-viso-32884399
  <br />
  <br />
  Dr. Seemal Asif - Lecturer in Artificial Intelligence and Robotics at Cranfield University
  <br />
  E-mail: s.asif@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/dr-seemal-asif-ceng-fhea-miet-9370515a/
  <br />
  Profile: https://www.cranfield.ac.uk/people/dr-seemal-asif-695915
  <br />
  <br />
  Professor Phil Webb - Professor of Aero-Structure Design and Assembly at Cranfield University
  <br />
  E-mail: p.f.webb@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/phil-webb-64283223/
  <br />
  Profile: https://www.cranfield.ac.uk/people/professor-phil-webb-746415 
  <br />
</p>