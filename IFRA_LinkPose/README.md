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
# IFRA-Cranfield (2023) LinkPose Plugin for ROS2-Gazebo Simulation. URL: https://github.com/IFRA-Cranfield/IFRA_LinkPose.

-->

<div id="top"></div>



<br />
<div align="center">

  <h2 align="center">IFRA_LinkPose - Gazebo-ROS2 Plugin</h2>

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


### IFRA_LinkPose Repository

IFRA_LinkPose is a simple ROS2-Gazebo plugin that publishes the exact WorldPose of any model link in Gazebo to a ROS2 Topic. Thanks to this plugin, it is possible to get the POSITION+ROTATION information of any object from Gazebo in order to use it within the ROS2 Source Code.

<br />

## INSTALLATION

All ROS2 Packages in this GitHub repository have been tested and verified in ROS2 Foxy and Humble (the plugin has not been tested in other ROS2 distributions yet). It can be easily downloaded and installed by executing the following commands:

```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/IFRA_LinkPose.git
cd ~/dev_ws
colcon build
```

<br />

## USAGE

The "ros2_linkpose" Gazebo plugin is a MODEL PLUGIN. Thus, it has to be defined inside a model (.urdf or .sdf), and it is loaded when the model is spawned into the Gazebo World. 

The following tag must be added to the .urdf file of any model that wants to be monitored...

```sh
<gazebo>
  <plugin name="ros2_linkpose_plugin" filename="libros2_linkpose_plugin.so" >
    <link>{LINK_NAME}</link>
  </plugin>
</gazebo>
```

... where {LINK_NAME} is the name of the link whose pose is going to be obtained. The pose information will be published to the /LinkPose_{ModelName}_{LinkName} topic as soon as the model (w/Link) is spawned to the Gazebo environment.

__EXAMPLE: box.urdf__

1. Launch the LinkPose Gazebo environment:
    ```sh
    ros2 launch linkpose_gazebo linkpose.launch.py
    ```

2. Spawn the BOX to the Gazebo world:
    ```sh
    ros2 run ros2_linkpose SpawnObject.py --package "linkpose_gazebo" --urdf "box.urdf" --name "box" --x 0.5 --y -0.5 --z 0.5
    ```

3. Check the LinkPose in the ROS2 Terminal shell:
    ```sh
    ros2 topic echo /LinkPose_box_box_LINK
    ```

4. Manually move the box in Gazebo, and check how the LinkPose information is automatically updated.

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
  IFRA-Cranfield (2023) LinkPose Plugin for ROS2-Gazebo Simulation. URL: https://github.com/IFRA-Cranfield/IFRA_LinkPose.
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
  Dr. Seemal Asif - Senior Lecturer in Artificial Intelligence and Robotics at Cranfield University
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
