# abb_robot_driver_interfaces

[![Github Issues](https://img.shields.io/github/issues/ros-industrial/abb_robot_driver_interfaces.svg)](http://github.com/ros-industrial/abb_robot_driver_interfaces/issues)

[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: vendor](https://img.shields.io/badge/support%20level-vendor-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

**Please note that the included packages have not been productized, and that academia is the intended audience.**\
**The packages are provided "as-is", and as such no more than limited support can be expected.**

## Overview

ROS2 packages with ROS message and service definitions, intended for use with [abb_ros2](https://github.com/PickNikRobotics/abb_ros2).

The included packages are briefly described in the following table:

| Package | Description |
| --- | --- |
| [abb_robot_msgs](abb_robot_msgs) | ROS message and service definitions representing basic interaction with ABB robots:<br><ul><li>Reading system states (*e.g. motors on/off and if auto/manual mode is active*).</li><li>Starting and stopping `RAPID` program execution.</li><li>Reading and writing of `RAPID` data symbols (*e.g. variables and constants*).</li><li>Reading and writing of IO-signals.</li><li>And more.</li></ul> |
| [abb_rapid_msgs](abb_rapid_msgs) | ROS message definitions representing complex `RAPID` data structures:<br><ul><li>`loaddata`.</li><li>`tooldata`.</li><li>`wobjdata`.</li><li>`jointtarget`.</li><li>`robtarget`.</li><li>And more.</li></ul> |
| [abb_egm_msgs](abb_egm_msgs) | ROS message definitions representing data related to *Externally Guided Motion* (`EGM`) communication channels. |
| [abb_rapid_sm_addin_msgs](abb_rapid_sm_addin_msgs) | ROS message and service definitions representing interaction with `RAPID` program instances of the `RobotWare` [StateMachine Add-In](https://robotapps.robotstudio.com/#/viewApp/c163de01-792e-4892-a290-37dbe050b6e1):<br><ul><li>Reading runtime states of the `RAPID` program instances (*i.e. the state machines*).</li><li>Starting and stopping `EGM` communication sessions.</li><li>Reading and writing of `EGM` `RAPID` settings.</li><li>Opening/closing `SmartGripper` fingers.</li><li>And more.</li></ul> |

Please see [abb_ros2](https://github.com/PickNikRobotics/abb_ros2) for more details.

## Acknowledgements

### ROSIN Project

<p>
  <a href="http://rosin-project.eu">
    <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="50" align="left">
  </a>
  The core development has been made within the European Union's Horizon 2020 project: ROSIN - ROS-Industrial Quality-Assured Robot Software Components (see http://rosin-project.eu for more info).
  <br><br>
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" alt="eu_flag" height="50" align="left">
  The ROSIN project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement no. 732287.
</p>

*The opinions expressed here reflects only the author's view and reflects in no way the European Commission's opinions. The European Commission is not responsible for any use that may be made of the contained information.*

### Special Thanks

Special thanks to [gavanderhoorn](https://github.com/gavanderhoorn) for guidance with open-source practices and conventions.
