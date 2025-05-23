# abb_egm_rws_managers

![CI - Ubuntu Bionic](https://github.com/ros-industrial/abb_egm_rws_managers/workflows/CI%20-%20Ubuntu%20Bionic/badge.svg)
![CI - Ubuntu Focal](https://github.com/ros-industrial/abb_egm_rws_managers/workflows/CI%20-%20Ubuntu%20Focal/badge.svg)
[![Github Issues](https://img.shields.io/github/issues/ros-industrial/abb_egm_rws_managers.svg)](http://github.com/ros-industrial/abb_egm_rws_managers/issues)

[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

**Please note that this package has not been productized, and that academia is the intended audience.**\
**The package is provided "as-is", and as such no more than limited support can be expected.**

## Overview

C++ library intended for use with [abb_robot_driver](https://github.com/ros-industrial/abb_robot_driver), with the purpose of encapsulating *ROS agnostic* components.

The *ROS agnostic* components are primarily:

- Higher level management of *Externally Guided Motion* (`EGM`) communication sessions.
- Higher level management of *Robot Web Services* (`RWS`) communication sessions.
- Structured description definitions of an ABB robot controller (*i.e. see the [robot_controller_description.proto](proto/robot_controller_description.proto) file*).

Please see [abb_robot_driver](https://github.com/ros-industrial/abb_robot_driver) for more details.

### Requirements

- `RobotWare` version `6.07.01` or higher (less than `7.0`).

Please see the underlying [abb_libegm](https://github.com/ros-industrial/abb_libegm) and [abb_librws](https://github.com/ros-industrial/abb_librws) packages for more details.

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
