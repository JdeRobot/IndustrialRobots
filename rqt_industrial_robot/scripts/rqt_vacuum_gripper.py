#!/usr/bin/env python

import sys

from rqt_vacuum_gripper.vacuum_gripper_module import VacuumGripper
from rqt_gui.main import Main

plugin = 'rqt_vacuum_gripper'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))