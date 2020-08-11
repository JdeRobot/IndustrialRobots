#!/usr/bin/env python

import sys

from rqt_kinematics.kinematics_module import Kinematics
from rqt_gui.main import Main

plugin = 'rqt_kinematics'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))