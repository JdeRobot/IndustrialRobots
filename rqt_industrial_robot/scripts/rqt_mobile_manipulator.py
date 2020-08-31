#!/usr/bin/env python

import sys

from rqt_mobile_manipulator.mobile_manipulator_module import MobileManipulator
from rqt_gui.main import Main

plugin = 'rqt_mobile_manipulator'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))