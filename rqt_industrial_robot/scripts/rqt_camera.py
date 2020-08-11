#!/usr/bin/env python

import sys

from rqt_camera.camera_module import CameraViewer
from rqt_gui.main import Main

plugin = 'rqt_camera'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))