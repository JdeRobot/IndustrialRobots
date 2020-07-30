import threading
import numpy


class Algorithm:
    def __init__(self):
        self.is_on = False

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def myalgorithm(self, stopevent, pauseevent):
        self.pick_place.buildmap()

        # insert following two lines where you want to stop the algorithm 
        # with the stop button in GUI
        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        ### pick green cylinder
        position = self.pick_place.get_object_position("green", "cylinder")
        position.z = -0.145
        self.pick_place.pickup("green_cylinder", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        position = self.pick_place.get_target_position("target6")
        # position.z += 0.1
        self.pick_place.place("green_cylinder", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        ### pick red sphere
        position = self.pick_place.get_object_position("red", "sphere")
        position.z += 0.04
        self.pick_place.pickup("red_ball", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        position = self.pick_place.get_target_position("target5")
        # position.z += 0.1
        self.pick_place.place("red_ball", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        ### pick blue sphere
        position = self.pick_place.get_object_position("blue", "sphere")
        position.z += 0.04
        self.pick_place.pickup("blue_ball", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        position = self.pick_place.get_target_position("target10")
        # position.z += 0.1
        self.pick_place.place("blue_ball", position)

        while not pauseevent.isSet():
            if not stopevent.isSet():
                return

        self.pick_place.back_to_home()
        self.pick_place.send_message("Algorithm finished")
