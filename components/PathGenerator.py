from math import pi
import os.path
import pickle
from numpy import linspace

import wpilib
from magicbot import tunable
from networktables import NetworkTables


class PathGenerator():
    ROBOT_WIDTH = 0.69
    DISTANCE = 0.005

    def __init__(self):
        self.table = NetworkTables.getTable("SmartDashboard")
        self.right_path = [0]
        self.left_path = [0]

        pickle_file = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')
        if wpilib.RobotBase.isSimulation():
            self.createPath()
            with open(os.path.join(os.path.dirname(__file__) + "right_points.txt"), "w") as f:
                f.write(str(self.right_path))
            with open(pickle_file, "wb") as fp:
                pickle.dump({"right": self.right_path, "left": self.left_path}, fp)
        else:
            with open(os.path.join(os.path.dirname(__file__), "points_right_Talon.csv"), "r") as f:
                for line in f.readlines():
                    self.right_path.append(float(line.replace("\n", "")))

            with open(os.path.join(os.path.dirname(__file__), "points_left_Talon.csv"), "r") as f:
                for line in f.readlines():
                    self.left_path.append(float(line.replace("\n", "")))
            #
            # with open(pickle_file, "rb") as fp:
            #     points = pickle.load(fp)
            #     self.right_path = points["right"]
            #     self.left_path = points["left"]

    def createPath(self):
        self.right_path = []
        self.left_path = []
        self.add_straight(1)
        self.add_circular(radius=1, angle=-90)
        self.add_straight(3)
        self.add_circular(radius=1, angle=90)

        if wpilib.RobotBase.isSimulation():
            print(os.path.dirname(__file__))
            with open(os.path.join(os.path.dirname(__file__), "right_points.txt"), "w") as f:

                string = ""
                for point in self.right_path:
                    string += str(point) + "\n"
                f.write(string)
                print("wrote to file", os.path.join(os.path.dirname(__file__), "right_points.txt"))
            with open(os.path.join(os.path.dirname(__file__), "left_points.txt"), "w") as f:
                string = ""
                for point in self.left_path:
                    string += str(point) + "\n"
                f.write(string)


    def add_straight(self, len):
        self.right_path = self._add_part(self.right_path, len)
        self.left_path = self._add_part(self.left_path, len)

    @classmethod
    def _add_part(cls, points:list, dist, iter_num=None):
        if len(points) > 0:
            last = points[-1]
        else:
            last = 0

        if iter_num == None:
            for point in cls.drange(last, last + dist, cls.DISTANCE):
                points.append(point)
            if points[-1] < last + dist - (cls.DISTANCE / 2):
                points.append(last + dist)
        else:
            points += linspace(start=last, stop=last + dist, num=iter_num).tolist()
        return points

    @staticmethod
    def drange(start, stop, step):
        r = start
        while r < stop:
            yield r
            r += step

    def add_circular(self, radius, angle):
        '''

        :param radius: radius of the circule to drive over it's radius
        :param angle: clockwhise angle
        :return:
        '''
        inner_radius = radius - (self.ROBOT_WIDTH / 2)
        outter_radius = radius + (self.ROBOT_WIDTH / 2)
        arrow = 2 * pi * (angle / 360)
        if angle > 0:
            size = len(self.right_path)
            self.right_path = self._add_part(self.right_path, inner_radius * arrow)
            self.left_path = self._add_part(self.left_path, outter_radius * arrow, iter_num=len(self.right_path) - size)
        elif angle < 0:
            size = len(self.left_path)
            self.left_path = self._add_part(self.left_path, inner_radius * -1 * arrow)
            self.right_path = self._add_part(self.right_path, outter_radius * -1 * arrow, iter_num=len(self.left_path) - size)

