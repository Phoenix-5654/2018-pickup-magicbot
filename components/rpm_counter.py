import time
import math

import wpilib
import ctre
from .DriveTrain import DriveTrain

class RPMCounter():
    drivetrain:DriveTrain

    PID_IDX = 0
    TIMEOUT_MS = 10
    ITER_NUM = 1000

    GEAR_RATIO = 16.73 #power speed 7.56
    EVO_RATIO = 3
    DIAMETER = 6 * 2.54 * math.pi
    RIGHT_RATIO =  1.2263 / (GEAR_RATIO * DIAMETER * 100)
    LEFT_RATIO = 1 / (EVO_RATIO * DIAMETER * 2 * 10)

    def setup(self):
        self.left = self.drivetrain.drive.leftMotor
        self.left.__class__ = ctre.WPI_TalonSRX

        self.right = self.drivetrain.drive.rightMotor
        self.right.__class__ = ctre.WPI_TalonSRX

        # self.left = ctre.WPI_TalonSRX(1)
        # self.right = ctre.WPI_TalonSRX(2)
        self.left_encoder = self.left.getSensorCollection()
        self.right_encoder = self.right.getSensorCollection()

        self.enable = False
        self.starting_time = 0
        self.counter = 0
        # self.reset()


    def reset(self):
        self.right.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
        self.left.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
        self.counter = 0

    def start(self):
        # self.reset()
        self.enable = True
        self.starting_time = time.time()

    def execute(self):
        if (self.enable and self.counter < self.ITER_NUM):
            self.drivetrain.moveX(1)
            self.counter += 1

        elif self.counter == self.ITER_NUM:
            self.end_time = time.time()
            time_diff = (self.end_time - self.starting_time)
            print("time diff=", time_diff)
            print("right dist", self.right_encoder.getQuadraturePosition() * self.RIGHT_RATIO)
            print("rpm right =", ((self.right_encoder.getQuadraturePosition() * self.RIGHT_RATIO) / time_diff))
            print("left dist", self.left_encoder.getQuadraturePosition() * self.LEFT_RATIO)
            print("rpm left =", ((self.left_encoder.getQuadraturePosition() * self.LEFT_RATIO) / time_diff))
            self.counter = 0
            self.enable = False