import time
import csv
import os.path
import logging

import wpilib
import ctre
from ctre.trajectorypoint import TrajectoryPoint as TalonPoint
from .DriveTrain import DriveTrain


class MotionProfile():
    right_drive_talon:ctre.WPI_TalonSRX
    left_drive_talon:ctre.WPI_TalonSRX


    MIN_POINTS_IN_TALON = 100
    PID_IDX = 0
    TIMEOUT_MS = 10
    RIGHT_FILE_NAME = os.path.dirname(__file__) + "/../points_right_Talon.csv"
    LEFT_FILE_NAME = os.path.dirname(__file__) + "/../points_left_Talon.csv"
    MAX_SPEED = 2.2

    def setup(self):
        logging.basicConfig(filename='log.log', filemode="w", level=logging.INFO)
        with open(self.RIGHT_FILE_NAME, "r") as file:
            self.right_points = file.readlines()
        with open(self.LEFT_FILE_NAME, "r") as file:
            self.left_points = file.readlines()

        self.left = self.right_drive_talon
        self.right = self.left_drive_talon
        logging.warning(self.right.isMotionProfileTopLevelBufferFull())
        logging.warning(self.left.isMotionProfileTopLevelBufferFull())
        self.left.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                self.PID_IDX, self.TIMEOUT_MS)
        self.right.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                self.PID_IDX, self.TIMEOUT_MS)
        # self.right.setSensorPhase(True)
        # self.left.setSensorPhase(False)

        self.right.setInverted(True)

        self.right.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
        self.left.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)


        self.left.config_kF(slotIdx=0, value=10.0, timeoutMs=self.TIMEOUT_MS)
        self.left.config_kP(slotIdx=0, value=10.0, timeoutMs=self.TIMEOUT_MS)
        self.left.config_kI(slotIdx=0, value=10.0, timeoutMs=self.TIMEOUT_MS)
        self.left.config_kD(slotIdx=0, value=10.0, timeoutMs=self.TIMEOUT_MS)
        # self.right.setControlFramePeriod(frame=5, periodMs=self.TIMEOUT_MS)
        # self.left.setControlFramePeriod(frame=5, periodMs=self.TIMEOUT_MS)

        self.right.controlMode = ctre.ControlMode.Velocity
        self.left.controlMode = ctre.ControlMode.Velocity

        try:
            self.left_points = self.fillPoints(self.RIGHT_FILE_NAME, self.right)
            self.right_points = self.fillPoints(self.LEFT_FILE_NAME, self.left)
        except NotImplementedError:
            print("NotImplementedError")
        self.counter = 0

    def reset(self):
        self.counter = 0
        self.right.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
        self.left.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)


    """Push another trajectory point into the top level buffer (which is emptied
    into the motor controller's bottom buffer as room allows).

    :param trajPt: to push into buffer.
        The members should be filled in with these values...

        position:  servo position in sensor units.
        velocity:  velocity to feed-forward in sensor units per 100ms.
        profileSlotSelect0:  Which slot to get PIDF gains. PID is used for position servo. F is used
            as the Kv constant for velocity feed-forward. Typically this is hardcoded
            to the a particular slot, but you are free gain schedule if need be.
            Choose from [0,3]
        profileSlotSelect1: Which slot to get PIDF gains for auxiliary PId.
            This only has impact during MotionProfileArc Control mode.
            Choose from [0,1].
        isLastPoint:  set to nonzero to signal motor controller to keep processing this
            trajectory point, instead of jumping to the next one
            when timeDurMs expires.  Otherwise MP executer will
            eventually see an empty buffer after the last point
            expires, causing it to assert the IsUnderRun flag.
            However this may be desired if calling application
            never wants to terminate the MP.
        zeroPos:  set to nonzero to signal motor controller to "zero" the selected
            position sensor before executing this trajectory point.
            Typically the first point should have this set only thus
            allowing the remainder of the MP positions to be relative to
            zero.
        timeDur: Duration to apply this trajectory pt.
            This time unit is ADDED to the exising base time set by
            configMotionProfileTrajectoryPeriod().

    :returns: CTR_OKAY if trajectory point push ok. ErrorCode if buffer is
        full due to kMotionProfileTopBufferCapacity.
    """
    @staticmethod
    def fillPoints(filename, talon):
        points = []
        position = 0
        talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(position=0,
                                                                               velocity=0,
                                                                               auxiliaryPos=0.0,
                                                                               profileSlotSelect0=0,
                                                                               profileSlotSelect1=0,
                                                                               isLastPoint=False,
                                                                               zeroPos=True,
                                                                               timeDur=0))
        with open(filename, "r") as csv_file:
            csv_reader = csv.reader(csv_file)
            for line in csv_reader:
                #TrajectoryPoint = namedtuple("TrajectoryPoint",
                 #                            ["position", "velocity", "auxiliaryPos", "profileSlotSelect0",
                  #                            "profileSlotSelect1", "isLastPoint", "zeroPos", "timeDur"])
                position = float(line[0])
                points.append([float(line[0]), float(line[1]), float(line[2])])
                talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(position=float(line[0]),
                                                                                       velocity=float(line[1]),
                                                                                       auxiliaryPos=0,
                                                                                       profileSlotSelect0=0,
                                                                                       profileSlotSelect1=0,
                                                                                       isLastPoint=False,
                                                                                       zeroPos=True,
                                                                                       timeDur=int(line[2])))



        talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(position=position,
                                                                               velocity=0,
                                                                               auxiliaryPos=0,
                                                                               profileSlotSelect0=0,
                                                                               profileSlotSelect1=0,
                                                                               isLastPoint=True,
                                                                               zeroPos=False,
                                                                               timeDur=0))


        for i in range(MotionProfile.MIN_POINTS_IN_TALON):
            talon.processMotionProfileBuffer()
        return points

    def execute(self):
        try:
            self.left.setNeutralMode(ctre.ControlMode.MotionProfile)
            self.right.setNeutralMode(ctre.ControlMode.MotionProfile)
            self.left.sendMode = self.right.sendMode = ctre.ControlMode.MotionProfile
            self.left.controlMode = self.right.controlMode = ctre.ControlMode.MotionProfile
            for i in range(3):

                print(self.right.getMotionProfileStatus())
                print(self.right.getActiveTrajectoryAll())
                print(len(self.right.getActiveTrajectoryAll()))
                # with open("mylogger.log", "w") as f:
                #     f.write(self.right.getMotionProfileStatus())

                self.right.processMotionProfileBuffer()
                self.left.processMotionProfileBuffer()
        except NotImplementedError:
            print("NotImplementedError")

        return None
        if self.counter < len(self.right_points):
            self.handleMotorSet(self.right, self.right_points[self.counter][1] / self.MAX_SPEED)
            self.handleMotorSet(self.left, self.left_points[self.counter][1] / self.MAX_SPEED)
            self.counter += 1

    @staticmethod
    def handleMotorSet(motor, speed):
        if speed > 1:
            logging.info("too high speed")
            motor.set(1)
        elif speed < -1:
            logging.info("too low speed")
            motor.set(-1)
        else:
            motor.set(speed)