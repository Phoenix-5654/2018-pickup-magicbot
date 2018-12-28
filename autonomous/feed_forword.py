import math
import os

from ctre import WPI_TalonSRX, FeedbackDevice
# from components.PathGenerator import PathGenerator
from networktables import NetworkTables
from magicbot import AutonomousStateMachine, state, timed_state
import wpilib

class FeedForword(AutonomousStateMachine):


    MODE_NAME = 'feed forword'
    DEFAULT = True

    right_drive_talon:WPI_TalonSRX
    left_drive_talon:WPI_TalonSRX

    PID_IDX = 0
    TIMEOUT_MS = 1
    ERROR = 5

    GEAR_RATIO = 16.73  # power speed 7.56
    EVO_RATIO = 3
    DIAMETER = 6 * 2.54 * math.pi
    RIGHT_RATIO =  1.2263 / (GEAR_RATIO * DIAMETER * 100)
    LEFT_RATIO = 1 / (EVO_RATIO * DIAMETER * 2 * 10)

    @state(first=True)
    def setup(self):
        self.findPath()
        self.right = self.right_drive_talon
        self.left = self.left_drive_talon

        self.table = NetworkTables.getTable("SmartDashboard")

        self.right.setSensorPhase(True)
        self.left.setSensorPhase(False)

        self.setup_talon(self.right)
        self.setup_talon(self.left)

        self.counter = 0

        self.enable = True
        self.reset()

        self.next_state_now('drive')

    @classmethod
    def setup_talon(cls, talon:WPI_TalonSRX):
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                           FeedForword.PID_IDX, FeedForword.TIMEOUT_MS)

        talon.setInverted(False)

        talon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_13_Base_PIDF0, 1, cls.TIMEOUT_MS)
        talon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_10_MotionMagic, 1, cls.TIMEOUT_MS)
        talon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature, 1, cls.TIMEOUT_MS)

        talon.configNominalOutputForward(0,cls.TIMEOUT_MS)
        talon.configNominalOutputReverse(0, cls.TIMEOUT_MS)
        talon.configPeakOutputForward(1, cls.TIMEOUT_MS)
        talon.configPeakOutputReverse(-1, cls.TIMEOUT_MS)

        talon.selectProfileSlot(cls.PID_IDX, 0)
        # talon.config_kF(0, 0, cls.TIMEOUT_MS)
        # talon.config_kP(0, 0.2, cls.TIMEOUT_MS)
        # talon.config_kI(0, 0, cls.TIMEOUT_MS)
        # talon.config_kD(0, 0, cls.TIMEOUT_MS)

        talon.configMotionCruiseVelocity(15000, FeedForword.TIMEOUT_MS) #31140
        talon.configMotionAcceleration(6000, FeedForword.TIMEOUT_MS)

        talon.setSelectedSensorPosition(0, FeedForword.PID_IDX, FeedForword.TIMEOUT_MS)

    def reset(self):
        self.counter = 0
        self.right.setSelectedSensorPosition(0, FeedForword.PID_IDX, FeedForword.TIMEOUT_MS)
        self.left.setSelectedSensorPosition(0, FeedForword.PID_IDX, FeedForword.TIMEOUT_MS)
        self.findPath()
        self.setParm()


    def setParm(self):
        self._setParm(self.left)
        self._setParm(self.right)


    @classmethod
    def _setParm(cls, talon):
        sp = NetworkTables.getTable("/SmartDashboard/DriveAndTurn")
        kp = sp.getNumber("kp", 0.0)
        ki = sp.getNumber("ki", 0.0)
        kd = sp.getNumber("kd", 0.0)
        kf = sp.getNumber("kf", 0.0)
        talon.config_kF(0, kp, cls.TIMEOUT_MS)
        talon.config_kP(0, ki, cls.TIMEOUT_MS)
        talon.config_kI(0, kd, cls.TIMEOUT_MS)
        talon.config_kD(0, kf, cls.TIMEOUT_MS)



    def findPath(self):
        self.right_path = []
        self.left_path = []
        with open(os.path.join(os.path.dirname(__file__), "points_right_Talon.csv"), "r") as f:
            for line in f.readlines():
                self.right_path.append(float(line.replace("\n", "")))

        with open(os.path.join(os.path.dirname(__file__), "points_left_Talon.csv"), "r") as f:
            for line in f.readlines():
                self.left_path.append(float(line.replace("\n", "")))
        #path_generator = PathGenerator()
        # self.right_path = path_generator.right_path
        # self.left_path = path_generator.left_path


    def start(self):
        self.enable = True


    @timed_state(duration=12, must_finish=True)
    def drive(self):
        if self.enable:
            if self.counter < len(self.right_path):
                self.right.set(WPI_TalonSRX.ControlMode.MotionMagic, -1 * self.right_path[self.counter] / self.RIGHT_RATIO)
                self.left.set(WPI_TalonSRX.ControlMode.MotionMagic, self.left_path[self.counter] / self.LEFT_RATIO)
                self.counter += 1
            elif self.is_finished():
                self.enable = False
            else:
                self.right.set(WPI_TalonSRX.ControlMode.Position, -1 * self.right_path[self.counter - 1] / self.RIGHT_RATIO)
                self.left.set(WPI_TalonSRX.ControlMode.Position, self.left_path[self.counter - 1] / self.LEFT_RATIO)

            self.printer()

    def is_finished(self):
        return  ((-1 * self.right_path[self.counter - 1] / self.RIGHT_RATIO)
                 - self.right.getQuadraturePosition()) * self.RIGHT_RATIO < self.ERROR and \
                ((-1 * self.left_path[self.counter - 1] / self.RIGHT_RATIO)
                 - self.left.getQuadraturePosition()) * self.LEFT_RATIO < self.ERROR


    def printer(self):
        if self.counter > 0 and len(self.right_path) > 0:
            self.table.putNumber("right desired", self.right_path[self.counter - 1] / self.RIGHT_RATIO)
            self.table.putNumber("left desired", self.left_path[self.counter - 1] / self.LEFT_RATIO)

