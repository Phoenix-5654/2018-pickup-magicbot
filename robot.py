import math
import numpy as np

import wpilib
import wpilib.drive
import wpilib.buttons
import ctre
from magicbot import MagicRobot
from networktables import NetworkTables
import pathfinder as pf

#from components.DriveTrain import DriveTrain
# from components.motion_profile import MotionProfile
#from components.rpm_counter import RPMCounter
from components.feed_forword import FeedForword


class MyRobot(MagicRobot):
   # drivetrain: DriveTrain
   #  motion_profile: MotionProfile
    # rpm_counter: RPMCounter
    feed_forword:FeedForword

    PID_IDX = 0
    TIMEOUT_MS = 10
    ITER_NUM = 1000
    GEAR_RATIO = 16.73 #power speed 7.56
    EVO_RATIO = 3
    DIAMETER = 6 * 2.54 * math.pi
    RIGHT_RATIO =  1.2263 / (GEAR_RATIO * DIAMETER * 100)
    LEFT_RATIO = 1 / (EVO_RATIO * DIAMETER * 2 * 10)

    def createObjects(self):

        NetworkTables.initialize()

        self.table = NetworkTables.getTable("SmartDashboard")

        self.right_drive_talon = ctre.WPI_TalonSRX(6)
        self.seocend_right_drive_talon = ctre.WPI_TalonSRX(5)


        self.left_drive_talon = ctre.WPI_TalonSRX(1)
        self.seocend_left_drive_talon = ctre.WPI_TalonSRX(2)

        self.seocend_right_drive_talon.follow(self.right_drive_talon)
        self.seocend_left_drive_talon.follow(self.left_drive_talon)


        #elf.drive = wpilib.drive.DifferentialDrive(leftMotor=self.left_drive_talon,
         #                                           rightMotor=self.right_drive_talon)

        self.left_drive_talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                self.PID_IDX, self.TIMEOUT_MS)
        self.right_drive_talon.configSelectedFeedbackSensor(ctre.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                self.PID_IDX, self.TIMEOUT_MS)

        self.left_encoder = self.left_drive_talon.getSensorCollection()
        self.right_encoder = self.right_drive_talon.getSensorCollection()

        self.drivetrain_solenoid = wpilib.DoubleSolenoid(2, 3)

        self.joystick = wpilib.Joystick(0)

        self.gear_change_btn = wpilib.buttons.JoystickButton(self.joystick, 1)

        self.presure_sensor = wpilib.AnalogInput(0)
        self.pressure_history = []

    def handle_presure(self, k = 32):
        '''
        function will calculate the mean pressure in k times and send it to the smartdashboard
        '''
        if len(self.pressure_history) == k:
            self.pressure_history.pop(0)
        self.pressure_history.append(self.presure_sensor.getVoltage())
        self.pressure = (50 * np.mean(self.pressure_history)) - 25
        self.table.putNumber("pressure", self.presure)


    def teleopInit(self):
#       self.right_encoder.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
#        self.left_encoder.setQuadraturePosition(newPosition=0, timeoutMs=self.TIMEOUT_MS)
        #self.motion_profile.reset()
        self.feed_forword.reset()


        # pass
    def teleopPeriodic(self):
        # if self.rpm_counter.enable:
        #     return None
        # if self.gear_change_btn.get():
        #     self.drivetrain.change_gear_mode()

        # self.drivetrain.move(-self.joystick.getY(), self.joystick.getX())

        self.table.putNumber("rightVal", self.right_encoder.getQuadraturePosition() * self.RIGHT_RATIO)
        self.table.putNumber("leftVal", self.left_encoder.getQuadraturePosition() * self.LEFT_RATIO)


if __name__ == '__main__':
    wpilib.run(MyRobot)