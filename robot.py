import math
import time

import ctre
import magicbot
import wpilib
import wpilib.buttons
import wpilib.drive

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.grippers import Grippers
from components.handles import Handles

class MyRobot(magicbot.MagicRobot):
    drivetrain: Drivetrain
    elevator: Elevator
    handles: Handles
    grippers: Grippers

    WHL_DIAMETER = 15.24
    WHL_CIRC = WHL_DIAMETER * math.pi

    ENCODER_RES = 1024

    QUAD_MULTIPLIER = 4

    STRENGTH_GEAR_RATIO = 16.5

    DIST_TO_TICKS = WHL_CIRC / (ENCODER_RES * QUAD_MULTIPLIER *
                                STRENGTH_GEAR_RATIO)

    ABS_TOLERANCE = 3 / DIST_TO_TICKS

    MAX_SPEED = 3.5    #0.65
    MIN_SPEED = -MAX_SPEED



    TIME_OUT_MS = 10
    PID_LOOP_IDX = 0
    SLOT_IDX = 0

    def createObjects(self):

        self.left_rear_motor = ctre.WPI_TalonSRX(2)
        self.left_front_motor = ctre.WPI_TalonSRX(1)

        self.right_rear_motor = ctre.WPI_TalonSRX(5)
        self.right_front_motor = ctre.WPI_TalonSRX(6)

        self.left_rear_motor.follow(self.left_front_motor)
        self.right_rear_motor.follow(self.right_front_motor)


        # self.right_front_motor.\
        #     configSelectedFeedbackSensor(feedbackDevice=ctre.WPI_TalonSRX.
        #                                  FeedbackDevice.CTRE_MagEncoder_Relative,
        #                                  pidIdx=self.PID_LOOP_IDX, timeoutMs=self.TIME_OUT_MS)
        #
        # self.left_front_motor.\
        #     configSelectedFeedbackSensor(feedbackDevice=ctre.WPI_TalonSRX.
        #                                  FeedbackDevice.CTRE_MagEncoder_Relative,
        #                                  pidIdx=self.PID_LOOP_IDX, timeoutMs=self.TIME_OUT_MS)
        #
        # self.right_front_motor.setSensorPhase(True)
        # self.left_front_motor.setSensorPhase(False)

        #self.rf_motor.setOutputRange(self.MIN_SPEED, self.MAX_SPEED)
        #self.lr_motor.setOutputRange(self.MIN_SPEED, self.MAX_SPEED)


        #self.left = wpilib.SpeedControllerGroup(self.lf_motor, self.lr_motor)
        #self.right = wpilib.SpeedControllerGroup(self.rf_motor, self.rr_motor)

        self.left = wpilib.SpeedControllerGroup(self.left_front_motor)
        self.right = wpilib.SpeedControllerGroup(self.right_front_motor)
        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)

        self.drivetrain_solenoid = wpilib.DoubleSolenoid(2, 3)
        self.drivetrain_gyro = wpilib.AnalogGyro(1)

        self.elevator_motor = wpilib.VictorSP(2)

        self.top_switch = wpilib.DigitalInput(4)
        self.bot_switch = wpilib.DigitalInput(5)

        self.handles_solenoid = wpilib.DoubleSolenoid(0, 1)

        self.l_gripper_motor = wpilib.VictorSP(0)
        self.r_gripper_motor = wpilib.VictorSP(1)

        self.joystick = wpilib.Joystick(0)

        self.trigger = wpilib.buttons.JoystickButton(self.joystick, 1)
        self.button_2 = wpilib.buttons.JoystickButton(self.joystick, 2)
        self.button_3 = wpilib.buttons.JoystickButton(self.joystick, 3)
        self.button_4 = wpilib.buttons.JoystickButton(self.joystick, 4)
        self.button_5 = wpilib.buttons.JoystickButton(self.joystick, 5)
        self.button_7 = wpilib.buttons.JoystickButton(self.joystick, 7)
        self.button_10 = wpilib.buttons.JoystickButton(self.joystick, 10)
        self.button_11 = wpilib.buttons.JoystickButton(self.joystick, 11)
        self.profile_btn = wpilib.buttons.JoystickButton(self.joystick, 9)

    def teleopInit(self):
        self.start_time = time.time()
        self.counter = 0

    def teleopPeriodic(self):
        # if self.counter < 100:
        #     self.counter += 1
        #     self.drivetrain.move_y(1)
        # elif self.counter==100:
        #     self.end_time = time.time()
        #     print("rpm right =", self.right_front_motor.getAnalogIn() / (self.end_time - self.start_time))
        #     print("rpm left =", self.left_front_motor.getAnalogIn() / (self.end_time - self.start_time))
        #     raise("exepected exception")

        return None


        if not self.drivetrain.overload_joystick:
            self.drivetrain.move(self.joystick.getX(), -self.joystick.getY())

        if self.button_2.get():
            self.elevator.lower_elevator()
        elif self.button_3.get():
            self.elevator.raise_elevator()
        if self.button_5.get():
            self.handles.change_handles_state()
        if self.trigger.get():
            self.grippers.intake()
        elif self.button_4.get():
            self.grippers.exhaust()
        if self.button_7.get():
            self.angle_ctrl.align_to(0)
        if self.button_10.get():
            self.drivetrain.set_speed_state()
        elif self.button_11.get():
            self.drivetrain.set_strength_state()
        if self.profile_btn.get():
            self.drivetrain.motion_profile()


if __name__ == '__main__':
    wpilib.run(MyRobot)
