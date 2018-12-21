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

    def createObjects(self):

        self.lr_motor = ctre.WPI_TalonSRX(1)
        self.lf_motor = ctre.WPI_TalonSRX(2)

        self.rr_motor = ctre.WPI_TalonSRX(5)
        self.rf_motor = ctre.WPI_TalonSRX(6)

        self.left = wpilib.SpeedControllerGroup(self.lf_motor, self.lr_motor)
        self.right = wpilib.SpeedControllerGroup(self.rf_motor, self.rr_motor)

        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)

        self.drivetrain_gyro = wpilib.AnalogGyro(1)

        self.drivetrain_solenoid = wpilib.DoubleSolenoid(2, 3)

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

    def teleopPeriodic(self):
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
        if self.button_10.get():
            self.drivetrain.set_speed_state()
        elif self.button_11.get():
            self.drivetrain.set_strength_state()


if __name__ == '__main__':
    wpilib.run(MyRobot, physics_enabled=True)
