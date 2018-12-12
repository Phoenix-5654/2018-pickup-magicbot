import wpilib
from magicbot import tunable
from components.drivetrain import Drivetrain


class AngleController:
    drivetrain: Drivetrain
    kP = tunable(default=0.2)
    kI = tunable(default=0)
    kD = tunable(default=0)

    def __init__(self):
        self.pid_output = 0

    def setup(self):
        self.pid = wpilib.PIDController(self.kP, self.kI, self.kD,
                                        source=self.drivetrain.gyro,
                                        output=self)
        self.pid.setOutputRange(-0.4, 0.4)
        self.pid.setInputRange(-180, 180)
        self.pid.setContinuous()
        self.pid.setAbsoluteTolerance(2)

    def align_to(self, angle):
        self.pid.setSetpoint(angle)
        self.pid.enable()

    def is_enabled(self):
        return self.pid.enabled

    def reset_angle(self):
        self.drivetrain.gyro.reset()

    def pidWrite(self, output):
        self.pid_output = output

    def execute(self):
        if self.pid.enabled:
            if self.pid.onTarget():
                self.pid.disable()
            else:
                self.drivetrain.move_x(self.pid_output)

