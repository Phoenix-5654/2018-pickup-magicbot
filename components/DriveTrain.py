import wpilib.drive
import wpilib

class DriveTrain():
    drive:wpilib.drive.DifferentialDrive
    drivetrain_solenoid:wpilib.DoubleSolenoid


    SPD_STATE = 1
    STR_STATE = 2

    def __init__(self):
        self.x = 0
        self.y = 0
        self.change_gear = self.changed_gear = False
        self.solenoid_position = self.SPD_STATE
        self.use_joystick = False

    def move(self, x, y):
        self.x = x
        self.y = y

    def moveY(self, y):
        self.y = y
        self.x = 0

    def moveX(self, x):
        self.x = x
        self.y = 0

    def change_gear_mode(self):
        self.change_gear = True

    def _change_gear(self):
        if self.change_gear and not self.changed_gear:
            self.solenoid_position = 3 - self.solenoid_position
            self.drivetrain_solenoid.set(wpilib.DoubleSolenoid.Value(self.solenoid_position))
            self.changed_gear = True
        elif not self.change_gear:
            self.changed_gear = False
        self.change_gear = False

    def execute(self):
        if self.use_joystick:
            self._change_gear()
            self.drive.arcadeDrive(self.x, self.y, squaredInputs=True)




