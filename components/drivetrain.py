import wpilib.drive


class Drivetrain:
    SPD_STATE = 1
    STR_STATE = 2

    drive: wpilib.drive.DifferentialDrive
    solenoid: wpilib.DoubleSolenoid
    gyro: wpilib.AnalogGyro

    def __init__(self):
        self.x = self.y = 0
        self.speed_state = self.strength_state = self.overload_joystick = False

    def set_speed_state(self):
        self.speed_state = True

    def set_strength_state(self):
        self.strength_state = True

    def move_x(self, x):
        self.x = x

    def move_y(self, y):
        self.y = y

    def move(self, x, y):
        self.move_x(x)
        self.move_y(y)

    def execute(self):
        self.drive.arcadeDrive(self.y, self.x)

        if self.speed_state:
            self.solenoid.set(self.solenoid.Value(self.SPD_STATE))
        elif self.strength_state:
            self.solenoid.set(self.solenoid.Value(self.STR_STATE))

        self.x = self.y = 0
        self.speed_state = self.strength_state = self.overload_joystick = False
