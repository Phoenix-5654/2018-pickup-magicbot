import wpilib.drive
import ctre
import ctre.trajectorypoint
import csv

class Drivetrain:
    SPD_STATE = 1
    STR_STATE = 2

    MIN_POINTS_IN_TALON = 100
    BTN_BUFFER_CNT_ID = 3

    drive: wpilib.drive.DifferentialDrive
    solenoid: wpilib.DoubleSolenoid
    gyro: wpilib.AnalogGyro

    def __init__(self):
        self.x = self.y = 0
        self.speed_state = self.strength_state = self.overload_joystick = False
        self.enable_motion_profile = self.is_motion_profile_enabled= False
        #
        # self.right = ctre.WPI_TalonSRX(2)
        # self.left = ctre.WPI_TalonSRX(1)
        #
        self.left = self.drive.leftMotor
        self.left.__class__ = ctre.WPI_TalonSRX
        self.left = self.drive.rightMotor
        self.right.__class__ = ctre.WPI_TalonSRX

        self.right.changeMotionControlFramePeriod(5)
        self.left.changeMotionControlFramePeriod(5)

        #self.setPIDF(self.right)
        #self.setPIDF(self.left)

        #self.fillPoints("points_left_Talon.csv", self.left)
        #self.fillPoints("points_right_Talon.csv", self.right)


    @staticmethod
    def setPIDF(talon):
        talon.config_kF(1)
        talon.config_kP(0)
        talon.config_kI(0)
        talon.config_kD(0)

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

    def motion_profile(self):
        self.enable_motion_profile = True


    def reset(self):
        self.right.clearMotionProfileTrajectories()
        self.left.clearMotionProfileTrajectories()

        self.stop_event.set()

        self.left.pushMotionProfileTrajectory()

    # @staticmethod
    # def fillPoints(filename, talon):
    #     talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(velocity=0, isLastPoint=False,
    #                                                                                zeroPos=True, timeDur=10))
    #     with open(filename, "r") as csv_file:
    #         csv_reader = csv.Reader(csv_file)
    #
    #         for line in csv_reader:
    #             talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(position=line[0],
    #                                                                                        velocity=line[1],
    #                                                                                        timeDur=line[2],
    #                                                                                        isLastPoint=False,
    #                                                                                        zeroPos=False,
    #                                                                                        profileSlotSelect0=0,
    #                                                                                        profileSlotSelect1=0,))
    #
    #     talon.pushMotionProfileTrajectory(ctre.trajectorypoint.TrajectoryPoint(velocity=0, isLastPoint=True))
    #
    #     for i in range(Drivetrain.MIN_POINTS_IN_TALON):
    #         talon.processMotionProfileBuffer()

    def execute(self):
        if self.enable_motion_profile and not self.is_motion_profile_enabled:
            self.is_motion_profile_enabled = True
            self.left.controlMode(ctre.ControlMode.MotionProfile)
            self.right.controlMode(ctre.ControlMode.MotionProfile)
            for i in range(3):
                self.left.processMotionProfileBuffer()
                self.right.processMotionProfileBuffer()
            return None

        elif not self.enable_motion_profile:
            self.is_motion_profile_enabled = False

        self.enable_motion_profile = False

        self.left.controlMode(ctre.ControlMode.PercentOutput)
        self.right.controlMode(ctre.ControlMode.PercentOutput)
        self.drive.arcadeDrive(self.y, self.x)
        if self.speed_state:
            self.solenoid.set(self.solenoid.Value(self.SPD_STATE))
        elif self.strength_state:
            self.solenoid.set(self.solenoid.Value(self.STR_STATE))

        self.x = self.y = 0
        self.speed_state = self.strength_state = self.overload_joystick = False
