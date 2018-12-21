import os.path
import pickle

import ctre
import pathfinder as pf
import wpilib
from magicbot import AutonomousStateMachine, state

from components.drivetrain import Drivetrain


class DriveAndTurn(AutonomousStateMachine):
    MODE_NAME = 'Drive and Turn'
    DEFAULT = True

    WHEEL_DIAMETER = 0.5  # 6 inches
    ENCODER_COUNTS_PER_REV = 1024 * 4

    MAX_VELOCITY = 5  # ft/s
    MAX_ACCELERATION = 6

    lf_motor: ctre.WPI_TalonSRX
    rf_motor: ctre.WPI_TalonSRX
    drivetrain: Drivetrain

    pickle_file = os.path.join(os.path.dirname(__file__), 'trajectory.pickle')

    @state(first=True)
    def setup(self):
        print("setting")
        if wpilib.RobotBase.isSimulation():
            points = [
                pf.Waypoint(0, 0, 0),
                pf.Waypoint(9, 5, 0)
            ]

            info, trajectory = pf.generate(
                points,
                pf.FIT_HERMITE_CUBIC,
                pf.SAMPLES_HIGH,
                dt=0.03,
                max_velocity=self.MAX_VELOCITY,
                max_acceleration=self.MAX_ACCELERATION,
                max_jerk=120.0,
            )

            with open(self.pickle_file, 'wb') as fp:
                pickle.dump(trajectory, fp)
        else:
            with open(self.pickle_file, 'rb') as fp:
                trajectory = pickle.load(fp)

        modifier = pf.modifiers.TankModifier(trajectory).modify(2.264)

        left = modifier.getLeftTrajectory()
        right = modifier.getRightTrajectory()

        left_follower = pf.followers.EncoderFollower(left)
        left_follower.configureEncoder(
            self.lf_motor.getQuadraturePosition(), self.ENCODER_COUNTS_PER_REV,
            self.WHEEL_DIAMETER
        )
        left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)

        right_follower = pf.followers.EncoderFollower(right)
        right_follower.configureEncoder(
            self.rf_motor.getQuadraturePosition(), self.ENCODER_COUNTS_PER_REV,
            self.WHEEL_DIAMETER
        )
        right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / self.MAX_VELOCITY, 0)

        self.left_follower = left_follower
        self.right_follower = right_follower

        if wpilib.RobotBase.isSimulation():
            from pyfrc.sim import get_user_renderer

            renderer = get_user_renderer()
            if renderer:
                renderer.draw_pathfinder_trajectory(
                    left, color="#0000ff", offset=(-1, 0)
                )
                renderer.draw_pathfinder_trajectory(
                    modifier.source, color="#00ff00", show_dt=1.0,
                    dt_offset=0.0
                )
                renderer.draw_pathfinder_trajectory(
                    right, color="#0000ff", offset=(1, 0)
                )

        self.next_state_now('drive_and_turn')

    @state
    def drive_and_turn(self):
        while not self.left_follower.isFinished() and not self.right_follower.isFinished():
            l = self.left_follower.calculate(
                self.lf_motor.getQuadraturePosition())
            r = self.right_follower.calculate(
                self.rf_motor.getQuadraturePosition())

            gyro_heading = (
                -self.drivetrain.gyro.getAngle()
            )
            desired_heading = pf.r2d(
                self.left_follower.getHeading()
            )

            angle_difference = pf.boundHalfDegrees(
                desired_heading - gyro_heading)
            turn = 5 * (-1.0 / 80.0) * angle_difference

            l += turn
            r -= turn

            self.drivetrain.drive.tankDrive(l, r)
        self.done()
