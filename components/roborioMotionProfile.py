#
# import math
# import wpilib
# import pathfinder as pf
# from .DriveTrain import DriveTrain
# from pathfinder.followers import EncoderFollower
# import ctre
#
# class RoborioMotionProfile():
#     right_drive_talon:ctre.WPI_TalonSRX
#     left_drive_talon:ctre.WPI_TalonSRX
#
#     METER_SEC_TO_FEET_SEC = 3.2808398950131
#     MAX_SPEED = 2.2 * METER_SEC_TO_FEET_SEC # could be until 2.5
#
#
#     PID_IDX = 0
#     TIMEOUT_MS = 10
#     ITER_NUM = 1000
#     GEAR_RATIO = 16.73 #power speed 7.56
#     EVO_RATIO = 3
#     DIAMETER = 6 * 2.54 * math.pi
#     RIGHT_RATIO =  1.2263 / (GEAR_RATIO * DIAMETER * 100)
#     LEFT_RATIO = 1 / (EVO_RATIO * DIAMETER * 2 * 10)
#
#     def setup(self):
#         self.right = self.right_drive_talon
#         self.left = self.left_drive_talon
#
#         self.right.set
#
#
#         points = [pf.Waypoint(0, 2, 0),
#                   pf.Waypoint(3, 2, 0),
#                   pf.Waypoint(4, 3, math.radians(90)),
#                   pf.Waypoint(4.0, 6.0, math.radians(90))]
#         info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
#                                        dt=0.02,  # 50ms
#                                        max_velocity=self.MAX_SPEED,
#                                        max_acceleration=1.9685,
#                                        max_jerk=0.6562)
#         modifier = pf.modifiers.TankModifier(trajectory).modify(0.69)
#
#         left = EncoderFollower(modifier.getLeftTrajectory())
#         right = EncoderFollower(modifier.getRightTrajectory())
#
#
#         self
#
#
#
#         self.l_encoder.setDistancePerPulse(
#             (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
#
#
#
#
#
#
