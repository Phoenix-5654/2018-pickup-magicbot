from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.drivetrains import TwoMotorDrivetrain
from pyfrc.physics.units import units
import math

class PhysicsEngine():
    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.position = 0
        return None
        bumper_width = 3.25 * units.inch
        self.drivetrain = tankmodel.TankModel.theory(
            motor_config=motor_cfgs.MOTOR_CFG_CIM,
            robot_mass=60 * units.kg,
            gearing=16.37, #power speed 7.56
            nmotors=2,
            x_wheelbase=68 * units.cm,
            robot_width=70 * units.cm,
            robot_length=70 * units.cm,
            wheel_diameter= 6 * units.inch,
            vintercept=1.3 * units.volts,
            timestep=5 * units.ms
        )

    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        # x = hal_data["joysticks"][0]["axes"][0]
        # y = hal_data["joysticks"][0]["axes"][1]
        # angle = 0
        # if x != 0:
        #     angle = math.degrees(math.atan(y / x))
        # self.physics_controller.distance_drive(x, y, angle)
        # left = hal_data['CAN'][1]['value']
        # right = hal_data['CAN'][2]['value']
        # x, y, angle = self.drivetrain.get_distance(left, right, tm_diff)
        # self.physics_controller.distance_drive(x, y, angle)
        # update position (use tm_diff so the rate is constant)

        try:
            talon_data = hal_data['CAN'][1]
        except (KeyError, IndexError):
            # talon must not be initialized yet
            return

            # encoder increments speed mutiplied by the time by some constant
            # -> must be an integer
        speed = int(4096 * 4 * talon_data['value'] * tm_diff)
        talon_data['quad_position'] += speed
        talon_data['quad_velocity'] = speed

