from components.drivetrain import Drivetrain


class DistanceController:
    drivetrain: Drivetrain

    MAX_SPEED = 0.65
    ACCELERATION = 0.15

    def __init__(self, desired_distance, init_speed=0.2):
        self.dest = desired_distance
        self.speed = init_speed
        self.drivetrain.overload_joystick = True
        self.drivetrain.

    def execute(self):
        self.drivetrain.move_y(self.speed)
        if self.speed <= self.MAX_SPEED:
            self.speed *= self.ACCELERATION
