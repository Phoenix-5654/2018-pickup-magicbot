import wpilib


class Grippers:
    l_gripper_motor: wpilib.VictorSP
    r_gripper_motor: wpilib.VictorSP

    INTAKE = (0.7, -0.7)
    EXHAUST = (-0.7, 0.7)
    STOP = (0, 0)

    def __init__(self):
        self.intake_signal = self.exhaust_signal = False

    def intake(self):
        self.intake_signal = True

    def exhaust(self):
        self.exhaust_signal = True

    def set_motors(self, l_motor_percent, r_motor_percent):

        self.l_gripper_motor.set(l_motor_percent)
        self.r_gripper_motor.set(r_motor_percent)

    def execute(self):
        if self.intake_signal:
            self.set_motors(*self.INTAKE)
        elif self.exhaust_signal:
            self.set_motors(*self.EXHAUST)
        else:
            self.set_motors(*self.STOP)

        self.intake_signal = self.exhaust_signal = False
