import wpilib


class Handles:
    CLOSED = 1

    solenoid: wpilib.DoubleSolenoid

    def __init__(self):
        self.state = wpilib.DoubleSolenoid.Value(self.CLOSED)
        self.change_state = self.changed_state = False

    def change_handles_state(self):
        self.change_state = True

    def execute(self):
        if self.change_state and not self.changed_state:
            self.state = 2 - self.state + 1
            self.solenoid.set(self.state)
            self.changed_state = True
        elif not self.change_state:
            self.changed_state = False

        self.change_state = False
