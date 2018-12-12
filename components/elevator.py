import wpilib


class Elevator:
    UP_SPEED = 0.7
    DOWN_SPEED = -UP_SPEED
    STOP = 0

    elevator_motor: wpilib.VictorSP
    top_switch: wpilib.DigitalInput
    bot_switch: wpilib.DigitalInput

    def __init__(self):
        self.go_up = self.go_down = False

    def raise_elevator(self):
        self.go_up = True

    def lower_elevator(self):
        self.go_down = True

    def is_at_top(self):
        return self.top_switch.get()

    def is_at_bot(self):
        return self.bot_switch.get()

    def execute(self):
        if self.go_up and not self.is_at_top():
            self.elevator_motor.set(self.UP_SPEED)
        elif self.go_down and not self.is_at_bot():
            self.elevator_motor.set(self.DOWN_SPEED)
        else:
            self.elevator_motor.set(self.STOP)

        self.go_up = self.go_down = False
