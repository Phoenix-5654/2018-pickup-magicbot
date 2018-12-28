from magicbot import AutonomousStateMachine, timed_state

class Step(AutonomousStateMachine):
    MODE_NAME = 'Drive and Turn'
    DEFAULT = True

    @timed_state(duration=2, next_state="do_something", first=True)
    def dont_do_something(self):
        """This happens first"""
        pass