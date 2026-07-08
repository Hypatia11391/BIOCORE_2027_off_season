from commands2 import Command
from wpilib import Joystick

from typing import override

from src.subsytems.mechanisms.intake import Intake
import commands.operation_constants as operation_consts
from src.buttons import Buttons


class OperateTelop(Command):
    def __init__(self, intake: Intake, controller: Joystick):
        super().__init__()

        self.intake = intake
        self.controller = controller

        self.addRequirements(intake)

        self.intake_lift_state = operation_consts.IntakeLiftState.OFF.value
        self.intake_feed_state = operation_consts.IntakeFeedState.OFF.value

    @override
    def initialize(self) -> None:
        self.intake.stop()

    @override
    def execute(self) -> None:
        self.update_states()

        match self.intake_lift_state:
            case operation_consts.IntakeLiftState.OFF.value:
                self.intake.set_lift_position(0)

            case operation_consts.IntakeLiftState.UP.value:
                self.intake.set_lift_position(operation_consts.INTAKE_LIFT_POS_UP)

            case operation_consts.IntakeLiftState.DOWN.value:
                self.intake.set_lift_position(operation_consts.INTAKE_LIFT_POS_DOWN)

        match self.intake_feed_state:
            case operation_consts.IntakeFeedState.OFF.value:
                self.intake.set_feed_speed(0)

            case operation_consts.IntakeFeedState.IN.value:
                self.intake.set_feed_speed(operation_consts.INTAKE_FEED_PWR)

            case operation_consts.IntakeFeedState.OUT.value:
                self.intake.set_feed_speed(-operation_consts.INTAKE_FEED_PWR)

    @override
    def end(self, interrupted: bool) -> None:
        self.intake.stop()

    @override
    def isFinished(self) -> bool:
        return False

    def update_states(self) -> None:
        a_down = self.controller.getRawButtonPressed(Buttons.A.value)
        y_up = self.controller.getRawButtonPressed(Buttons.Y.value)

        if a_down:
            if self.intake_lift_state != operation_consts.IntakeLiftState.DOWN.value:
                self.intake_lift_state = operation_consts.IntakeLiftState.DOWN.value

            else:
                self.intake_lift_state = operation_consts.IntakeLiftState.OFF.value

        if y_up:
            if self.intake_lift_state != operation_consts.IntakeLiftState.UP.value:
                self.intake_lift_state = operation_consts.IntakeLiftState.UP.value

            else:
                self.intake_lift_state = operation_consts.IntakeLiftState.OFF.value

        lb_out = self.controller.getRawButtonPressed(Buttons.LB.value)
        rb_in = self.controller.getRawButtonPressed(Buttons.RB.value)

        if lb_out:
            if self.intake_feed_state != operation_consts.IntakeFeedState.OUT.value:
                self.intake_feed_state = operation_consts.IntakeFeedState.OUT.value

            else:
                self.intake_feed_state = operation_consts.IntakeFeedState.OFF.value

        if rb_in:
            if self.intake_feed_state != operation_consts.IntakeFeedState.IN.value:
                self.intake_feed_state = operation_consts.IntakeFeedState.IN.value

            else:
                self.intake_feed_state = operation_consts.IntakeFeedState.OFF.value
