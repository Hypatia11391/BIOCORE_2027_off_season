from typing import override

from commands2 import Command
from wpilib import Joystick, Timer

import src.commands.operation_constants as operation_consts
from src.buttons import Buttons
from src.joysticks_axes import JoystickAxes
from src.subsystems.mechanisms.feed import Feed
from src.subsystems.mechanisms.intake import Intake
from src.subsystems.mechanisms.kicker import Kicker
from src.subsystems.mechanisms.shooter import Shooter


class OperateTelop(Command):
    def __init__(self, intake: Intake, feed: Feed, kicker: Kicker, shooter: Shooter, controller: Joystick) -> None:
        super().__init__()

        self.intake = intake
        self.feed = feed
        self.kicker = kicker
        self.shooter = shooter
        self.controller = controller

        self.addRequirements(intake, feed, kicker, shooter)

        self.intake_lift_state = operation_consts.IntakeLiftState.OFF.value
        self.intake_feed_state = operation_consts.IntakeFeedState.OFF.value

        self.time_at_target_speed = -1.0

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

        rt_shoot = self.controller.getRawAxis(JoystickAxes.RT.value)

        if abs(rt_shoot) > 0.08:
            left_shooter_speed = rt_shoot * operation_consts.HIGH_LEFT_RPM
            right_shooter_speed = rt_shoot * operation_consts.HIGH_RIGHT_RPM

            self.shooter.set_target_rpm(left_shooter_speed, right_shooter_speed)

            if self.shooter.is_at_target_rpm():
                self.kicker.set_kicker_speed(operation_consts.KICKER_POWER)
                self.feed.set_feed_speed(operation_consts.FEED_POWER)

                if self.time_at_target_speed < 0.0:
                    self.time_at_target_speed = Timer.getFPGATimestamp()

            else:
                self.feed.stop()
                self.kicker.stop()
                # self.shooter.stop()

        else:
            self.feed.stop()
            self.kicker.stop()
            self.shooter.stop()

    @override
    def end(self, interrupted: bool) -> None:
        self.intake.stop()
        self.feed.stop()
        self.kicker.stop()
        self.shooter.stop()

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
