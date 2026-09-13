from typing import override

from commands2 import Command

import src.commands.operation_constants as operation_consts
from src.subsystems.mechanisms.intake import Intake


class IntakeBalls(Command):
    def __init__(self, intake: Intake):
        super().__init__()

        self.intake = intake

        self.addRequirements(intake)

    @override
    def initialize(self):
        self.intake.set_lift_position(operation_consts.INTAKE_LIFT_POS_DOWN)
        self.intake.set_feed_speed(operation_consts.INTAKE_FEED_PWR)

    @override
    def execute(self):
        pass

    @override
    def end(self, interrupted: bool):
        self.intake.set_lift_position(operation_consts.INTAKE_LIFT_POS_UP)
        self.intake.stop_feed()
