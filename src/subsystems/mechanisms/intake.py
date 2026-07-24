from commands2 import Subsystem
from rev import SparkLowLevel, SparkMaxConfig

import src.subsystems.mechanisms.intake_constants as intake_consts
from src.subsystems.abstract_controllers.position_controller_1d import Controller1d


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intake_lift = Controller1d(intake_consts.INTAKE_LIFT_ID, SparkLowLevel.MotorType.kBrushless)
        self.intake_feed = Controller1d(intake_consts.INTAKE_FEED_ID, SparkLowLevel.MotorType.kBrushless)

        lift_config = SparkMaxConfig()
        lift_config.inverted(intake_consts.INTAKE_LIFT_INVERTED)
        lift_config.setIdleMode(intake_consts.INTAKE_LIFT_IDLE_MODE)
        lift_config.smartCurrentLimit(intake_consts.INTAKE_LIFT_SMART_LIMIT)
        lift_config.voltageCompensation(intake_consts.INTAKE_LIFT_VOLTAGE_COMPENSATION)
        lift_config.encoder.velocityConversionFactor(intake_consts.INTAKE_LIFT_ENCODER_VELOCITY_CONVERSION_FACTOR)
        lift_config.encoder.positionConversionFactor(intake_consts.INTAKE_LIFT_ENCODER_POSITION_CONVERSION_FACTOR)
        lift_config.closedLoop.setFeedbackSensor(intake_consts.INTAKE_LIFT_CLOSED_LOOP_FEEDBACK_SENSOR)
        lift_config.closedLoop.pid(intake_consts.INTAKE_LIFT_kP, intake_consts.INTAKE_LIFT_kI, intake_consts.INTAKE_LIFT_kD)
        lift_config.closedLoop.outputRange(-2.0, 2.0)

        self.intake_lift.config(lift_config)

        feed_config = SparkMaxConfig()
        feed_config.inverted(intake_consts.INTAKE_FEED_INVERTED)
        feed_config.setIdleMode(intake_consts.INTAKE_FEED_IDLE_MODE)
        feed_config.smartCurrentLimit(intake_consts.INTAKE_FEED_SMART_LIMIT)
        feed_config.voltageCompensation(intake_consts.INTAKE_FEED_VOLTAGE_COMPENSATION)

        self.intake_feed.config(feed_config)

    # In degrees
    def set_lift_position(self, target_pos: float) -> None:
        self.intake_lift.set_target_pos(target_pos, SparkLowLevel.ControlType.kPosition)

    # Speed between -1, 1
    def set_feed_speed(self, speed: float) -> None:
        self.intake_feed.set_target_pos(speed, SparkLowLevel.ControlType.kVelocity)

    def stop(self) -> None:
        self.intake_lift.stop()
        self.intake_feed.stop()


# class StallDetector:
#     def __init__(self):
#         self.current_poss = deque(maxlen=50)
#         self.target_poss = deque(maxlen=50)

#     def update(self, current_pos, target_pos):
#         self.current_poss.append(current_pos)
#         self.target_poss.append(target_pos)

#     def is_stalling(self, target_pos) -> bool:
#         result = len(self.current_poss) == self.current_poss.maxlen and max(self.current_poss) - min(self.current_poss) < 1 and max(self.target_poss) - min(self.target_poss) < 1 and self.current_poss[-1] != self.target_poss[-1]
#         if result:
#             print("MOTOR STALLING DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#             print(f"{self.positions[-1]=}, {target_pos=}")
#         return result
