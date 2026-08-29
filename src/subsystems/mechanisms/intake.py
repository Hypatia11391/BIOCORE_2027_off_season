from commands2 import Subsystem
from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax, SparkMaxConfig

import src.subsystems.mechanisms.intake_constants as intake_consts


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.intake_lift = SparkMax(intake_consts.INTAKE_LIFT_ID, SparkLowLevel.MotorType.kBrushless)
        self.intake_feed = SparkMax(intake_consts.INTAKE_FEED_ID, SparkLowLevel.MotorType.kBrushless)

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

        self.intake_lift.configureAsync(
            lift_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        self.lift_encoder = self.intake_lift.getEncoder()
        self.lift_loop = self.intake_lift.getClosedLoopController()

        self.init_pos = self.lift_encoder.getPosition()
        self.target_pos = self.init_pos
        # self.stall_detector = StallDetector()

        feed_config = SparkMaxConfig()
        feed_config.inverted(intake_consts.INTAKE_FEED_INVERTED)
        feed_config.setIdleMode(intake_consts.INTAKE_FEED_IDLE_MODE)
        feed_config.smartCurrentLimit(intake_consts.INTAKE_FEED_SMART_LIMIT)
        feed_config.voltageCompensation(intake_consts.INTAKE_FEED_VOLTAGE_COMPENSATION)

        self.intake_feed.configureAsync(
            feed_config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    # In degrees
    def set_lift_position(self, target_pos: float) -> None:
        # self.target_pos = max(intake_consts.INTAKE_LIFT_UP_POS, min(target_pos, intake_consts.INTAKE_LIFT_DOWN_POS))  # Clamp TODO: once have more accurate method of position detection add back
        self.target_pos = target_pos * ((48 * (50 / 18)) / 360) + self.init_pos

        current_pos = self.lift_encoder.getPosition()
        # self.stall_detector.update(current_pos)
        if abs(current_pos - self.target_pos) < intake_consts.INTAKE_LIFT_POSITION_THRESHOLD:  # or self.stall_detector.is_stalling(self.target_pos):
            self.target_pos = current_pos

        self.lift_loop.setSetpoint(self.target_pos, SparkLowLevel.ControlType.kPosition)

    # Speed between -1, 1
    def set_feed_speed(self, speed: float) -> None:
        self.intake_feed.set(speed)

    def stop(self) -> None:
        self.intake_lift.stopMotor()
        self.intake_feed.stopMotor()

    # def periodic(self) -> None:
    #     print(self.target_pos, self.target_pos - self.lift_encoder.getPosition(), self.intake_lift.getAppliedOutput())


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
