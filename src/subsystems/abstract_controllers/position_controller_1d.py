from rev import PersistMode, ResetMode, SparkLowLevel, SparkMax, SparkMaxConfig


class Controller1d:
    def __init__(self, motor_id: int, motor_type: SparkLowLevel.MotorType):
        self.motor = SparkMax(motor_id, motor_type)
        self.encoder = self.motor.getAbsoluteEncoder()
        self.closed_loop = self.motor.getClosedLoopController()

        self.init_pos = self.encoder.getPosition()
        self.target_pos = self.init_pos

    def config(self, config: SparkMaxConfig) -> None:
        self.motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters)

    def get_target_pos(self) -> float:
        return self.target_pos

    def set_target_pos(self, target_pos: float, control_type: SparkLowLevel.ControlType) -> None:
        self.target_pos = self.init_pos + target_pos

        self.closed_loop.setSetpoint(self.target_pos, SparkLowLevel.ControlType.kPosition)

    def stop(self) -> None:
        self.motor.stopMotor()
