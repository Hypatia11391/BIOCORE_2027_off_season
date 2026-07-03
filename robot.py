import wpilib
import wpilib.drive as drive
import rev


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        print("INFO: Robot initiation sequence")

        self.left_front_drive = rev.SparkMax(9, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_front_drive = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        self.left_rear_drive = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
        self.right_rear_drive = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)

        self.robot_drive = drive.MecanumDrive(
            self.left_front_drive,
            self.left_rear_drive,
            self.right_front_drive,
            self.right_rear_drive,
        )

        self.controller = wpilib.Joystick(0)

        self.timer = wpilib.Timer()

        self.i = 0

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        forward_speed = -self.controller.getRawAxis(1)
        strafe_speed = self.controller.getRawAxis(0)
        turn_speed = -self.controller.getRawAxis(2)

        if not self.i:
            print(f"{forward_speed=}\n{strafe_speed=}\n{turn_speed=}")
            self.i = 10

        else:
            self.i -= 1

        forward_speed = 0 if abs(forward_speed) < 0.05 else forward_speed
        strafe_speed = 0 if abs(strafe_speed) < 0.05 else strafe_speed
        turn_speed = 0 if abs(turn_speed) < 0.05 else turn_speed

        speed_scaler: float = 0.25

        self.robot_drive.driveCartesian(
            forward_speed * speed_scaler,
            strafe_speed * speed_scaler,
            turn_speed * speed_scaler,
        )

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

    def disabledPeriodic(self):
        pass

    def robotPeriodic(self):
        pass

    def simulationPeriodic(self):
        pass
